#include "zcm/transport.h"
#include "zcm/transport_registrar.h"
#include "zcm/transport_register.hpp"

#include "zcm/zcm-cpp.hpp"
#include "zcm/util/debug.h"

#include "util/Types.hpp"
#include "util/TimeUtil.hpp"

#include <cstdio>
#include <cassert>
#include <unordered_map>
#include <mutex>
#include <unistd.h>
#include <limits.h>

#define ZCM_TRANS_CLASSNAME TransportFile
#define MTU (SSIZE_MAX)

using namespace std;

struct ZCM_TRANS_CLASSNAME : public zcm_trans_t
{
    zcm::LogFile *log = nullptr;
    unordered_map<string, string> options;

    string mode = "r";
    double speed = 1.0;

    u64 lastMsgUtime = 0;
    u64 lastDispatchUtime = 0;

    string *findOption(const string& s)
    {
        auto it = options.find(s);
        if (it == options.end()) return nullptr;
        return &it->second;
    }

    ZCM_TRANS_CLASSNAME(zcm_url_t *url)
    {
        trans_type = ZCM_BLOCKING;
        vtbl = &methods;

        // build 'options'
        auto *opts = zcm_url_opts(url);
        for (size_t i = 0; i < opts->numopts; i++)
            options[opts->name[i]] = opts->value[i];

        string* speedStr = findOption("speed");
        if (speedStr) {
            speed = atof(speedStr->c_str());
            if (speed <= 0) {
                ZCM_DEBUG("Expected double argument for 'speed'");
                return;
            }
        }

        string* modeStr = findOption("mode");
        if (modeStr) {
            mode = string(*modeStr);
            if (!(mode == "r" || mode == "w" || mode == "a")) {
                ZCM_DEBUG("Expected r|w|a as mode argument for zcm file");
                return;
            }
        }

        auto filename = zcm_url_address(url);
        ZCM_DEBUG("Opening zcm logfile: \"%s\"", filename);
        log = new zcm::LogFile(filename, string(mode));
        if (!log->good()) {
            fprintf(stderr, "Unable to open logfile %s\n", filename);
            return;
        }
    }

    ~ZCM_TRANS_CLASSNAME()
    {
        if (log) delete log;
    }

    bool good()
    {
        return log ? log->good() : false;
    }

    /********************** METHODS **********************/
    size_t get_mtu()
    {
        return MTU;
    }

    int sendmsg(zcm_msg_t msg)
    {
        assert(good());
        assert(mode == "w" || mode == "a");

        if (msg.len > get_mtu())
            return ZCM_EINVALID;

        zcm::LogEvent le;
        le.timestamp = msg.utime;
        le.datalen = msg.len;
        le.channel = string(msg.channel);
        le.data = msg.buf;

        log->writeEvent(&le);

        return ZCM_EOK;
    }

    int recvmsg_enable(const char *channel, bool enable)
    {
        return ZCM_EOK;
    }

    int recvmsg(zcm_msg_t *msg, int timeout)
    {
        assert(mode == "r");
        if (!good()) {
            // TODO Build in a way for a transport to tell zcm that an "error"
            //      has occurred. Not sure what to do here since this function
            //      has no way of communicating to the caller that this function
            //      shouldn't be called anymore
            usleep(timeout);
            return ZCM_ECONNECT;
        }

        const zcm::LogEvent* le = log->readNextEvent();
        if (!le) {
            delete log;
            log = nullptr;
            return ZCM_ECONNECT;
        }

        msg->utime = le->timestamp;
        msg->channel = le->channel.c_str();
        msg->len = le->datalen;
        msg->buf = le->data;

        u64 now = TimeUtil::utime();

        if (lastMsgUtime == 0)
            lastMsgUtime = msg->utime;

        if (lastDispatchUtime == 0)
            lastDispatchUtime = now;

        u64 localDiff = now - lastDispatchUtime;
        u64 logDiff = msg->utime - lastMsgUtime;
        u64 logDiffSpeed = logDiff / this->speed;
        u64 diff = logDiffSpeed > localDiff ? logDiffSpeed - localDiff : 0;

        if (diff > 0)
            usleep(diff);

        lastDispatchUtime = TimeUtil::utime();
        lastMsgUtime = msg->utime;

        return ZCM_EOK;
    }

    /********************** STATICS **********************/
    static zcm_trans_methods_t methods;
    static ZCM_TRANS_CLASSNAME *cast(zcm_trans_t *zt)
    {
        assert(zt->vtbl == &methods);
        return (ZCM_TRANS_CLASSNAME*)zt;
    }

    static size_t _get_mtu(zcm_trans_t *zt)
    { return cast(zt)->get_mtu(); }

    static int _sendmsg(zcm_trans_t *zt, zcm_msg_t msg)
    { return cast(zt)->sendmsg(msg); }

    static int _recvmsg_enable(zcm_trans_t *zt, const char *channel, bool enable)
    { return cast(zt)->recvmsg_enable(channel, enable); }

    static int _recvmsg(zcm_trans_t *zt, zcm_msg_t *msg, int timeout)
    { return cast(zt)->recvmsg(msg, timeout); }

    static void _destroy(zcm_trans_t *zt)
    { delete cast(zt); }

    static const TransportRegister reg;
};

zcm_trans_methods_t ZCM_TRANS_CLASSNAME::methods = {
    &ZCM_TRANS_CLASSNAME::_get_mtu,
    &ZCM_TRANS_CLASSNAME::_sendmsg,
    &ZCM_TRANS_CLASSNAME::_recvmsg_enable,
    &ZCM_TRANS_CLASSNAME::_recvmsg,
    NULL,
    &ZCM_TRANS_CLASSNAME::_destroy,
};

static zcm_trans_t *create(zcm_url_t *url)
{
    auto *trans = new ZCM_TRANS_CLASSNAME(url);
    if (trans->good())
        return trans;

    delete trans;
    return nullptr;
}

const TransportRegister ZCM_TRANS_CLASSNAME::reg(
    "file", "Interact with zcm log file (e.g. 'file://vehicle.log?speed=2.0)", create);
