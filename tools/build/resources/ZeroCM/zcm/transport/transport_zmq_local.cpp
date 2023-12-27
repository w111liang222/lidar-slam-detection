#ifdef USING_ZMQ

#include "zcm/transport.h"
#include "zcm/transport_registrar.h"
#include "zcm/transport_register.hpp"
#include "zcm/util/debug.h"
#include "zcm/util/lockfile.h"
#include <zmq.h>

#include "util/TimeUtil.hpp"

#include <unistd.h>
#include <dirent.h>

#include <cstdio>
#include <cstring>
#include <cassert>

#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <mutex>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <regex>

using namespace std;

// Define this the class name you want
#define ZCM_TRANS_CLASSNAME TransportZmqLocal
#define MTU (1<<28)
#define START_BUF_SIZE (1 << 20)
#define ZMQ_IO_THREADS 1
#define IPC_NAME_PREFIX "zcm-channel-zmq-ipc-"

enum Type { IPC, INPROC, };

static bool isRegexChannel(const string& channel)
{
    // These chars are considered regex
    auto isRegexChar = [](char c) {
        return c == '(' || c == ')' || c == '|' ||
        c == '.' || c == '*' || c == '+';
    };

    for (auto& c : channel)
        if (isRegexChar(c))
            return true;

    return false;
}

struct ZCM_TRANS_CLASSNAME : public zcm_trans_t
{
    void *ctx;
    Type type;

    string subnet;
    int pubhwm = 1000, subhwm = 1000;

    unordered_map<string, pair<void*,lockfile_t*>> pubsocks;
    // socket pair contains the socket + whether it was subscribed to explicitly or not
    unordered_map<string, pair<void*, bool>> subsocks;
    typedef unordered_map<string, pair<void*, bool>>::iterator SubsocksItr;
    unordered_map<string, std::regex> regexChannels;

    string recvmsgChannel;
    size_t recvmsgBufferSize = START_BUF_SIZE; // Start at 1MB but allow it to grow to MTU
    uint8_t* recvmsgBuffer;
    size_t startRecvSockIdx = 0;

    // Mutex used to protect 'subsocks' while allowing
    // recvmsgEnable() and recvmsg() to be called
    // concurrently
    mutex mut;

    ZCM_TRANS_CLASSNAME(Type type_, zcm_url_t *url)
    {
        trans_type = ZCM_BLOCKING;
        vtbl = &methods;

        unordered_map<string, string> options;
        auto* opts = zcm_url_opts(url);
        for (size_t i = 0; i < opts->numopts; ++i)
            options[opts->name[i]] = opts->value[i];
        auto findOption = [&options](const string& s){
            auto it = options.find(s);
            if (it == options.end()) return (string*)nullptr;
            return &it->second;
        };

        // These are intentionally left undocumented as nobody should be
        // using them. They're here for advanced debugging only
        auto* pubhwmStr = findOption("pubhwm");
        if (pubhwmStr) pubhwm = atoi(pubhwmStr->c_str());
        auto* subhwmStr = findOption("subhwm");
        if (subhwmStr) subhwm = atoi(subhwmStr->c_str());

        subnet = zcm_url_address(url);

        // Make directory with all permissions
        mkdir(string("/tmp/" + subnet).c_str(), S_IRWXO | S_IRWXG | S_IRWXU);

        ZCM_DEBUG("ZMQ Subnet Address: %s\n", subnet.c_str());

        recvmsgBuffer = new uint8_t[recvmsgBufferSize];

        ctx = zmq_init(ZMQ_IO_THREADS);
        assert(ctx != nullptr);
        type = type_;
    }

    ~ZCM_TRANS_CLASSNAME()
    {
        int rc;
        string address;

        // Clean up all publish sockets
        for (auto it = pubsocks.begin(); it != pubsocks.end(); ++it) {
            address = getAddress(it->first);

            rc = zmq_unbind(it->second.first, address.c_str());
            if (rc == -1) {
                ZCM_DEBUG("failed to unbind pubsock: %s", zmq_strerror(errno));
            }

            rc = zmq_close(it->second.first);
            if (rc == -1) {
                ZCM_DEBUG("failed to close pubsock: %s", zmq_strerror(errno));
            }

            lockfile_unlock(it->second.second);
        }

        // Clean up all subscribe sockets
        for (auto it = subsocks.begin(); it != subsocks.end(); ++it) {
            address = getAddress(it->first);

            rc = zmq_disconnect(it->second.first, address.c_str());
            if (rc == -1) {
                ZCM_DEBUG("failed to disconnect subsock: %s", zmq_strerror(errno));
            }

            rc = zmq_close(it->second.first);
            if (rc == -1) {
                ZCM_DEBUG("failed to disconnect subsock: %s", zmq_strerror(errno));
            }
        }

        // Clean up the zmq context
        rc = zmq_ctx_term(ctx);
        if (rc == -1) {
            ZCM_DEBUG("failed to terminate context: %s", zmq_strerror(errno));
        }

        delete[] recvmsgBuffer;
    }

    bool isRegexSubscribed(string channel)
    {
        for (auto regIt = regexChannels.begin(); regIt != regexChannels.end(); ++regIt) {
            if (regex_match(channel, regIt->second)) {
                return true;
            }
        }
        return false;
    }

    string getAddress(const string& channel)
    {
        switch (type) {
            case IPC:
                return "ipc:///tmp/" + subnet + "/" + IPC_NAME_PREFIX + channel;
            case INPROC:
                return "inproc://" + subnet + "/" + IPC_NAME_PREFIX + channel;
        }
        assert(0 && "unreachable");
    }

    // May return null if it cannot create a new pubsock
    void *pubsockFindOrCreate(const string& channel)
    {
        auto it = pubsocks.find(channel);
        if (it != pubsocks.end())
            return it->second.first;
        // Before we create a pubsock, we need to acquire the lock file for this
        lockfile_t *lf = lockfile_trylock(getAddress(channel).c_str());
        if (!lf) {
            fprintf(stderr, "Failed to acquire publish lock on %s! "
                            "Are you attempting multiple publishers?\n",
                            channel.c_str());
            return nullptr;
        }
        void *sock = zmq_socket(ctx, ZMQ_PUB);
        if (sock == nullptr) {
            ZCM_DEBUG("failed to create pubsock: %s", zmq_strerror(errno));
            lockfile_unlock(lf);
            return nullptr;
        }
        int rc;
        rc = zmq_setsockopt(sock, ZMQ_SNDHWM, &pubhwm, sizeof(pubhwm));
        if (rc == -1) {
            ZCM_DEBUG("failed to set pub high water mark: %s", zmq_strerror(errno));
            lockfile_unlock(lf);
            return nullptr;
        }
        string address = getAddress(channel);
        rc = zmq_bind(sock, address.c_str());
        if (rc == -1) {
            ZCM_DEBUG("failed to bind pubsock: %s", zmq_strerror(errno));
            lockfile_unlock(lf);
            return nullptr;
        }
        pair<void*, lockfile_t*> p {sock, lf};
        pubsocks.emplace(channel, p);
        return sock;
    }

    // May return null if it cannot create a new subsock
    void *subsockFindOrCreate(const string& channel, bool subExplicit)
    {
        auto it = subsocks.find(channel);
        if (it != subsocks.end()) {
            it->second.second |= subExplicit;
            return it->second.first;
        }
        void *sock = zmq_socket(ctx, ZMQ_SUB);
        if (sock == nullptr) {
            ZCM_DEBUG("failed to create subsock: %s", zmq_strerror(errno));
            return nullptr;
        }
        int rc;
        rc = zmq_setsockopt(sock, ZMQ_RCVHWM, &subhwm, sizeof(subhwm));
        if (rc == -1) {
            ZCM_DEBUG("failed to set sub high water mark: %s", zmq_strerror(errno));
            return nullptr;
        }
        string address = getAddress(channel);
        rc = zmq_connect(sock, address.c_str());
        if (rc == -1) {
            ZCM_DEBUG("failed to connect subsock: %s", zmq_strerror(errno));
            return nullptr;
        }
        rc = zmq_setsockopt(sock, ZMQ_SUBSCRIBE, "", 0);
        if (rc == -1) {
            ZCM_DEBUG("failed to setsockopt on subsock: %s", zmq_strerror(errno));
            return nullptr;
        }
        subsocks.emplace(channel, make_pair(sock, subExplicit));
        return sock;
    }

    // Updates the iterator in case of erasing
    int subsockDelete(SubsocksItr& it)
    {
        string address = getAddress(it->first);
        int rc = zmq_disconnect(it->second.first, address.c_str());
        if (rc == -1) {
            ZCM_DEBUG("failed to disconnect subsock: %s", zmq_strerror(errno));
            return ZCM_ECONNECT;
        }

        rc = zmq_close(it->second.first);
        if (rc == -1) {
            ZCM_DEBUG("failed to close subsock: %s", zmq_strerror(errno));
            return ZCM_ECONNECT;
        }

        it = subsocks.erase(it);
        return ZCM_EOK;
    }

    void ipcScanForNewChannels()
    {
        const char *prefix = IPC_NAME_PREFIX;
        size_t prefixLen = strlen(IPC_NAME_PREFIX);

        DIR *d;
        dirent *ent;

        if (!(d=opendir(string("/tmp/" + subnet).c_str())))
            return;

        while ((ent=readdir(d)) != nullptr) {
            if (strncmp(ent->d_name, prefix, prefixLen) == 0) {
                string channel(ent->d_name + prefixLen);

                if (!isRegexSubscribed(channel)) continue;

                void *sock = subsockFindOrCreate(channel, false);
                if (sock == nullptr) {
                    ZCM_DEBUG("failed to open subsock in scanForNewChannels(%s)",
                              channel.c_str());
                }
            }
        }

        closedir(d);
    }

    // Note: This only works for channels within this instance! Creating another
    //       ZCM instance using 'inproc' will cause this scan to miss some channels!
    //       Need to implement a better technique. Should use a globally shared datastruct.
    void inprocScanForNewChannels()
    {
        for (auto& elt : pubsocks) {
            auto& channel = elt.first;
            if (!isRegexSubscribed(channel)) continue;
            void *sock = subsockFindOrCreate(channel, false);
            if (sock == nullptr) {
                ZCM_DEBUG("failed to open subsock in scanForNewChannels(%s)", channel.c_str());
            }
        }
    }

    /********************** METHODS **********************/
    size_t getMtu()
    {
        return MTU;
    }

    int sendmsg(zcm_msg_t msg)
    {
        string channel = msg.channel;
        if (channel.size() > ZCM_CHANNEL_MAXLEN)
            return ZCM_EINVALID;
        if (msg.len > MTU)
            return ZCM_EINVALID;

        void *sock = pubsockFindOrCreate(channel);
        if (sock == nullptr)
            return ZCM_ECONNECT;
        int rc = zmq_send(sock, msg.buf, msg.len, 0);
        if (rc == (int)msg.len)
            return ZCM_EOK;
        assert(rc == -1);
        ZCM_DEBUG("zmq_send failed with: %s", zmq_strerror(errno));
        return ZCM_EUNKNOWN;
    }

    int recvmsgEnable(const char *channel, bool enable)
    {
        assert(channel && "channel cannot be null");
        bool regex = isRegexChannel(channel);
        // Mutex used to protect 'subsocks' while allowing
        // recvmsgEnable() and recvmsg() to be called
        // concurrently
        unique_lock<mutex> lk(mut);

        if (regex) {
            if (enable) {
                regexChannels.insert({channel, std::regex(channel)});
            } else {
                regexChannels.erase(channel);

                for (auto it = subsocks.begin(); it != subsocks.end(); ) {
                    // This channel is subscribed to explicitly
                    if (!it->second.second) {
                        ++it;
                        continue;
                    }

                    if (isRegexSubscribed(channel)) continue;

                    auto ret = subsockDelete(it); // updates it
                    if (ret != ZCM_EOK) return ret;
                }
            }
            return ZCM_EOK;
        } else {
            if (enable) {
                void *sock = subsockFindOrCreate(channel, true);
                if (sock == nullptr) return ZCM_ECONNECT;
            } else {
                auto it = subsocks.find(channel);
                if (it == subsocks.end()) return ZCM_EINVALID;
                it->second.second = false;

                if (!isRegexSubscribed(channel)) {
                    auto ret = subsockDelete(it);
                    if (ret != ZCM_EOK) return ret;
                }
            }
            return ZCM_EOK;
        }
    }

    int recvmsg(zcm_msg_t *msg, int timeout)
    {
        // Build up a list of poll items
        vector<zmq_pollitem_t> pitems;
        vector<string> pchannels;
        {
            // Mutex used to protect 'subsocks' while allowing
            // recvmsgEnable() and recvmsg() to be called
            // concurrently
            unique_lock<mutex> lk(mut);

            // XXX Only call this if enough time has ellapse since the last
            //     time you called it
            if (!regexChannels.empty()) {
                switch (type) {
                    case IPC: ipcScanForNewChannels();
                    case INPROC: inprocScanForNewChannels();
                }
            }

            pitems.resize(subsocks.size());
            int i = 0;
            for (auto& elt : subsocks) {
                auto& channel = elt.first;
                auto& sock = elt.second.first;
                auto *p = &pitems[i];
                memset(p, 0, sizeof(*p));
                p->socket = sock;
                p->events = ZMQ_POLLIN;
                pchannels.emplace_back(channel);
                ++i;
            }
        }

        timeout = (timeout >= 0) ? timeout : -1;
        int rc = zmq_poll(pitems.data(), pitems.size(), timeout);
        // TODO: implement better error handling, but can't assert because this triggers during
        //       clean up of the zmq subscriptions and context (may need to look towards having a
        //       "ZCM_ETERM" return code that we can use to cancel the recv message thread
        if (rc == -1) {
            ZCM_DEBUG("zmq_poll failed with: %s", zmq_strerror(errno));
            return ZCM_EAGAIN;
        }
        if (rc >= 0) {
            if (startRecvSockIdx >= pitems.size()) startRecvSockIdx = 0;

            auto recvFromSocket = [&](size_t i){
                auto& p = pitems[i];

                // NOTE: zmq_recv can return an integer > the len parameter passed in
                //       (in this case recvmsgBufferSize); however, all bytes past
                //       len are truncated and not placed in the buffer. This means
                //       that you will always lose the first message you get that is
                //       larger than recvmsgBufferSize
                int rc = zmq_recv(p.socket, recvmsgBuffer, recvmsgBufferSize, 0);
                msg->utime = TimeUtil::utime();
                if (rc == -1) {
                    fprintf(stderr, "zmq_recv failed with: %s", zmq_strerror(errno));
                    // TODO: implement error handling, don't just assert
                    assert(0 && "unexpected codepath");
                }
                assert(0 <= rc);
                assert(rc < MTU && "Received message that is bigger than a legally-published message could be");
                if (rc > (int)recvmsgBufferSize) {
                    ZCM_DEBUG("Reallocating recv buffer to handle larger messages. Size is now %d", rc);
                    recvmsgBufferSize = rc * 2;
                    delete[] recvmsgBuffer;
                    recvmsgBuffer = new uint8_t[recvmsgBufferSize];
                    return ZCM_EAGAIN;
                }
                recvmsgChannel = pchannels[i];
                msg->channel = recvmsgChannel.c_str();
                msg->len = rc;
                msg->buf = recvmsgBuffer;

                // Note: This is probably fine and there probably isn't an elegant
                //       way to improve this, but we could technically have more than
                //       one socket with a message ready, but because our API is set
                //       up to only handle one message at a time, we don't read all
                //       the messages that are ready, only the first. You could
                //       imagine a case where one channel was at a very high freq
                //       and getting interpreted first could shadow a low freq message
                //       on a resource constrained system, though perhaps that's just
                //       and indicator that you need to optimize your code or do less.

                return ZCM_EOK;
            };

            for (size_t i = startRecvSockIdx; i < pitems.size(); ++i) {
                auto& p = pitems[i];
                if (p.revents == 0) continue;

                ++startRecvSockIdx;
                return recvFromSocket(i);
            }
            for (size_t i = 0; i < startRecvSockIdx; ++i) {
                auto& p = pitems[i];
                if (p.revents == 0) continue;

                ++startRecvSockIdx;
                return recvFromSocket(i);
            }

        }

        return ZCM_EAGAIN;
    }

    /********************** STATICS **********************/
    static zcm_trans_methods_t methods;
    static ZCM_TRANS_CLASSNAME *cast(zcm_trans_t *zt)
    {
        assert(zt->vtbl == &methods);
        return (ZCM_TRANS_CLASSNAME*)zt;
    }

    static size_t _getMtu(zcm_trans_t *zt)
    { return cast(zt)->getMtu(); }

    static int _sendmsg(zcm_trans_t *zt, zcm_msg_t msg)
    { return cast(zt)->sendmsg(msg); }

    static int _recvmsgEnable(zcm_trans_t *zt, const char *channel, bool enable)
    { return cast(zt)->recvmsgEnable(channel, enable); }

    static int _recvmsg(zcm_trans_t *zt, zcm_msg_t *msg, int timeout)
    { return cast(zt)->recvmsg(msg, timeout); }

    static void _destroy(zcm_trans_t *zt)
    { delete cast(zt); }

    static const TransportRegister regIpc;
    static const TransportRegister regInproc;
};

zcm_trans_methods_t ZCM_TRANS_CLASSNAME::methods = {
    &ZCM_TRANS_CLASSNAME::_getMtu,
    &ZCM_TRANS_CLASSNAME::_sendmsg,
    &ZCM_TRANS_CLASSNAME::_recvmsgEnable,
    &ZCM_TRANS_CLASSNAME::_recvmsg,
    NULL, // update
    &ZCM_TRANS_CLASSNAME::_destroy,
};

static zcm_trans_t *createIpc(zcm_url_t *url)
{
    return new ZCM_TRANS_CLASSNAME(IPC, url);
}

static zcm_trans_t *createInproc(zcm_url_t *url)
{
    return new ZCM_TRANS_CLASSNAME(INPROC, url);
}

// Register this transport with ZCM
#ifdef USING_TRANS_IPC
const TransportRegister ZCM_TRANS_CLASSNAME::regIpc(
    "ipc",    "Transfer data via Inter-process Communication (e.g. 'ipc')", createIpc);
#endif

#ifdef USING_TRANS_INPROC
const TransportRegister ZCM_TRANS_CLASSNAME::regInproc(
    "inproc", "Transfer data via Internal process memory (e.g. 'inproc')",  createInproc);
#endif

#endif
