#include <iostream>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <unistd.h>
#include <cinttypes>
#include <regex>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>
#include <signal.h>
#include <string>

#include <errno.h>
#include <time.h>
#include <getopt.h>

#include "zcm/zcm-cpp.hpp"
#include "zcm/util/debug.h"
#include "zcm/zcm_coretypes.h"

#include "util/TranscoderPluginDb.hpp"

#include "util/FileUtil.hpp"
#include "util/TimeUtil.hpp"
#include "util/Types.hpp"

using namespace std;

#include "platform.hpp"

static atomic_int done {0};

struct Args
{
    struct Shard
    {
        string zcmurl = "";
        vector<string> channels;
        int queue_size = 0;

        Shard(const string& zcmurl, int queue_size) :
            zcmurl(zcmurl), queue_size(queue_size)
        {}
    };

    vector<Shard> shards;

    double auto_split_mb      = 0.0;
    bool   force_overwrite    = false;
    bool   auto_increment     = false;
    bool   use_strftime       = false;
    bool   quiet              = false;
    int    rotate             = -1;
    int    fflush_interval_ms = 100;
    i64    max_target_memory  = 0;
    string plugin_path        = "";
    bool   debug              = false;

    string input_fname;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "hu:c:z:b:fir:s:ql:m:p:d";
        struct option long_opts[] = {
            { "help",              no_argument,       0, 'h' },
            { "zcm-url",           required_argument, 0, 'u' },
            { "channel",           required_argument, 0, 'c' },
            { "queue-size",        required_argument, 0, 'z' },
            { "split-mb",          required_argument, 0, 'b' },
            { "force",             no_argument,       0, 'f' },
            { "increment",         no_argument,       0, 'i' },
            { "rotate",            required_argument, 0, 'r' },
            { "strftime",          required_argument, 0, 's' },
            { "quiet",             no_argument,       0, 'q' },
            { "flush-interval",    required_argument, 0, 'l' },
            { "max-target-memory", required_argument, 0, 'm' },
            { "plugin-path",       required_argument, 0, 'p' },
            { "debug",             no_argument,       0, 'd' },

            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
            switch (c) {
                case 'u':
                    shards.emplace_back(optarg, 0);
                    break;
                case 'c':
                    if (shards.empty()) shards.emplace_back("", 0);
                    shards.back().channels.push_back(optarg);
                    break;
                case 'z':
                    if (shards.empty()) shards.emplace_back("", 0);
                    shards.back().queue_size = atoi(optarg);
                    if (shards.back().queue_size == 0) {
                        cerr << "Please specify a valid queue size greater than 0" << endl;
                        return false;
                    }
                    break;
                case 'b':
                    auto_split_mb = strtod(optarg, NULL);
                    if (auto_split_mb <= 0) {
                        cerr << "Please specify an auto split size greater than 0 MB" << endl;
                        return false;
                    }
                    break;
                case 'f':
                    force_overwrite = 1;
                    break;
                case 'i':
                    auto_increment = true;
                    break;
                case 'r': {
                    char* eptr = NULL;
                    rotate = strtol(optarg, &eptr, 10);
                    if (*eptr) {
                        cerr << "Please specify a valid rotate maximum" << endl;
                        return false;
                    }
                } break;
                case 's':
                    use_strftime = true;
                    break;
                case 'q':
                    quiet = true;
                    break;
                case 'l':
                    fflush_interval_ms = atol(optarg);
                    if (fflush_interval_ms <= 0) {
                        cerr << "Please specify a flush interval greater than 0 ms" << endl;
                        return false;
                    }
                    break;
                case 'm':
                    max_target_memory = atoll(optarg);
                    break;
                case 'p':
                    plugin_path = string(optarg);
                    break;
                case 'd':
                    debug = true;
                    break;
                case 'h': default: usage(); return false;
            };
        }

        if (optind == argc) {
            input_fname = "zcmlog-%Y-%m-%d";
            auto_increment = true;
            use_strftime = true;
        } else if (optind == argc - 1) {
            input_fname = argv[optind];
        } else if (optind < argc-1) {
            return false;
        }

        if (shards.empty()) shards.emplace_back("", 0);
        for (auto& s : shards) {
            if (s.channels.empty()) {
                s.channels.push_back(".*");
            }
        }

        if (auto_split_mb > 0 && !(auto_increment || (rotate > 0))) {
            cerr << "ERROR.  --split-mb requires either --increment or --rotate" << endl;
            return false;
        }

        if (rotate > 0 && auto_increment) {
            cerr << "ERROR.  --increment and --rotate can't both be used" << endl;
            return false;
        }

        return true;
    }

    void usage()
    {
        cout << "usage: zcm-logger [options] [FILE]" << endl
             << endl
             << "    ZCM message logging utility. Subscribes to traffic on one or more zcm" << endl
             << "    transports, and records all messages received on to FILE. If FILE is not" << endl
             << "    specified, then a filename is automatically chosen." << endl
             << endl
             << "Options:" << endl
             << endl
             << "  -u, --zcm-url=URL          Log messages on the specified ZCM URL" << endl
             << "                             Can specify this argument multiple times" << endl
             << "                             If no -c is specified for this -u, subscribe to \".*\"" << endl
             << "  -c, --channel=CHAN         Channel string to pass to zcm_subscribe." << endl
             << "                             Can provide multiple times."<< endl
             << "                             Every -c is subscribed to on the prior specified -u url" << endl
             << "                             If no -u url has been specified, -c will apply to the " << endl
             << "                             ZCM_DEFAULT_URL." << endl
             << "                             Inverting channel selection is possible through regex" << endl
             << "                             For example: -c \"^(?!(EXAMPLE)$).*$\" will subscribe" << endl
             << "                             to everything except \"EXAMPLE\"" << endl
             << "  -z, --queue-size=MSGS      Size of zcm send and receive queues in number of messages." << endl
             << "                             Can provide multiple times." << endl
             << "                             Applies to prior -u url." << endl
             << "  -l, --flush-interval=MS    Flush the log file to disk every MS milliseconds." << endl
             << "                             (default: 100)" << endl
             << "  -f, --force                Overwrite existing files" << endl
             << "  -h, --help                 Shows this help text and exits" << endl
             << "  -i, --increment            Automatically append a suffix to FILE" << endl
             << "                             such that the resulting filename does not" << endl
             << "                             already exist.  This option precludes -f and" << endl
             << "                             --rotate" << endl
             << "  -m, --max-unwritten-mb=SZ  Maximum size of received but unwritten" << endl
             << "                             messages to store in memory before dropping" << endl
             << "                             messages.  (default: 100 MB)" << endl
             << "  -r, --rotate=NUM           When creating a new log file, rename existing files" << endl
             << "                             out of the way and always write to FILE.0.  If" << endl
             << "                             FILE.0 already exists, it is renamed to FILE.1.  If" << endl
             << "                             FILE.1 exists, it is renamed to FILE.2, etc.  If" << endl
             << "                             FILE.NUM exists, then it is deleted.  This option" << endl
             << "                             precludes -i." << endl
             << "  -b, --split-mb=N           Automatically start writing to a new log" << endl
             << "                             file once the log file exceeds N MB in size" << endl
             << "                             (can be fractional).  This option requires -i" << endl
             << "                             or --rotate." << endl
             << "  -q, --quiet                Suppress normal output and only report errors." << endl
             << "  -s, --strftime             Format FILE with strftime." << endl
             << "  -m, --max-target-memory    Attempt to limit the total buffer usage to this" << endl
             << "                             amount of memory. If specified, ensure that this" << endl
             << "                             number is at least as large as the maximum message" << endl
             << "                             size you expect to receive. This argument is" << endl
             << "                             specified in bytes. Suffixes are not yet supported." << endl
             << "                             This argument is independent from --queue-size and total" << endl
             << "                             program memory usage will be closer to the sum of the size" << endl
             << "                             of all queues + max-target-memory" << endl
             << "  -p, --plugin-path=path     Path to shared library containing transcoder plugins" << endl
             << endl
             << "Rotating / splitting log files" << endl
             << "==============================" << endl
             << "    For long-term logging, zcm-logger can rotate through a fixed number of" << endl
             << "    log files, moving to a new log file as existing files reach a maximum size." << endl
             << "    To do this, use --rotate and --split-mb.  For example:" << endl
             << endl
             << "        # Rotate through logfile.0, logfile.1, ... logfile.4" << endl
             << "        zcm-logger --rotate=5 --split-mb=2 logfile" << endl
             << endl
             << "    Moving to a new file happens either when the current log file size exceeds" << endl
             << "    the limit specified by --split-mb, or when zcm-logger receives a SIGHUP." << endl
             << endl << endl;
    }
};

static zcm::LogEvent* cloneLogEvent(const zcm::LogEvent* evt)
{
    zcm::LogEvent* ret = new zcm::LogEvent;
    ret->eventnum  = evt->eventnum;
    ret->timestamp = evt->timestamp;
    ret->channel   = evt->channel;
    ret->datalen   = evt->datalen;
    ret->data      = new uint8_t[evt->datalen];
    memcpy(ret->data, evt->data, evt->datalen * sizeof(uint8_t));
    return ret;
}

struct Logger
{
    Args args;

    string filename;
    string fname_prefix;

    zcm::LogFile* log = nullptr;

    int next_increment_num          = 0;

    // variables for inverted matching (e.g., logging all but some channels)
    vector<regex> invert_regex;

    // these members controlled by writing
    size_t nevents                  = 0;
    size_t logsize                  = 0;
    size_t events_since_last_report = 0;
    u64    last_report_time         = 0;
    size_t last_report_logsize      = 0;
    u64    time0                    = TimeUtil::utime();
    u64    last_fflush_time         = 0;

    size_t dropped_packets_count    = 0;
    u64    last_drop_report_utime   = 0;
    size_t last_drop_report_count   = 0;

    int    num_splits               = 0;

    i64    totalMemoryUsage         = 0;

    mutex lk;
    condition_variable newEventCond;

    queue<zcm::LogEvent*> q;

    TranscoderPluginDb* pluginDb = nullptr;
    vector<zcm::TranscoderPlugin*> plugins;

    Logger() {}

    ~Logger()
    {
        if (pluginDb) { delete pluginDb; pluginDb = nullptr; }
        if (log)      { log->close(); delete log; }

        while (!q.empty()) {
            delete[] q.front()->data;
            delete q.front();
            q.pop();
        }
    }

    bool init(int argc, char *argv[])
    {
        if (!args.parse(argc, argv))
            return false;

        if (!openLogfile())
            return false;

        // Load plugins from path if specified
        assert(pluginDb == nullptr);
        if (args.plugin_path != "") {
            pluginDb = new TranscoderPluginDb(args.plugin_path, args.debug);
            vector<const zcm::TranscoderPlugin*> dbPlugins = pluginDb->getPlugins();
            if (dbPlugins.empty()) {
                cerr << "Couldn't find any plugins. Aborting." << endl;
                return false;
            }
            vector<string> dbPluginNames = pluginDb->getPluginNames();
            for (size_t i = 0; i < dbPlugins.size(); ++i) {
                plugins.push_back((zcm::TranscoderPlugin*) dbPlugins[i]);
                if (args.debug) cout << "Loaded plugin: " << dbPluginNames[i] << endl;
            }
        }

        if (args.debug) return true;

        return true;
    }

    void rotate_logfiles()
    {
        if (!args.quiet) cout << "Rotating log files" << endl;

        // delete log files that have fallen off the end of the rotation
        string tomove = fname_prefix + "." + to_string(args.rotate-1);
        if (FileUtil::exists(tomove))
            if (0 != FileUtil::remove(tomove))
                cerr << "ERROR! Unable to delete [" << tomove << "]" << endl;

        // Rotate away any existing log files
        for (int file_num = args.rotate-1; file_num >= 0; file_num--) {
            string newname = fname_prefix + "." + to_string(file_num);
            string tomove  = fname_prefix + "." + to_string(file_num-1);
            if (FileUtil::exists(tomove))
                if (0 != FileUtil::rename(tomove, newname))
                    cerr << "ERROR!  Unable to rotate [" << tomove << "]" << endl;
        }
    }

    bool openLogfile()
    {
        char tmp_path[PATH_MAX];

        // maybe run the filename through strftime
        if (args.use_strftime) {
            time_t now = time (NULL);
            strftime(tmp_path, sizeof(tmp_path),
                     args.input_fname.c_str(), localtime(&now));
            string new_prefix = tmp_path;

            // If auto-increment is enabled and the strftime-formatted filename
            // prefix has changed, then reset the auto-increment counter.
            if (args.auto_increment && fname_prefix != new_prefix)
                next_increment_num = 0;
            fname_prefix = std::move(new_prefix);
        } else {
            fname_prefix = args.input_fname;
        }

        if (args.auto_increment) {
            /* Loop through possible file names until we find one that doesn't
             * already exist.  This way, we never overwrite an existing file. */
            do {
                snprintf(tmp_path, sizeof(tmp_path), "%s.%04d",
                         fname_prefix.c_str(), next_increment_num);
                filename = tmp_path;
                next_increment_num++;
            } while (FileUtil::exists(filename));
        } else if (args.rotate > 0) {
            filename = fname_prefix + ".0";
        } else {
            filename = fname_prefix;
            if (!args.force_overwrite) {
                if (FileUtil::exists(filename)) {
                    cerr << "Refusing to overwrite existing file \""
                         << filename << "\"" << endl;
                    return false;
                }
            }
        }

        // create directories if needed
        string dirpart = FileUtil::dirname(filename);
        if (!FileUtil::dirExists(dirpart))
            FileUtil::mkdirWithParents(dirpart, 0755);

        if (!args.quiet) cout << "Opening log file \"" << filename << "\"" << endl;

        // open output file in append mode if we're rotating log files, or write
        // mode if not.
        log = new zcm::LogFile(filename, (args.rotate > 0) ? "a" : "w");
        if (!log->good()) {
            perror("Error: fopen failed");
            delete log;
            return false;
        }
        return true;
    }

    void handler(const zcm::ReceiveBuffer* rbuf, const string& channel)
    {
        vector<zcm::LogEvent*> evts;

        zcm::LogEvent* le = new zcm::LogEvent;
        le->timestamp = rbuf->recv_utime;
        le->channel   = channel;
        le->datalen   = rbuf->data_size;

        if (!plugins.empty()) {
            le->data = rbuf->data;

            int64_t msg_hash;
            __int64_t_decode_array(le->data, 0, 8, &msg_hash, 1);

            for (auto& p : plugins) {
                vector<const zcm::LogEvent*> pevts =
                    p->transcodeEvent((uint64_t) msg_hash, le);
                for (auto* evt : pevts)
                    if (evt) evts.emplace_back(cloneLogEvent(evt));
                    else     evts.emplace_back(nullptr);
            }
        }

        if (evts.empty()) {
            le->data = new uint8_t[rbuf->data_size];
            memcpy(le->data, rbuf->data, sizeof(uint8_t) * rbuf->data_size);
            evts.push_back(le);
        } else {
            delete le;
        }

        bool stillRoom = true;
        {
            unique_lock<mutex> lock{lk};
            while (!evts.empty()) {
                if (stillRoom) {
                    zcm::LogEvent* le = evts.back();
                    if (!le) {
                        evts.pop_back();
                        continue;
                    }
                    q.push(le);
                    totalMemoryUsage += le->datalen + le->channel.size() + sizeof(*le);
                    stillRoom = (args.max_target_memory == 0) ? true :
                        (totalMemoryUsage + rbuf->data_size < args.max_target_memory);
                    evts.pop_back();
                } else {
                    ZCM_DEBUG("Dropping message due to enforced memory constraints");
                    ZCM_DEBUG("Current memory estimations are at %" PRId64 " bytes",
                              totalMemoryUsage);
                    while (!evts.empty()) {
                        delete[] evts.back()->data;
                        delete evts.back();
                        evts.pop_back();
                    }
                    break;
                }
            }
        }
        newEventCond.notify_all();
    }

    void flushWhenReady()
    {
        zcm::LogEvent *le = nullptr;
        size_t qSize = 0;
        i64 memUsed = 0;
        {
            unique_lock<mutex> lock{lk};

            while (q.empty()) {
                if (done) return;
                newEventCond.wait(lock);
            }
            if (done) return;

            le = q.front();
            q.pop();
            qSize = q.size();
            memUsed = totalMemoryUsage; // want to capture the max mem used, not post flush
            totalMemoryUsage -= (le->datalen + le->channel.size() + sizeof(*le));
        }
        if (qSize != 0) ZCM_DEBUG("Queue size = %zu\n", qSize);

        // Is it time to start a new logfile?
        if (args.auto_split_mb) {
            double logsize_mb = (double)logsize / (1 << 20);
            if (logsize_mb > args.auto_split_mb) {
                // Yes.  open up a new log file
                log->close();
                if (args.rotate > 0)
                    rotate_logfiles();
                if (!openLogfile()) exit(1);
                num_splits++;
                logsize = 0;
                last_report_logsize = 0;
            }
        }

        if (log->writeEvent(le) != 0) {
            static u64 last_spew_utime = 0;
            string reason = strerror(errno);
            u64 now = TimeUtil::utime();
            if (now - last_spew_utime > 500000) {
                cerr << "zcm_eventlog_write_event: " << reason << endl;
                last_spew_utime = now;
            }
            if (errno == ENOSPC)
                exit(1);

            delete[] le->data;
            delete le;
            return;
        }

        if (args.fflush_interval_ms >= 0 &&
            (le->timestamp - last_fflush_time) > (u64)args.fflush_interval_ms * 1000) {
            Platform::fflush(log->getFilePtr());
            last_fflush_time = le->timestamp;
        }

        // bookkeeping, cleanup
        nevents++;
        events_since_last_report++;
        logsize += 4 + 8 + 8 + 4 + le->channel.size() + 4 + le->datalen;

        i64 offset_utime = le->timestamp - time0;
        if (!args.quiet && (offset_utime - last_report_time > 1000000)) {
            double dt = (offset_utime - last_report_time)/1000000.0;

            double tps =  events_since_last_report / dt;
            double kbps = (logsize - last_report_logsize) / dt / 1024.0;
            printf("Summary: %s ti:%4" PRId64 " sec  |  Events: %-9zu ( %4zu MB )  |  "
                   "TPS: %8.2f  |  KB/s: %8.2f  |  Buf Size: % 8" PRId64 " KB\n",
                   filename.c_str(),
                   offset_utime / 1000000,
                   nevents, logsize/1048576,
                   tps, kbps, memUsed / 1024);
            last_report_time = offset_utime;
            events_since_last_report = 0;
            last_report_logsize = logsize;
        }

        delete[] le->data;
        delete le;
    }

    void wakeup()
    {
        unique_lock<mutex> lock(lk);
        newEventCond.notify_all();
    }
};

Logger logger{};

void sighandler(int signal)
{
    done++;
    logger.wakeup();
    if (done == 3) exit(1);
}

int main(int argc, char *argv[])
{
    Platform::setstreambuf();

    if (!logger.init(argc, argv)) return 1;

    vector<unique_ptr<zcm::ZCM>> zcms;
    for (const auto& s : logger.args.shards) {
        ZCM_DEBUG("Constructing shard with url: %s", s.zcmurl.c_str());
        zcms.emplace_back(new zcm::ZCM(s.zcmurl));
        if (!zcms.back()->good()) {
            cerr << "Couldn't initialize ZCM: " << s.zcmurl << endl
                 << "Please provide a valid zcm url either with the ZCM_DEFAULT_URL" << endl
                 << "environment variable, or with the '-u' command line argument." << endl;
            return 1;
        }

        if (s.queue_size > 0) {
            ZCM_DEBUG("Setting shard queue size to: %d", s.queue_size);
            zcms.back()->setQueueSize(s.queue_size);
        }

        for (const auto& c : s.channels) {
            ZCM_DEBUG("Subscribing to : %s", c.c_str());
            zcms.back()->subscribe(c, &Logger::handler, &logger);
        }
    }

    // Register signal handlers
    signal(SIGINT,  sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);

    ZCM_DEBUG("Starting zcms");
    for (auto& z : zcms) z->start();

    while (!done) logger.flushWhenReady();

    ZCM_DEBUG("Stopping zcms");
    for (auto& z : zcms) {
        z->stop();
        z->flush();
    }

    cerr << "Logger exiting" << endl;

    return 0;
}
