#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <getopt.h>
#include <atomic>
#include <signal.h>
#include <unistd.h>
#include <limits>
#include <unordered_map>

#include <zcm/zcm-cpp.hpp>

#include "zcm/json/json.h"

#include "util/TimeUtil.hpp"

using namespace std;

static atomic_int done {0};

static void sighandler(int signal)
{
    done++;
    if (done == 3) exit(1);
}

struct Args
{
    double speed = 1.0;
    bool verbose = false;
    string zcmUrlOut = "";
    string filename = "";
    string jslpFilename = "";
    zcm::Json::Value jslpRoot;
    string outfile = "";
    bool highAccuracyMode = false;

    bool init(int argc, char *argv[])
    {
        struct option long_opts[] = {
            { "help",                no_argument, 0, 'h' },
            { "output",        required_argument, 0, 'o' },
            { "speed",         required_argument, 0, 's' },
            { "zcm-url",       required_argument, 0, 'u' },
            { "jslp",          required_argument, 0, 'j' },
            { "high-accuracy",       no_argument, 0, 'a' },
            { "verbose",             no_argument, 0, 'v' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long(argc, argv, "ho:s:u:j:av", long_opts, 0)) >= 0) {
            switch (c) {
                case 'o':          outfile = string(optarg);       break;
                case 's':            speed = strtod(optarg, NULL); break;
                case 'u':        zcmUrlOut = string(optarg);       break;
                case 'j':     jslpFilename = string(optarg);       break;
                case 'a': highAccuracyMode = true;                 break;
                case 'v':          verbose = true;                 break;
                case 'h': default: usage(); return false;
            };
        }

        if (optind != argc - 1) {
            cerr << "Please specify a logfile" << endl;
            usage();
            return false;
        }

        filename = string(argv[optind]);

        ifstream jslpFile { jslpFilename != "" ? jslpFilename : filename + ".jslp" };
        if (jslpFile.good()) {
            zcm::Json::Reader reader;
            if (!reader.parse(jslpFile, jslpRoot, false)) {
                if (verbose) {
                    cerr << "Failed to parse jslp file " << endl;
                    cerr << reader.getFormattedErrorMessages() << endl;
                }
                return false;
            }
            if (outfile != "") speed = 0;
            cerr << "Found jslp file. Filtering output." << endl;
        } else if (jslpFilename == "") {
            cerr << "No jslp file specified" << endl;
            if (outfile != "") {
                cerr << "Output file specified, but no jslp filter metafile found." << endl;
                return false;
            }
        } else {
            cerr << "Unable to find specified jslp file: " << jslpFilename << endl;
            return false;
        }

        if (speed == 0) speed = std::numeric_limits<decltype(speed)>::infinity();

        return true;
    }

    void usage()
    {
        cerr << "usage: zcm-logplayer [options] [FILE]" << endl
             << "" << endl
             << "    Reads packets from an ZCM log file and publishes them to a " << endl
             << "    ZCM transport." << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -s, --speed=NUM        Playback speed multiplier.  Default is 1." << endl
             << "  -u, --zcm-url=URL      Play logged messages on the specified ZCM URL." << endl
             << "  -o, --output=filename  Instead of broadcasting over zcm, log directly " << endl
             << "                         to a file. Enabling this, ignores the" << endl
             << "                         \"--speed\" option" << endl
             << "  -j, --jslp=filename    Use this jslp meta file. " << endl
             << "                         If unspecified, zcm-logplayer looks for a file " << endl
             << "                         with the same filename as the input log and " << endl
             << "                         a .jslp suffix" << endl
             << "  -a, --high-accuracy    Enable extremely accurate publish timing." << endl
             << "                         Note that enabling this feature will probably consume" << endl
             << "                         a full CPU so logplayer can bypass the OS scheduler" << endl
             << "                         with busy waits in between messages." << endl
             << "  -v, --verbose          Print information about each packet." << endl
             << "  -h, --help             Shows some help text and exits." << endl
             << endl;
    }
};

struct LogPlayer
{
    Args args;
    zcm::LogFile *zcmIn  = nullptr;
    zcm::ZCM     *zcmOut = nullptr;
    zcm::LogFile *logOut = nullptr;

    enum class StartMode { CHANNEL, US_DELAY, NUM_MODES };
    StartMode startMode = StartMode::NUM_MODES;
    string startChan = "";
    uint64_t startDelayUs = 0;

    bool filtering;

    enum class FilterType { CHANNELS, NUM_TYPES };
    FilterType filterType;

    enum class FilterMode { WHITELIST, BLACKLIST, SPECIFIED, NUM_MODES };
    FilterMode filterMode;

    struct ChannelDetails
    {
        bool enabled;
        string replayChannel;
    };
    unordered_map<string, ChannelDetails> channelMap;

    LogPlayer() { }

    ~LogPlayer()
    {
        if (logOut) { logOut->close(); delete logOut; }
        if (zcmIn)  { delete zcmIn;                   }
        if (zcmOut) { delete zcmOut;                  }
    }

    bool init(int argc, char *argv[])
    {
        if (!args.init(argc, argv))
            return false;

        zcmIn = new zcm::LogFile(args.filename, "r");
        if (!zcmIn->good()) {
            cerr << "Error: Failed to open '" << args.filename << "'" << endl;
            return false;
        }

        if (args.outfile == "") {
            zcmOut = new zcm::ZCM(args.zcmUrlOut);
            if (!zcmOut->good()) {
                cerr << "Error: Failed to create output ZCM" << endl;
                return false;
            }

            cout << "Using playback speed " << args.speed << endl;
        } else {
            logOut = new zcm::LogFile(args.outfile, "w");
            if (!logOut->good()) {
                cerr << "Error: Failed to create output log" << endl;
                return false;
            }
        }

        if (args.jslpRoot.isMember("START")) {
            string startModeStr = args.jslpRoot["START"]["mode"].asString();
            if (!args.jslpRoot["START"].isMember("mode")) {
                cerr << "Start mode unspecified in jslp" << endl;
                return false;
            }
            if (startModeStr == "channel") {
                startMode = StartMode::CHANNEL;
                if (!args.jslpRoot["START"].isMember("channel")) {
                    cerr << "Start channel unspecified in jslp" << endl;
                    return false;
                }
                startChan = args.jslpRoot["START"]["channel"].asString();
                cout << "Starting after first message published on channel: "
                     << startChan << endl;
            } else if (startModeStr == "us_delay") {
                startMode = StartMode::US_DELAY;
                if (!args.jslpRoot["START"].isMember("us_delay")) {
                    cerr << "Start us_delay unspecified in jslp" << endl;
                    return false;
                }
                startDelayUs = args.jslpRoot["START"]["us_delay"].asUInt64();
                cout << "Starting after " << startDelayUs << " microseconds" << endl;
            } else {
                cerr << "Start mode unrecognized in jslp: " << startModeStr << endl;
                return false;
            }
        }

        filtering = args.jslpRoot.isMember("FILTER");
        if (filtering) {
            if (!args.jslpRoot["FILTER"].isMember("type")) {
                cerr << "Filter \"type\" in jslp file unspecified" << endl;
                return false;
            }
            if (!args.jslpRoot["FILTER"].isMember("mode")) {
                cerr << "Filter \"mode\" in jslp file unspecified" << endl;
                return false;
            }

            if (args.jslpRoot["FILTER"]["type"] == "channels") {
                filterType = FilterType::CHANNELS;
                cout << "Filtering based on channels" << endl;
            } else {
                cerr << "Filter \"mode\" unrecognized: "
                     << args.jslpRoot["FILTER"]["mode"] << endl;
                return false;
            }

            if (args.jslpRoot["FILTER"]["mode"] == "whitelist") {
                filterMode = FilterMode::WHITELIST;
                cout << "Using whitelisting filter" << endl;
            } else if (args.jslpRoot["FILTER"]["mode"] == "blacklist") {
                filterMode = FilterMode::BLACKLIST;
                cout << "Using blacklisting filter" << endl;
            } else if (args.jslpRoot["FILTER"]["mode"] == "specified") {
                filterMode = FilterMode::SPECIFIED;
                cout << "Using specified filter" << endl;
            } else {
                cerr << "Filter \"type\" unrecognized: "
                     << args.jslpRoot["FILTER"]["type"] << endl;
                return false;
            }

            auto newChannel = [&] (string channel) {
                if (filterMode == FilterMode::SPECIFIED)
                    channelMap[channel].enabled = args.jslpRoot["FILTER"]["channels"][channel].asBool();
                else
                    channelMap[channel].enabled = true;

                if (args.verbose)
                    cout << channel << " : "
                         << (channelMap[channel].enabled ? "true" : "false") << endl;
            };

            if (args.jslpRoot["FILTER"]["channels"].isArray()) {
                for (auto channel : args.jslpRoot["FILTER"]["channels"])
                    newChannel(channel.asString());
            } else {
                for (auto channel : args.jslpRoot["FILTER"]["channels"].getMemberNames())
                    newChannel(channel);
            }
        }

        if (args.jslpRoot.isMember("RENAME")) {
            for (auto channel : args.jslpRoot["RENAME"].getMemberNames()) {
                channelMap[channel].replayChannel =
                    args.jslpRoot["RENAME"][channel].asString();
            }
        }

        return true;
    }

    int run()
    {
        int err = 0;
        const zcm::LogEvent* le = zcmIn->readNextEvent();
        uint64_t nowUs = TimeUtil::utime();

        if (!le) return err;

        uint64_t firstMsgUtime = (uint64_t) le->timestamp;
        // timestamp when first message is dispatched; will be overwritten in the loop
        uint64_t firstDispatchUtime = nowUs;

        bool startedPub = false;
        if (startMode == StartMode::NUM_MODES) startedPub = true;

        do {

            nowUs = TimeUtil::utime();

            // Total time difference from now to publishing the first message
            // is zero in first run
            uint64_t localDiffUs = nowUs - firstDispatchUtime;
            // Total difference of timestamps of the current and first message
            uint64_t logDiffUs = (uint64_t) le->timestamp - firstMsgUtime;
            uint64_t logDiffSpeedUs = logDiffUs / args.speed;
            uint64_t diffUs = logDiffSpeedUs > localDiffUs ? logDiffSpeedUs - localDiffUs : 0;
            // Ensure nanosleep wakes up before the range of uncertainty of
            // the OS scheduler would impact our sleep time. Then we busy wait
            const uint64_t busyWaitUs = args.highAccuracyMode ? 10000 : 0;
            diffUs = diffUs > busyWaitUs ? diffUs - busyWaitUs : 0;
            // Introducing time differences to starting times rather than last loop
            // times eliminates linear increase of delay when message are published
            timespec delay;
            delay.tv_sec = (long int) diffUs / 1000000;
            delay.tv_nsec = (long int) (diffUs - (delay.tv_sec * 1000000)) * 1000;

            // Sleep until we're supposed to wake up and busy wait
            if (diffUs > 0 && startedPub) nanosleep(&delay, nullptr);
            // Busy wait the rest
            while (logDiffSpeedUs > TimeUtil::utime() - firstDispatchUtime);

            if (!startedPub) {
                if (startMode == StartMode::CHANNEL) {
                    if (le->channel == startChan)
                        startedPub = true;
                } else if (startMode == StartMode::US_DELAY) {
                    if ((uint64_t) le->timestamp > firstMsgUtime + startDelayUs)
                        startedPub = true;
                }
            }

            zcm::LogEvent newLe = *le;
            if (channelMap.count(le->channel) > 0 &&
                !channelMap[le->channel].replayChannel.empty()) {
                newLe.channel = channelMap[le->channel].replayChannel;
            }

            auto publish = [&](){
                if (args.verbose)
                    printf("%.3f Channel %-20s size %d\n", newLe.timestamp / 1e6,
                           newLe.channel.c_str(), newLe.datalen);

                if (args.outfile == "")
                    zcmOut->publish(newLe.channel, newLe.data, newLe.datalen);
                else
                    logOut->writeEvent(&newLe);
            };

            if (startedPub) {
                if (!filtering) {
                    publish();
                } else {
                    if (filterType == FilterType::CHANNELS) {
                        if (filterMode == FilterMode::WHITELIST) {
                            if (channelMap.count(le->channel) > 0) publish();
                        } else if (filterMode == FilterMode::BLACKLIST) {
                            if (channelMap.count(le->channel) == 0) publish();
                        } else if (filterMode == FilterMode::SPECIFIED) {
                            if (channelMap.count(le->channel) == 0) {
                                cerr << "jslp file does not specify filtering behavior "
                                     << "for channel: " << le->channel << endl;
                                done = true;
                                err = 1;
                                continue;
                            }
                            if (channelMap[le->channel].enabled) publish();
                        } else {
                            assert(false && "Fatal error.");
                        }
                    } else {
                        assert(false && "Fatal error.");
                    }
                }
            }

        } while ((le = zcmIn->readNextEvent()) && !done);

        return err;
    }
};

int main(int argc, char* argv[])
{
    LogPlayer lp;
    if (!lp.init(argc, argv)) return 1;

    // Register signal handlers
    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);

    int ret = lp.run();

    cout << "zcm-logplayer done" << endl;

    return ret;
}
