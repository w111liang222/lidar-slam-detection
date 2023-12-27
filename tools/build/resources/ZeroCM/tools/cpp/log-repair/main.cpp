#include <algorithm>
#include <atomic>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include <getopt.h>
#include <signal.h>
#include <unistd.h>

#include <zcm/zcm-cpp.hpp>

using namespace std;

static atomic_int done {0};

static void sighandler(int signal)
{
    done++;
    if (done == 3) exit(1);
}

struct Args
{
    string infile  = "";
    string outfile = "";
    bool   verify  = false;

    bool init(int argc, char *argv[])
    {
        struct option long_opts[] = {
            { "help",         no_argument, 0, 'h' },
            { "output", required_argument, 0, 'o' },
            { "verify",       no_argument, 0, 'v' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long(argc, argv, "ho:v", long_opts, 0)) >= 0) {
            switch (c) {
                case 'o': outfile    = string(optarg); break;
                case 'v': verify = true;           break;
                case 'h': default: usage(); return false;
            };
        }

        if (optind != argc - 1) {
            cerr << "Please specify a logfile" << endl;
            usage();
            return false;
        }

        infile = string(argv[optind]);

        if (outfile.empty() && !verify) {
            cerr << "Must specify output file or verify mode" << endl;
            return false;
        }

        return true;
    }

    void usage()
    {
        cerr << "usage: zcm-log-repair [options] [FILE]" << endl
             << "" << endl
             << "    Reads packets from a ZCM log file and writes them to an output " << endl
             << "    log file ensuring that all events are stored in recv_utime order." << endl
             << "    This is generally only required if the original log was captured" << endl
             << "    using multiple zcm shards and the user requires a strict ordering" << endl
             << "    of events in the log." << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -h, --help             Shows this help text and exits" << endl
             << "  -o, --output=filename  specify output file" << endl
             << "  -v, --verify           verify input log is monotonic in timestamp" << endl
             << endl;
    }
};

struct LogRepair
{
    Args args;
    zcm::LogFile* logIn  = nullptr;
    zcm::LogFile* logOut = nullptr;

    const zcm::LogEvent*         event;
    off_t                        length;
    off_t                        offset;
    vector<pair<int64_t, off_t>> timestamps;
    size_t                       progress;

    LogRepair() { }

    ~LogRepair()
    {
        if (logIn) delete logIn;
        if (logOut) delete logOut;
    }

    bool init(int argc, char *argv[])
    {
        if (!args.init(argc, argv))
            return false;

        logIn = new zcm::LogFile(args.infile, "r");
        if (!logIn->good()) {
            cerr << "Error: Failed to open '" << args.infile << "'" << endl;
            return false;
        }

        if (!args.verify) {
            logOut = new zcm::LogFile(args.outfile, "w");
            if (!logOut->good()) {
                cerr << "Error: Failed to create output log" << endl;
                return false;
            }
        }

        fseeko(logIn->getFilePtr(), 0, SEEK_END);
        length = ftello(logIn->getFilePtr());
        fseeko(logIn->getFilePtr(), 0, SEEK_SET);

        // somewhat arbitrary, but starting with a high capacity helps speed up the read-in
        timestamps.reserve(1e6);

        return true;
    }

    int run()
    {
        // XXX: Note we are reading ALL of the event timestamps into memory, so
        //      this will use something like 16 * num_events memory. In some use
        //      cases that won't be ideal, so if people are running into that, we
        //      can try a different approach.
        cout << "Reading log" << endl;
        progress = 0;
        cout << progress << "%" << flush;
        while (true) {
            if (done) return 1;

            offset = ftello(logIn->getFilePtr());
            event  = logIn->readNextEvent();
            if (!event) break;

            if (args.verify && !timestamps.empty() &&
                event->timestamp < timestamps.back().first) {
                cerr << endl << "Detected nonmonotonic timestamp at event "
                     << timestamps.size() << endl;
                return 1;
            }

            timestamps.emplace_back(event->timestamp, offset);

            if (timestamps.size() == timestamps.capacity()) {
                timestamps.reserve(timestamps.capacity() * 2);
            }

            size_t p = (size_t)((100 * offset) / length);
            if (p != progress) {
                progress = p;
                cout << "\r" << progress << "%" << flush;
            }
        }
        cout << endl << "Read " << timestamps.size() << " events" << endl;

        if (args.verify) return 0;

        sort(timestamps.begin(), timestamps.end());

        cout << "Writing new log" << endl;
        progress = 0;
        cout << progress << "%" << flush;
        for (size_t i = 0; i < timestamps.size(); ++i) {
            if (done) return 1;

            logOut->writeEvent(logIn->readEventAtOffset(timestamps[i].second));

            size_t p = (100 * i) / timestamps.size();
            if (p != progress) {
                progress = p;
                cout << "\r" << progress << "%" << flush;
            }
        }

        cout << endl << "Flushing to disk" << endl;
        delete logOut;

        cout << "Verifying output" << endl;
        logOut = new zcm::LogFile(args.verify ? args.infile : args.outfile, "r");
        if (!logOut->good()) {
            cerr << "Error: Failed to open log for verification" << endl;
            return 1;
        }

        size_t i = 0;
        progress = 0;
        cout << progress << "%" << flush;
        while (true) {
            if (done) return 1;

            event = logOut->readNextEvent();
            if (!event) break;
            if (event->timestamp != timestamps[i++].first) {
                cerr << endl << "Error: output log timestamp mismatch" << endl;
                cerr << "Expected " << timestamps[i].first << " got "
                     << event->timestamp << " (idx " << i << ")" << endl;
                return 1;
            }

            size_t p = (100 * i) / timestamps.size();
            if (p != progress) {
                progress = p;
                cout << "\r" << progress << "%" << flush;
            }
        }
        if (i < timestamps.size()) {
            cerr << endl << "Error: output log was missing "
                 << timestamps.size() - i << " events" << endl;
        }

        return 0;
    }
};

int main(int argc, char* argv[])
{
    LogRepair lr;
    if (!lr.init(argc, argv)) return 1;

    // Register signal handlers
    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);

    int ret = lr.run();

    if (ret == 0) {
        cout << endl << "Success" << endl;
    } else {
        cerr << endl << "Failure" << endl;
    }

    return ret;
}
