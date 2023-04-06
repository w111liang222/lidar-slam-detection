#include <iostream>
#include <fstream>
#include <getopt.h>
#include <algorithm>
#include <memory>

#include <zcm/zcm-cpp.hpp>
#include <zcm/zcm_coretypes.h>

#include "zcm/json/json.h"

#include "util/TranscoderPluginDb.hpp"

using namespace std;

struct Args
{
    string inlog       = "";
    string outlog      = "";
    string plugin_path = "";
    bool debug         = false;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "l:o:p:dh";
        struct option long_opts[] = {
            { "log",         required_argument, 0, 'l' },
            { "output",      required_argument, 0, 'o' },
            { "plugin-path", required_argument, 0, 'p' },
            { "debug",       no_argument,       0, 'd' },
            { "help",        no_argument,       0, 'h' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
            switch (c) {
                case 'l': inlog       = string(optarg); break;
                case 'o': outlog      = string(optarg); break;
                case 'p': plugin_path = string(optarg); break;
                case 'd': debug       = true;           break;
                case 'h': default: usage(); return false;
            };
        }

        if (inlog == "") {
            cerr << "Please specify logfile input" << endl;
            return false;
        }

        if (outlog  == "") {
            cerr << "Please specify log file output" << endl;
            return false;
        }

        const char* plugin_path_env = getenv("ZCM_LOG_TRANSCODER_PLUGINS_PATH");
        if (plugin_path == "" && plugin_path_env) plugin_path = plugin_path_env;
        if (plugin_path == "") {
            cerr << "No plugin specified. Nothing to do" << endl;
            return false;
        }

        return true;
    }

    void usage()
    {
        cout << "usage: zcm-log-transcoder [options]" << endl
             << "" << endl
             << "    Convert messages in one log file from one zcm type to another" << endl
             << "" << endl
             << "Example:" << endl
             << "    zcm-log-transcoder -l zcm.log -o zcmout.log -p path/to/plugin.so" << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -h, --help              Shows this help text and exits" << endl
             << "  -l, --log=logfile       Input log to convert" << endl
             << "  -o, --output=logfile    Output converted log file" << endl
             << "  -p, --plugin-path=path  Path to shared library containing transcoder plugins" << endl
             << "                          Can also be specified via the environment variable" << endl
             << "                          ZCM_LOG_TRANSCODER_PLUGINS_PATH" << endl
             << "  -d, --debug             Run a dry run to ensure proper transcoder setup" << endl
             << endl << endl;
    }
};

int main(int argc, char* argv[])
{
    Args args;
    if (!args.parse(argc, argv)) return 1;

    zcm::LogFile inlog(args.inlog, "r");
    if (!inlog.good()) {
        cerr << "Unable to open input zcm log: " << args.inlog << endl;
        return 1;
    }
    fseeko(inlog.getFilePtr(), 0, SEEK_END);
    off64_t logSize = ftello(inlog.getFilePtr());
    fseeko(inlog.getFilePtr(), 0, SEEK_SET);

    zcm::LogFile outlog(args.outlog, "w");
    if (!outlog.good()) {
        cerr << "Unable to open output zcm log: " << args.outlog << endl;
        return 1;
    }


    vector<zcm::TranscoderPlugin*> plugins;
    TranscoderPluginDb pluginDb(args.plugin_path, args.debug);
    vector<const zcm::TranscoderPlugin*> dbPlugins = pluginDb.getPlugins();
    if (dbPlugins.empty()) {
        cerr << "Couldn't find any plugins. Aborting." << endl;
        return 1;
    }
    vector<string> dbPluginNames = pluginDb.getPluginNames();
    for (size_t i = 0; i < dbPlugins.size(); ++i) {
        plugins.push_back((zcm::TranscoderPlugin*) dbPlugins[i]);
        if (args.debug) cout << "Loaded plugin: " << dbPluginNames[i] << endl;
    }

    if (args.debug) return 0;

    size_t numInEvents = 0, numOutEvents = 0;
    const zcm::LogEvent* evt;
    off64_t offset;

    static int lastPrintPercent = 0;
    while (1) {
        offset = ftello(inlog.getFilePtr());

        int percent = 100.0 * offset / (logSize == 0 ? 1 : logSize);
        if (percent != lastPrintPercent) {
            cout << "\r" << "Percent Complete: " << percent << flush;
            lastPrintPercent = percent;
        }

        evt = inlog.readNextEvent();
        if (evt == nullptr) break;

        vector<const zcm::LogEvent*> evts;

        int64_t msg_hash;
        __int64_t_decode_array(evt->data, 0, 8, &msg_hash, 1);

        for (auto& p : plugins) {
            vector<const zcm::LogEvent*> pevts =
                p->transcodeEvent((uint64_t) msg_hash, evt);
            evts.insert(evts.end(), pevts.begin(), pevts.end());
        }

        if (evts.empty()) evts.push_back(evt);

        for (auto* evt : evts) {
            if (!evt) continue;
            outlog.writeEvent(evt);
            numOutEvents++;
        }

        numInEvents++;
    }
    if (lastPrintPercent != 100)
        cout << "\r" << "Percent Complete: 100" << flush;
    cout << endl;

    inlog.close();
    outlog.close();

    cout << "Transcoded " << numInEvents << " events into " << numOutEvents << " events" << endl;
    return 0;
}
