#include <iostream>
#include <fstream>
#include <getopt.h>
#include <algorithm>
#include <memory>

#include <zcm/zcm-cpp.hpp>

#include "zcm/json/json.h"

#include "util/TypeDb.hpp"

#include "IndexerPluginDb.hpp"

#include "TimingIndexerPlugin.hpp"
#include "BandwidthIndexerPlugin.hpp"

using namespace std;

struct Args
{
    string logfile     = "";
    string output      = "";
    string plugin_path = "";
    string type_path   = "";
    bool readable      = false;
    bool debug         = false;
    bool useTiming     = false;
    bool useBandwidth  = false;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "l:o:p:t:rh";
        struct option long_opts[] = {
            { "log",           required_argument, 0, 'l' },
            { "output",        required_argument, 0, 'o' },
            { "plugin-path",   required_argument, 0, 'p' },
            { "type-path",     required_argument, 0, 't' },
            { "readable",      no_argument,       0, 'r' },
            { "use-timing",    no_argument,       0,  0  },
            { "use-bandwidth", no_argument,       0,  0  },
            { "debug",         no_argument,       0,  0  },
            { "help",          no_argument,       0, 'h' },
            { 0, 0, 0, 0 }
        };

        int c;
        int option_index;
        while ((c = getopt_long(argc, argv, optstring, long_opts, &option_index)) >= 0) {
            switch (c) {
                case 'l': logfile     = string(optarg); break;
                case 'o': output      = string(optarg); break;
                case 'p': plugin_path = string(optarg); break;
                case 't': type_path   = string(optarg); break;
                case 'r': readable    = true;           break;
                case  0: {
                    string longopt = string(long_opts[option_index].name);
                    if (longopt == "debug") debug = true;
                    else if (longopt == "use-timing") useTiming = true;
                    else if (longopt == "use-bandwidth") useBandwidth = true;
                    break;
                }
                case 'h': default: usage(); return false;
            };
        }

        if (logfile == "") {
            cerr << "Please specify logfile input" << endl;
            return false;
        }

        if (output  == "") {
            cerr << "Please specify index file output" << endl;
            return false;
        }

        const char* type_path_env = getenv("ZCM_LOG_INDEXER_ZCMTYPES_PATH");
        if (type_path == "" && type_path_env) type_path = type_path_env;
        if (type_path == "") {
            cerr << "Please specify a zcmtypes.so path either through -t TYPE_PATH "
                    "or through the env var ZCM_LOG_INDEXER_ZCMTYPES_PATH" << endl;
            return false;
        }

        const char* plugin_path_env = getenv("ZCM_LOG_INDEXER_PLUGINS_PATH");
        if (plugin_path == "" && plugin_path_env) plugin_path = plugin_path_env;

        return true;
    }

    void usage()
    {
        cout << "usage: zcm-log-indexer [options]" << endl
             << "" << endl
             << "    Load in a log file and write an index json file that" << endl
             << "    allows for faster log indexing." << endl
             << "" << endl
             << "Example:" << endl
             << "    zcm-log-indexer -l zcm.log -o index.dbz -t path/to/zcmtypes.so" << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -h, --help              Shows this help text and exits" << endl
             << "  -l, --log=logfile       Input log to index for fast querying" << endl
             << "  -o, --output=indexfile  Output index file to be used with log" << endl
             << "  -p, --plugin-path=path  Path to shared library containing indexer plugins" << endl
             << "                          Can also be specified via the environment variable" << endl
             << "                          ZCM_LOG_INDEXER_PLUGINS_PATH" << endl
             << "  -t, --type-path=path    Path to shared library containing the zcmtypes" << endl
             << "                          Can also be specified via the environment variable" << endl
             << "                          ZCM_LOG_INDEXER_ZCMTYPES_PATH" << endl
             << "  -r, --readable          Don't minify the output index file. " << endl
             << "                          Leave it human readable" << endl
             << "      --use-timing        Run with the default timestamp indexer" << endl
             << "      --use-bandwidth     Run with the default bandwidth indexer" << endl
             << "      --debug             Run a dry run to ensure proper indexer setup" << endl
             << endl << endl;
    }
};

int main(int argc, char* argv[])
{
    Args args;
    if (!args.parse(argc, argv)) return 1;

    zcm::LogFile log(args.logfile, "r");
    if (!log.good()) {
        cerr << "Unable to open logfile: " << args.logfile << endl;
        return 1;
    }
    fseeko(log.getFilePtr(), 0, SEEK_END);
    // TODO: Look into handling large logfiles
    off_t logSize = ftello(log.getFilePtr());

    ofstream output;
    output.open(args.output);
    if (!output.is_open()) {
        cerr << "Unable to open output file: " << args.output << endl;
        log.close();
        return 1;
    }

    vector<zcm::IndexerPlugin*> plugins;

    struct DefaultPlugin {
        zcm::IndexerPlugin* plugin;
        bool shouldBeIncluded;
        DefaultPlugin(zcm::IndexerPlugin* p, bool shouldBeIncluded) :
            plugin(p), shouldBeIncluded(shouldBeIncluded)
        {}
    };
    vector<DefaultPlugin> defaults;
    defaults.emplace_back(new TimingIndexerPlugin(), args.useTiming);
    defaults.emplace_back(new BandwidthIndexerPlugin(), args.useBandwidth);

    IndexerPluginDb pluginDb(args.plugin_path, args.debug);
    // Load plugins from path if specified
    if (args.plugin_path != "") {
        vector<const zcm::IndexerPlugin*> dbPlugins = pluginDb.getPlugins();
        // casting away constness. Don't mess up.
        for (auto dbp : dbPlugins) {
            plugins.push_back((zcm::IndexerPlugin*) dbp);
            auto deps = dbp->dependsOn();
            for (auto dep : deps) {
                for (auto& p : defaults) {
                    if (dep == p.plugin->name()) {
                        p.shouldBeIncluded = true;
                        break;
                    }
                }
            }
        }
    }

    for (auto p : defaults) {
        if (p.shouldBeIncluded) {
            plugins.push_back(p.plugin);
        }
    }

    if (plugins.size() == 0) {
        cerr << "No plugins specified" << endl;
        return 1;
    }

    for (size_t i = 0; i < plugins.size(); ++i) {
        for (size_t j = i + 1; j < plugins.size(); ++j) {
            if (args.debug)
                cout << plugins.data()[i]->name() << "->name() == "
                     << plugins.data()[j]->name() << "->name()" << endl;

            if (plugins[i]->name() == plugins[j]->name()) {
                cerr << "Plugins must have unique names. Collision: "
                     << plugins[i]->name() << endl;
                return 1;
            }
        }
    }

    TypeDb types(args.type_path, args.debug);

    if (args.debug) return 0;

    struct PluginRuntimeInfo {
        zcm::IndexerPlugin* plugin;
        bool runThroughLog;
    };

    auto buildPluginGroups = [] (vector<zcm::IndexerPlugin*> plugins) {
        vector<vector<PluginRuntimeInfo>> groups;
        vector<zcm::IndexerPlugin*> prevLoopPlugins = plugins;
        while (!plugins.empty()) {
            groups.resize(groups.size() + 1);
            for (auto p = plugins.begin(); p != plugins.end();) {
                auto deps = (*p)->dependsOn();

                bool skipUntilLater = false;
                for (auto* p : plugins) {
                    if (find(deps.begin(), deps.end(), p->name()) != deps.end()) {
                        skipUntilLater = true;
                        break;
                    }
                }
                for (auto dep : groups.back()) {
                    if (find(deps.begin(), deps.end(), dep.plugin->name()) != deps.end()) {
                        skipUntilLater = true;
                        break;
                    }
                }
                if (!skipUntilLater) {
                    groups.back().push_back({*p, false});
                    p = plugins.erase(p);
                } else {
                    ++p;
                }
            }

            // Check for dependency resolution failure
            if (plugins == prevLoopPlugins) {
                cerr << "Unable to resolve all plugin dependencies. "
                        "Plugins left to resolve:" << endl << endl;
                for (auto* p : plugins) {
                    cerr << p->name() << endl
                         << "  Depends on:" << endl;
                    for (auto d : p->dependsOn())
                         cerr << "    " << d << endl;
                    cerr << endl;
                }
                exit(1);
            }
            prevLoopPlugins = plugins;
        }
        return groups;
    };

    // We do not own all the memory in here. We only own the default prorgam
    vector<vector<PluginRuntimeInfo>> pluginGroups;
    pluginGroups = buildPluginGroups(plugins);
    if (pluginGroups.size() > 1) {
        cout << "Identified " << pluginGroups.size() << " indexer plugin groups" << endl
             << "Running through log " << pluginGroups.size()
             << " times to satisfy dependencies" << endl;
    }

    zcm::Json::Value index;

    size_t numEvents = 0;
    const zcm::LogEvent* evt;
    for (size_t i = 0; i < pluginGroups.size(); ++i) {
        cout << "Running in parallel plugin group " << i << ": [" << endl;
        for (auto p : pluginGroups[i]) cout << "  " << p.plugin->name() << "," << endl;
        cout << "]" << endl;

        off_t offset = 0;
        fseeko(log.getFilePtr(), 0, SEEK_SET);

        bool anyRemaining = false;
        for (auto& p : pluginGroups[i]) {
            p.runThroughLog = p.plugin->setUp(index, index[p.plugin->name()], log);
            anyRemaining |= p.runThroughLog;
        }

        if (anyRemaining) {
            fseeko(log.getFilePtr(), 0, SEEK_SET);

            static int lastPrintPercent = 0;
            while (1) {
                offset = ftello(log.getFilePtr());

                int percent = 100.0 * offset / logSize;
                if (percent != lastPrintPercent) {
                    cout << "\r" << "Percent Complete: " << percent << flush;
                    lastPrintPercent = percent;
                }

                evt = log.readNextEvent();
                if (evt == nullptr) break;

                int64_t msg_hash;
                __int64_t_decode_array(evt->data, 0, 8, &msg_hash, 1);
                const TypeMetadata* md = types.getByHash(msg_hash);

                for (auto& p : pluginGroups[i]) {
                    assert(p.plugin);

                    if (!p.runThroughLog) continue;

                    p.plugin->indexEvent(index, index[p.plugin->name()],
                                         evt->channel, md ? md->name : "",
                                         offset, evt->timestamp,
                                         (uint64_t) msg_hash,
                                         evt->data, evt->datalen);

                    numEvents++;
                }
            }
            if (lastPrintPercent != 100)
                cout << "\r" << "Percent Complete: 100" << flush;
            cout << endl;
        }

        for (auto& p : pluginGroups[i]) {
            fseeko(log.getFilePtr(), 0, SEEK_SET);
            p.plugin->tearDown(index, index[p.plugin->name()], log);
        }
    }

    for (auto p : defaults) delete p.plugin;
    defaults.clear();

    cout << "Indexed " << numEvents << " events" << endl;

    zcm::Json::StreamWriterBuilder builder;
    builder["indentation"] = args.readable ? "    " : "";
    std::unique_ptr<zcm::Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(index, &output);
    if (output.fail()) {
        cerr << "Failure occurred while writing output file" << endl;
        return 1;
    }
    output << endl;
    output.close();
    if (output.fail()) {
        cerr << "Failure occurred while closing output file" << endl;
        return 1;
    }
    return 0;
}
