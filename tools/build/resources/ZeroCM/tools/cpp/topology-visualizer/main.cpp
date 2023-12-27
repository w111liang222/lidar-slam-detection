#include <iostream>
#include <fstream>
#include <getopt.h>
#include <dirent.h>
#include <sys/stat.h>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>
#include <regex>
#include <vector>

#include "zcm/json/json.h"
#include "zcm/util/debug.h"

#include "util/StringUtil.hpp"
#include "util/TypeDb.hpp"

using namespace std;

struct Args
{
    string topology_dir = "";
    string output       = "";
    string type_path    = "";
    bool nogui          = false;
    bool debug          = false;

    // groupName -> { beginIdx in names, endIdx in names }
    vector<pair<string, unordered_set<string>>> groups;
    // groupName -> color
    unordered_map<string, string> colors;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "d:o:t:ug:n:c:h";
        struct option long_opts[] = {
            { "topology-dir",  required_argument, 0, 'd' },
            { "output",        required_argument, 0, 'o' },
            { "type-path",     required_argument, 0, 't' },
            { "no-gui",        no_argument,       0, 'u' },
            { "debug",         no_argument,       0,  0  },
            { "group",         required_argument, 0, 'g'  },
            { "name",          required_argument, 0, 'n'  },
            { "color",         required_argument, 0, 'c'  },
            { "help",          no_argument,       0, 'h' },
            { 0, 0, 0, 0 }
        };

        int c;
        int option_index;
        while ((c = getopt_long(argc, argv, optstring, long_opts, &option_index)) >= 0) {
            switch (c) {
                case 'd': topology_dir = string(optarg); break;
                case 'o': output       = string(optarg); break;
                case 't': type_path    = string(optarg); break;
                case 'u': nogui        = true; break;
                case 'g': groups.push_back({ optarg, {} }); break;
                case 'n': {
                    if (groups.empty()) groups.push_back({ "", {} });
                    groups.back().second.emplace(optarg); break;
                }
                case 'c': {
                    if (groups.empty()) groups.push_back({ "", {} });
                    colors[groups.back().first] = optarg; break;
                }
                case  0: {
                    string longopt = string(long_opts[option_index].name);
                    if (longopt == "debug") debug = true;
                    break;
                }
                case 'h': default: usage(); return false;
            };
        }

        if (topology_dir == "") {
            cerr << "Please specify a topology directory to "
                 << "read topology json files from" << endl;
            return false;
        }
        if (topology_dir.back() != '/') topology_dir += "/";

        if (output  == "") {
            cerr << "Please specify dotvis file output" << endl;
            return false;
        }

        if (type_path == "") {
            cerr << "Please specify a zcmtypes.so path" << endl;
            return false;
        }

        if (groups.empty()) groups.push_back({ "", {".*"} });

        for (auto g : groups) {
            stringstream ss;
            ss << "Input group: " << (g.first == "" ? "<ungrouped>" : g.first);
            if (colors.count(g.first) > 0) ss << " - " << colors.at(g.first);
            ss << endl;
            for (auto n : g.second) {
                ss << "\t" << n << endl;
            }
            ZCM_DEBUG("%s", ss.str().c_str());
        }

        return true;
    }

    void usage()
    {
        cout << "usage: zcm-topology-visualizer [options]" << endl
             << "" << endl
             << "    Scrape through a topology directory full of topology json files" << endl
             << "    and generate a single dotvis file output that visualizes all of the" << endl
             << "    different messages received and sent on different channels" << endl
             << "" << endl
             << "Example:" << endl
             << "    zcm-topology-visualizer -d /tmp/zcm_topology -o /tmp/topology.dot -t path/to/zcmtypes.so" << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -h, --help              Shows this help text and exits" << endl
             << "  -d, --topology-dir=dir  Directory to scrape for topology json files" << endl
             << "  -o, --output=file       Output dotvis file" << endl
             << "  -t, --type-path=path    Path to shared library containing zcmtypes" << endl
             << "  -n, --name=name         Basename of file you want to visualize. " << endl
             << "                          If this argument is ommited, all files will" << endl
             << "                          be visualized. This argument can be specified" << endl
             << "                          multiple times." << endl
             << "  -g, --group=group       Group all subsequently specified -n args into" << endl
             << "                          a subgraph in the visualization." << endl
             << "                          For example: " << endl
             << "                          -g <g1> -n <n1.*> -g <g2> -n <n2.*>" << endl
             << "      --debug             Run a dry run to ensure proper indexer setup" << endl
             << endl << endl;
    }
};

int walkFiles(const string& directory, vector<string>& files)
{
    DIR *dp = opendir(directory.c_str());
    if (dp == nullptr) {
        cerr << "Unable to open directory: " << directory << endl;
        return 1;
    }

    struct dirent *entry = nullptr;
    while ((entry = readdir(dp))) {
        string name(entry->d_name);
        if (name == "." || name == "..") continue;
        if (!StringUtil::endswith(name, ".json")) continue;

        struct stat info;
        string fullpath = directory + name;
        int ret = stat(fullpath.c_str(), &info);
        if (ret != 0) {
            cerr << "Unable to open file: " << directory << entry->d_name << endl;
            perror("stat");
            continue;
        }
        if (!S_ISREG(info.st_mode)) continue;

        files.push_back(fullpath);
    }

    if (closedir(dp) != 0) return 1;

    return 0;
}

void buildIndex(zcm::Json::Value& index, const vector<string>& files)
{
    for (const auto& f : files) {
        ifstream infile{ f };
        if (!infile.good()) {
            cerr << "Unable to open file: " << f << endl;
            continue;
        }

        zcm::Json::Value root;
        zcm::Json::Reader reader;
        if (!reader.parse(infile, root, false)) {
            cerr << "Failed to parse json file: " << f << endl;
            cerr << reader.getFormattedErrorMessages() << endl;
            continue;
        }

        index[root["name"].asString()] = root;
    }
}

int writeOutput(const zcm::Json::Value& index,
                const vector<pair<string, unordered_set<string>>>& groupsToInclude,
                const unordered_map<string, string> colors,
                const string& outpath,
                const TypeDb& types)
{
    ofstream output{ outpath };
    if (!output.good()) {
        cerr << "Unable to open output file: " << outpath << endl;
        return 1;
    }
    output << "digraph arch {" << endl;
    output << endl;

    map<string, unordered_set<string>> groups;
    auto isInGroups = [&groups](const string& n){
        unordered_set<string> allnames;
        for (auto g : groups) {
            allnames.insert(g.second.begin(), g.second.end());
        }
        return allnames.count(n) > 0;
    };
    auto getNodeId = [&groups](const string& n, string& ret){
        size_t i = 0;
        for (auto g : groups) {
            for (auto gn : g.second) {
                if (n == gn) {
                    ret = string("node_") + to_string(i);
                    return true;
                }
                ++i;
            }
        }
        return false;
    };

    // Build list of groups of names
    for (auto n : index.getMemberNames()) {
        if (isInGroups(n)) continue;
        for (auto g : groupsToInclude) {
            bool found = false;
            for (auto gn : g.second) {
                if (regex_match(n, regex(gn))) {
                    groups[g.first].insert(n);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }

    output << "  subgraph cluster_legend {" << endl
           << "    labelloc=t" << endl
           << "    label=Legend" << endl
           << "    shape=rectangle" << endl
           << "    color=black" << endl << endl;
    size_t i = 0;
    for (auto g : groupsToInclude) {
        if (colors.count(g.first) == 0) continue;
        string groupName = g.first == "" ? "<default>" : g.first;
        output << "    group_" << i << "["
               << "shape=oval "
               << "label=" << zcm::Json::Value(groupName) << " "
               << "style=filled "
               << "fillcolor=" << zcm::Json::Value(colors.at(g.first)) << " "
               << "border=none"
               << "]" << endl;
        ++i;
    }
    output << endl;
    for (size_t j = 1; j < i; ++j) {
        output << "    group_" << (j - 1) << " -> group_" << j << " [style=invis]" << endl;
    }
    output << "  }" << endl << endl;

    output << "  subgraph cluster_body {" << endl
           << "    style=invis" << endl
           << endl;

    i = 0;
    for (auto g : groups) {
        for (auto n : g.second) {
            string node;
            assert(getNodeId(n, node) && "This should not be possible");
            output << "    "
                   << node << " ["
                   << "label=" << zcm::Json::Value(n) << " "
                   << "shape=oval ";
            if (colors.count(g.first) > 0)
                output << "style=filled "
                       << "fillcolor=" << zcm::Json::Value(colors.at(g.first)) << " ";
            output << "]" << endl;
        }
        ++i;
    }
    output << endl;

    unordered_map<string, string> channels;
    for (auto n : index.getMemberNames()) {
        if (!isInGroups(n)) continue;
        for (auto chan : index[n]["publishes"].getMemberNames()) {
            if (channels.count(chan) > 0) continue;
            size_t i = channels.size();
            channels[chan] = string("channel") + to_string(i);
        }
        for (auto chan : index[n]["subscribes"].getMemberNames()) {
            if (channels.count(chan) > 0) continue;
            size_t i = channels.size();
            channels[chan] = string("channel") + to_string(i);
        }
    }

    for (auto c : channels) {
        output << "    " << c.second << " ["
               << "label=" << zcm::Json::Value(c.first) << " "
               << "shape=rectangle"
               << "]" << endl;
    }

    output << endl << endl;

    for (auto n : index.getMemberNames()) {
        if (!isInGroups(n)) continue;
        string node;
        assert(getNodeId(n, node) && "This should not be possible");
        for (auto channel : index[n]["publishes"].getMemberNames()) {
            auto s = index[n]["publishes"][channel];
            vector<string> typeNames;
            for (auto t : s) {
                int64_t hashBE = stoll(t["BE"].asString());
                int64_t hashLE = stoll(t["LE"].asString());
                const TypeMetadata* md = types.getByHash(hashBE);
                if (!md) md = types.getByHash(hashLE);
                if (!md) {
                    cerr << "Unable to find matching type for hash pair" << endl
                         << n << " -> " << channel << ": " << t << endl;
                    continue;
                }
                typeNames.push_back(md->name);
            }
            output << "    " << node << " -> " << channels[channel] << " [label=\"";
            for (auto t : typeNames) output << t << " ";
            output << "\" color=blue]" << endl;
        }
        for (auto channel : index[n]["subscribes"].getMemberNames()) {
            auto s = index[n]["subscribes"][channel];
            vector<string> typeNames;
            for (auto t : s) {
                int64_t hashBE = stoll(t["BE"].asString());
                int64_t hashLE = stoll(t["LE"].asString());
                const TypeMetadata* md = types.getByHash(hashBE);
                if (!md) md = types.getByHash(hashLE);
                if (!md) {
                    cerr << "Unable to find matching type for hash pair" << endl
                         << n << " -> " << channel << ": " << t << endl;
                    continue;
                }
                typeNames.push_back(md->name);
            }
            output << "    " << channels[channel] << " -> " << node << " [label=\"";
            for (auto t : typeNames) output << t << " ";
            output << "\" color=red]" << endl;
        }
    }

    output << "  }" << endl
           << "}" << endl;

    output.close();

    return 0;
}

int main(int argc, char *argv[])
{
    Args args;
    if (!args.parse(argc, argv)) return 1;

    TypeDb types(args.type_path, args.debug);

    if (!types.good()) return 1;

    if (args.debug) return 0;

    vector<string> files;
    int ret = walkFiles(args.topology_dir, files);
    if (ret != 0) return ret;

    zcm::Json::Value index;
    buildIndex(index, files);

    ret = writeOutput(index, args.groups, args.colors, args.output, types);
    if (ret != 0) return ret;

    ret = 1;
    if (!args.nogui) ret = execlp("xdot", "xdot", args.output.c_str(), nullptr);
    if (ret != 0) {
        cout << "Successfully wrote file to " << args.output << endl;
        cout << "Run `xdot " << args.output << "` to view output" << endl;
    }

    return 0;
}
