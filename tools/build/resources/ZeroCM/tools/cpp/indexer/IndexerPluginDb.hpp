#pragma once

#include <string>

#include "zcm/tools/IndexerPlugin.hpp"

struct IndexerPluginMetadata
{
    std::string className;
    zcm::IndexerPlugin* (*makeIndexerPlugin)(void);
    inline bool operator==(const IndexerPluginMetadata& o) { return className == o.className; }
};

class IndexerPluginDb
{
  public:
    // Note paths is a ":" delimited list of paths just like $PATH
    IndexerPluginDb(const std::string& paths = "", bool debug = false);
    ~IndexerPluginDb();
    std::vector<const zcm::IndexerPlugin*> getPlugins();

  private:
    bool findPlugins(const std::string& libname);
    bool debug;
    std::vector<IndexerPluginMetadata> pluginMeta;
    std::vector<zcm::IndexerPlugin*> plugins;
    std::vector<const zcm::IndexerPlugin*> constPlugins;
};
