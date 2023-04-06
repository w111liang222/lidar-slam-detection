#pragma once

#include <string>
#include <vector>

#include "zcm/tools/TranscoderPlugin.hpp"

struct TranscoderPluginMetadata
{
    std::string className;
    zcm::TranscoderPlugin* (*makeTranscoderPlugin)(void);

    inline bool operator==(const TranscoderPluginMetadata& o)
    { return className == o.className; }
};

class TranscoderPluginDb
{
  public:
    // Note paths is a ":" delimited list of paths just like $PATH
    TranscoderPluginDb(const std::string& paths = "", bool debug = false);
    ~TranscoderPluginDb();
    std::vector<const zcm::TranscoderPlugin*> getPlugins() const;
    std::vector<std::string> getPluginNames() const;

  private:
    bool findPlugins(const std::string& libname);
    bool debug;
    std::vector<TranscoderPluginMetadata> pluginMeta;
    std::vector<zcm::TranscoderPlugin*> plugins;
    std::vector<std::string> names;
    std::vector<const zcm::TranscoderPlugin*> constPlugins;
};
