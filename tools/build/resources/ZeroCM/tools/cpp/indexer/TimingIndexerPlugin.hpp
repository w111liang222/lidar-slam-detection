#pragma once

#include "zcm/tools/IndexerPlugin.hpp"

class TimingIndexerPlugin : public zcm::IndexerPlugin
{
  public:
    static TimingIndexerPlugin* makeIndexerPlugin();

    virtual ~TimingIndexerPlugin();

    virtual std::string name() const;

    virtual void indexEvent(const zcm::Json::Value& index,
                            zcm::Json::Value& pluginIndex,
                            std::string channel,
                            std::string typeName,
                            off_t offset,
                            uint64_t timestamp,
                            int64_t hash,
                            const uint8_t* data,
                            int32_t datalen);

    virtual void tearDown(const zcm::Json::Value& index,
                          zcm::Json::Value& pluginIndex,
                          zcm::LogFile& log);
};
