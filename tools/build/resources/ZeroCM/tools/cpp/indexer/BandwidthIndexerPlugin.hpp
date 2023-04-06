#pragma once

#include <limits>

#include "zcm/tools/IndexerPlugin.hpp"

class BandwidthIndexerPlugin : public zcm::IndexerPlugin
{
  private:
    uint64_t startTimestamp = std::numeric_limits<uint64_t>::max();

  public:
    static BandwidthIndexerPlugin* makeIndexerPlugin();

    virtual ~BandwidthIndexerPlugin();

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
};
