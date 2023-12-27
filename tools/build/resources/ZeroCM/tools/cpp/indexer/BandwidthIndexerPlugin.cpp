#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cxxabi.h>
#include <typeinfo>
#include <string>

#include "BandwidthIndexerPlugin.hpp"

BandwidthIndexerPlugin* BandwidthIndexerPlugin::makeIndexerPlugin()
{ return new BandwidthIndexerPlugin(); }

BandwidthIndexerPlugin::~BandwidthIndexerPlugin()
{}

std::string BandwidthIndexerPlugin::name() const
{ return "bandwidth"; }

// Index every message according to timestamp
void BandwidthIndexerPlugin::indexEvent(const zcm::Json::Value& index,
                                        zcm::Json::Value& pluginIndex,
                                        std::string channel,
                                        std::string typeName,
                                        off_t offset,
                                        uint64_t timestamp,
                                        int64_t hash,
                                        const uint8_t* data,
                                        int32_t datalen)
{
    if (startTimestamp == std::numeric_limits<uint64_t>::max())
        startTimestamp = timestamp;
    double secondsSoFar = (double)(timestamp - startTimestamp) / 1e6;

    pluginIndex[channel][typeName]["totalBytes"] =
        pluginIndex[channel][typeName]["totalBytes"].asUInt64() + (uint64_t) datalen;

    pluginIndex[channel][typeName]["totalMessages"] =
        pluginIndex[channel][typeName]["totalMessages"].asUInt64() + 1;

    pluginIndex[channel][typeName]["averageBytesPerMessage"] =
        pluginIndex[channel][typeName]["totalBytes"].asDouble() /
        pluginIndex[channel][typeName]["totalMessages"].asDouble();

    pluginIndex[channel][typeName]["averageBytesPerSecond"] =
        (zcm::Json::Value::UInt64) (pluginIndex[channel][typeName]["totalBytes"].asDouble() / secondsSoFar);
}
