#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cxxabi.h>
#include <typeinfo>
#include <string>

#include "TimingIndexerPlugin.hpp"

TimingIndexerPlugin* TimingIndexerPlugin::makeIndexerPlugin()
{ return new TimingIndexerPlugin(); }

TimingIndexerPlugin::~TimingIndexerPlugin()
{}

std::string TimingIndexerPlugin::name() const
{ return "timestamp"; }

// Index every message according to timestamp
void TimingIndexerPlugin::indexEvent(const zcm::Json::Value& index,
                               zcm::Json::Value& pluginIndex,
                               std::string channel,
                               std::string typeName,
                               off_t offset,
                               uint64_t timestamp,
                               int64_t hash,
                               const uint8_t* data,
                               int32_t datalen)
{ pluginIndex[channel][typeName].append(std::to_string(offset)); }

void TimingIndexerPlugin::tearDown(const zcm::Json::Value& index,
                             zcm::Json::Value& pluginIndex,
                             zcm::LogFile& log)
{
    std::cout << "sorting " << name() << std::endl;

    fseeko(log.getFilePtr(), 0, SEEK_END);
    off_t logSize = ftello(log.getFilePtr());

    auto comparator = [&](off_t a, off_t b) {
        if (a < 0 || b < 0 || a > logSize || b > logSize) {
            std::cerr << "Sorting has failed. "
                      << "Sorting function is probably broken. "
                      << "Aborting." << std::endl;
            exit(1);
        }
        return a < b;
    };

    for (std::string channel : pluginIndex.getMemberNames()) {
        for (std::string type : pluginIndex[channel].getMemberNames()) {
            std::vector<off_t> offsets;
            for (size_t i = 0; i < pluginIndex[channel][type].size(); ++i) {
                std::string offset = pluginIndex[channel][type][(int)i].asString();
                size_t sz = 0;
                long long off = stoll(offset, &sz, 0);
                assert(sz <= offset.length());
                offsets.push_back((off_t) off);
            }
            std::sort(offsets.begin(), offsets.end(), comparator);
            for (size_t i = 0; i < pluginIndex[channel][type].size(); ++i) {
                pluginIndex[channel][type][(int)i] = std::to_string(offsets[i]);
            }
        }
    }


}
