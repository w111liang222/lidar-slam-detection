#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <cxxabi.h>
#include <typeinfo>
#include <string>

#include "IndexerPlugin.hpp"

using namespace zcm;

IndexerPlugin* IndexerPlugin::makeIndexerPlugin()
{ return new IndexerPlugin(); }

IndexerPlugin::~IndexerPlugin()
{}

std::string IndexerPlugin::name() const
{ return ""; }

std::set<std::string> IndexerPlugin::dependsOn() const
{ return {}; }

bool IndexerPlugin::setUp(const zcm::Json::Value& index,
                          zcm::Json::Value& pluginIndex,
                          zcm::LogFile& log)
{ return true; }

// Index every message according to timestamp
void IndexerPlugin::indexEvent(const zcm::Json::Value& index,
                               zcm::Json::Value& pluginIndex,
                               std::string channel,
                               std::string typeName,
                               off_t offset,
                               uint64_t timestamp,
                               int64_t hash,
                               const uint8_t* data,
                               int32_t datalen)
{}

void IndexerPlugin::tearDown(const zcm::Json::Value& index,
                             zcm::Json::Value& pluginIndex,
                             zcm::LogFile& log)
{}
