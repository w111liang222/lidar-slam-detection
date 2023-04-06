#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace zcm {

struct TopologyPairHash
{
    typedef std::pair<int64_t, int64_t> argument_type;
    typedef size_t result_type;
    std::hash<int64_t> hash;
    result_type operator()(argument_type const& s) const noexcept
    {
        return hash(s.first);
    }
};

// channel_name -> { big_endian_type_hash, little_endian_type_hash }
typedef std::unordered_map<std::string,
                           std::unordered_set<std::pair<int64_t,int64_t>,
                                              TopologyPairHash>> TopologyMap;

int writeTopology(const std::string& name,
                  const TopologyMap& receivedTopologyMap,
                  const TopologyMap& sentTopologyMap);

} /* zcm */
