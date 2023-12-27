#pragma once

// ZCM Headers
#include "zcm/zcm.h"
#include "zcm/zcm_coretypes.h"
#include "util/Types.hpp"
#include "util/StringUtil.hpp"
#include "util/TimeUtil.hpp"

// C++ Stdlib headers
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <cinttypes>
#include <cstdarg>
#include <cassert>
using namespace std;

// Helpers
template<class K, class V>
static inline V *lookup(std::unordered_map<K,V>& map, const K& key)
{
    auto it = map.find(key);
    if (it == map.end())
        return NULL;
    else
        return &it->second;
}

// Posix headers
#include <dlfcn.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
