#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

#include <cstdint>
#include <cstdarg>
#include <cinttypes>
#include <cstring>
#include <cassert>
#include <climits>
#include <cfloat>

#include <sys/stat.h>

using std::string;
using std::vector;
using std::unordered_map;
using std::unordered_set;

typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;

typedef int8_t    i8;
typedef int16_t   i16;
typedef int32_t   i32;
typedef int64_t   i64;

static void merge(unordered_set<string>& a, const unordered_set<string>& b)
{ a.insert(b.begin(), b.end()); }
