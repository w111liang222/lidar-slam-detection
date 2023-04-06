#pragma once
#include <vector>

#include "util/TypeDb.hpp"
#include "Common.hpp"

struct MsgDisplayState
{
    std::vector<size_t> recur_table;
};

void msg_display(TypeDb& db, const TypeMetadata& metadata, void *msg, const MsgDisplayState& state);
