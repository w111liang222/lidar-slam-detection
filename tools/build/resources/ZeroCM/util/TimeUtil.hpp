#pragma once
#include <sys/time.h>
#include "util/Types.hpp"

namespace TimeUtil
{
    static u64 utime()
    {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return (u64)tv.tv_sec * 1000000 + tv.tv_usec;
    }
}
