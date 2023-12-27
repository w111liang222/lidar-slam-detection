#pragma once
#include "Common.hpp"

#define DEBUG_LEVEL 2  /* 0=nothing, higher values mean more verbosity */
#define DEBUG_FILENAME "/tmp/spy-lite-debug.log"

extern FILE *DEBUG_FILE;
static inline void DEBUG_INIT(void)
{
    // DEBUG_FILE = fopen(DEBUG_FILENAME, "w");
}

static inline void DEBUG(int level, const char *fmt, ...)
{
    // if(!DEBUG_LEVEL || level > DEBUG_LEVEL)
    //     return;

    // va_list vargs;
    // va_start(vargs, fmt);
    // vfprintf(DEBUG_FILE, fmt, vargs);
    // va_end(vargs);

    // fflush(DEBUG_FILE);
}
