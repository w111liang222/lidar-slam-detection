#pragma once

#include <cstdio>

#define ERROR(...) do {\
    fprintf(stderr, "Err: ");\
    fprintf(stderr, __VA_ARGS__);\
  } while(0)
