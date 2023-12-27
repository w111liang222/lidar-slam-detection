#ifndef _ZCM_DEBUG_H
#define _ZCM_DEBUG_H

#include <stdlib.h>
#include <stdbool.h>

#define ZCM_DEBUG_ENV_ENABLE

#ifdef __cplusplus
# define CFUNC extern "C"
#else
# define CFUNC
#endif

#ifdef ZCM_DEBUG_ENV_ENABLE
extern bool ZCM_DEBUG_ENABLED;
CFUNC void zcm_debug_lock(void);
CFUNC void zcm_debug_unlock(void);
# define ZCM_DEBUG(...) do { \
    if (ZCM_DEBUG_ENABLED) {\
        zcm_debug_lock();\
        fprintf(stderr, "ZCM-DEBUG: ");\
        fprintf(stderr, __VA_ARGS__);\
        fprintf(stderr, "\n");\
        fflush(stderr);\
        zcm_debug_unlock();\
    }} while(0)
#else
# define ZCM_DEBUG(...)
#endif

#endif
