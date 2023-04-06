#include "zcm/util/debug.h"

#ifdef ZCM_DEBUG_ENV_ENABLE
bool ZCM_DEBUG_ENABLED;

#include <mutex>
static std::mutex mut;
void zcm_debug_lock(void) { mut.lock(); }
void zcm_debug_unlock(void) { mut.unlock(); }

// This code runs before the main() function to load check if
// the ZCM_DEBUG env-var is set
struct BeforeMainCode
{
    BeforeMainCode()
    { ZCM_DEBUG_ENABLED = (NULL != getenv("ZCM_DEBUG")); }
};
static BeforeMainCode run;

#endif  // ZCM_DEBUG_ENV_ENABLE
