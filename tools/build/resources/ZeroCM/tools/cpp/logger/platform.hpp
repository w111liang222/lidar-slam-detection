#ifndef ZCM_LOGGER_PLATFORM
#define ZCM_LOGGER_PLATFORM

#ifdef WIN32
#define __STDC_FORMAT_MACROS            // Enable integer types
// TODO re-enable windows support
//#include <zcm/windows/WinPorting.h>
#else
# include <sys/time.h>
# include <unistd.h>
# include <linux/limits.h>
#endif

struct Platform
{
    static inline void fflush(FILE *fp)
    {
        ::fflush(fp);
#ifndef WIN32
        // Perform a full fsync operation after flush
        fdatasync(fileno(fp));
#endif
    }

    static inline void setstreambuf()
    {
#ifndef WIN32
        setlinebuf(stdout);
#endif
    }
};

#endif
