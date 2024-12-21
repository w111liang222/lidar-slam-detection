#ifndef __SYSTEM_UTILS_H
#define __SYSTEM_UTILS_H

#include <thread>
#include <chrono>

#include "Logger.h"

void setThreadPriority(std::thread* thread, int priority);
void setSelfThreadPriority(int priority);
void setSelfThreadAffinity(int core_id);
uint64_t getCurrentTime();
uint64_t getMonotonicTime();
uint64_t gps2Utc(int gps_week, double gps_time);
int getGPSweek(const uint64_t &stamp);
double getGPSsecond(const uint64_t &stamp);
int setSystemTime(uint64_t time);
void createDirectory(std::string path);
std::string splitFilename(const std::string& str);
void backtrace_handle_init(void);

template <
    class result_t   = std::chrono::milliseconds,
    class clock_t    = std::chrono::steady_clock,
    class duration_t = std::chrono::milliseconds
>
auto since(std::chrono::time_point<clock_t, duration_t> const& start)
{
    return std::chrono::duration_cast<result_t>(clock_t::now() - start);
}

inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

#endif  //__SYSTEM_UTILS_H