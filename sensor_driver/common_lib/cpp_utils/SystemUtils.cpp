#include "SystemUtils.h"

#define BOOST_STACKTRACE_USE_ADDR2LINE
#include <sys/time.h>
#include <sched.h>

#include <boost/stacktrace.hpp>
#include <iostream>
#include <sstream>
#include <cstring>

#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <unistd.h>

void setThreadPriority(std::thread* thread, int priority) {
  sched_param sched;
  int policy;
  pthread_getschedparam(thread->native_handle(), &policy, &sched);
  sched.sched_priority = priority;
  if (pthread_setschedparam(thread->native_handle(), SCHED_RR, &sched)) {
    LOG_WARN("Failed to setschedparam: {}", std::strerror(errno));
  }
}

void setSelfThreadPriority(int priority) {
  sched_param sched;
  int policy;
  pthread_t current_thread = pthread_self();
  pthread_getschedparam(current_thread, &policy, &sched);
  sched.sched_priority = priority;
  if (pthread_setschedparam(current_thread, SCHED_FIFO, &sched)) {
    LOG_WARN("Failed to setschedparam: {}", std::strerror(errno));
  }
}

void setSelfThreadAffinity(int core_id) {
  int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
  if (core_id < 0 || core_id >= num_cores)
    return;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(core_id, &cpuset);

  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset)) {
    LOG_WARN("Failed to pthread_setaffinity_np: {}", std::strerror(errno));
  }
}

uint64_t getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint64_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

uint64_t getMonotonicTime() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return static_cast<uint64_t>(ts.tv_sec * 1000000 + ts.tv_nsec / 1000ULL);
}

#define SECS_PER_WEEK (60L*60*24*7)
#define LEAP_SECOND 18
uint64_t gps2Utc(int gps_week, double gps_time) {
    return uint64_t(SECS_PER_WEEK * gps_week * 1000000ULL + (gps_time * 1000000ULL) + 315964800000000ULL - LEAP_SECOND * 1000000ULL);
}

int getGPSweek(const uint64_t &stamp) {
  double diff = (stamp - 315964800000000ULL) / 1000000.0;
  return (int) (diff / SECS_PER_WEEK);
}

double getGPSsecond(const uint64_t &stamp) {
  double diff = (stamp - 315964800000000ULL) / 1000000.0;
  return (diff - (int) (diff / SECS_PER_WEEK) * SECS_PER_WEEK + LEAP_SECOND);
}

int setSystemTime(uint64_t time) {
  double tsec = time / 1000000.0;
  struct timespec ts;
  ts.tv_sec = int(tsec);
  ts.tv_nsec = int((tsec - ts.tv_sec) * 1000000000);
  return clock_settime(CLOCK_REALTIME, &ts);
}

void createDirectory(std::string path) {
  std::string command = "mkdir";
  command.append(" -p ");
  command.append(path);
  if (system(command.c_str()) != 0) {
    LOG_WARN("error: {}", command);
  }
}

std::string splitFilename(const std::string& str) {
  std::size_t found = str.find_last_of("/\\");
  return str.substr(found + 1);
}

void print_trace() {
    char pid_buf[30];
    sprintf(pid_buf, "--pid=%d", getpid());
    char name_buf[512];
    name_buf[readlink("/proc/self/exe", name_buf, 511)]=0;
    char gdb_buf[1024];
    sprintf(gdb_buf, "gdb --batch -n -ex \"thread apply all bt\" %s %s | tee output/logs/crash.log", name_buf, pid_buf);
    int child_pid = fork();
    if (!child_pid) {
        dup2(2,1); // redirect output to stderr
        fprintf(stdout,"stack trace for %s pid=%s\n",name_buf,pid_buf);
        int result = system(gdb_buf);
        // execlp("gdb", "gdb", "--batch", "-n", "-ex", "thread apply all bt", name_buf, pid_buf, NULL);
        // abort(); /* If gdb failed to start */
    } else {
        waitpid(child_pid,NULL,0);
    }
}

void signal_handler(int sig) {
  print_trace();
  signal(sig, SIG_DFL);
  raise(sig);
}

void backtrace_handle_init(void) {
  signal(SIGSEGV, signal_handler);
  signal(SIGABRT, signal_handler);
  signal(SIGBUS, signal_handler);
  signal(SIGCHLD, SIG_IGN);
}