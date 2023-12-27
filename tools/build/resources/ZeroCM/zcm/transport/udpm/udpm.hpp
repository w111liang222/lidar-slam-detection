#pragma once

// Headers from C library
#include <cstdlib>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <ctime>
#include <cassert>

// TODO: get rid of these
#include <thread>
#include <mutex>
#include <condition_variable>

// Headers for C++ library
#include <algorithm>
#include <vector>
#include <stack>
#include <unordered_map>
#include <string>
using namespace std;

// Headers needed on Unix
#ifndef WIN32
# include <unistd.h>
# include <fcntl.h>
# include <arpa/inet.h>
# include <netdb.h>
# include <netinet/in.h>
# include <sys/time.h>
# include <sys/uio.h>
# include <sys/socket.h>
# include <poll.h>
# include <sys/select.h>
typedef int SOCKET;
#endif

// Misc. Compatability
#ifdef SO_TIMESTAMP
# define MSG_EXT_HDR
#endif

// HUGE is not defined on cygwin as of 2008-03-05
#ifndef HUGE
# define HUGE 3.40282347e+38F
#endif

#ifdef __APPLE__
# define USE_REUSEPORT
#endif
#ifdef __FreeBSD__
# define USE_REUSEPORT
#endif

// Headers needed on Windows
#ifdef WIN32
# include "windows/WinPorting.h"
# include <winsock2.h>
# include <Ws2tcpip.h>
# define MSG_EXT_HDR
#endif

// Headers for ZCM API
#include "zcm/transport.h"
#include "zcm/util/debug.h"

// Useful integer typedefs
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;

#define zcm_internal_pipe_write write
#define zcm_internal_pipe_read read
#define zcm_internal_pipe_close close
#define zcm_internal_pipe_create pipe

/************************* Important Defines *******************/
#define ZCM_MAGIC_SHORT 0x4c433032   // hex repr of ascii "LC02"
#define ZCM_MAGIC_LONG  0x4c433033   // hex repr of ascii "LC03"

#ifdef __APPLE__
# define ZCM_SHORT_MESSAGE_MAX_SIZE 1435
# define ZCM_FRAGMENT_MAX_PAYLOAD 1423
#else
# define ZCM_SHORT_MESSAGE_MAX_SIZE 65499
# define ZCM_FRAGMENT_MAX_PAYLOAD 65487
#endif

#define ZCM_RINGBUF_SIZE (200*1024)
#define ZCM_DEFAULT_RECV_BUFS 2000
#define ZCM_MAX_UNFRAGMENTED_PACKET_SIZE 65536

#define MAX_FRAG_BUF_TOTAL_SIZE (1 << 24)// 16 megabytes
#define MAX_NUM_FRAG_BUFS 1000

#define SELF_TEST_CHANNEL "LCM_SELF_TEST"
