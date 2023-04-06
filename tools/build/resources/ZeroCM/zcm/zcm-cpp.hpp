#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include "zcm/zcm.h"

#ifndef ZCM_EMBEDDED
#include "zcm/eventlog.h"
#endif

#if __cplusplus > 199711L
#include <functional>
#endif

namespace zcm {

typedef zcm_recv_buf_t ReceiveBuffer;
typedef zcm_msg_handler_t MsgHandler;
class Subscription;

class ZCM
{
  public:
    #ifndef ZCM_EMBEDDED
    inline ZCM();
    inline ZCM(const std::string& transport);
    #endif
    inline ZCM(zcm_trans_t* zt);
    virtual inline ~ZCM();

    virtual inline bool good() const;
    virtual inline int err() const; // errno is a reserved name, so this returns zcm_errno()
    virtual inline const char* strerror() const;
    virtual inline const char* strerrno(int err) const;

    #ifndef ZCM_EMBEDDED
    virtual inline void run();
    virtual inline void start();
    virtual inline void stop();
    virtual inline void pause();
    virtual inline void resume();
    virtual inline int  handle();
    virtual inline void setQueueSize(uint32_t sz);
    virtual inline int  writeTopology(const std::string& name);
    #endif
    virtual inline int  handleNonblock();
    virtual inline void flush();

  public:
    inline int publish(const std::string& channel, const uint8_t* data, uint32_t len);

    // Note: if we make a publish binding that takes a const message reference, the compiler does
    //       not select the right version between the pointer and reference versions, so when the
    //       user intended to call the pointer version, the reference version is called and causes
    //       compile errors (turns the input into a double pointer). We have to choose one or the
    //       other for the api.
    template <class Msg>
    inline int publish(const std::string& channel, const Msg* msg);

    inline Subscription* subscribe(const std::string& channel,
                                   void (*cb)(const ReceiveBuffer* rbuf,
                                              const std::string& channel,
                                              void* usr),
                                   void* usr);

    template <class Msg, class Handler>
    inline Subscription* subscribe(const std::string& channel,
                                   void (Handler::*cb)(const ReceiveBuffer* rbuf,
                                                       const std::string& channel,
                                                       const Msg* msg),
                                   Handler* handler);

    template <class Handler>
    inline Subscription* subscribe(const std::string& channel,
                                   void (Handler::*cb)(const ReceiveBuffer* rbuf,
                                                       const std::string& channel),
                                   Handler* handler);

    template <class Msg>
    inline Subscription* subscribe(const std::string& channel,
                                   void (*cb)(const ReceiveBuffer* rbuf,
                                              const std::string& channel,
                                              const Msg* msg, void* usr),
                                   void* usr);

    #if __cplusplus > 199711L
    inline Subscription* subscribe(const std::string& channel,
                                   std::function<void (const ReceiveBuffer* rbuf,
                                                       const std::string& channel)> cb);

    template <class Msg>
    inline Subscription* subscribe(const std::string& channel,
                                   std::function<void (const ReceiveBuffer* rbuf,
                                                       const std::string& channel,
                                                       const Msg* msg)> cb);
    #endif

    inline void unsubscribe(Subscription* sub);

    virtual inline zcm_t* getUnderlyingZCM();

  protected:
    /**** Methods for inheritor override ****/
    virtual inline int publishRaw(const std::string& channel, const uint8_t* data, uint32_t len);

    // Set the value of "rawSub" with your underlying subscription. "rawSub" will be passed
    // (by reference) into unsubscribeRaw when zcm->unsubscribe() is called on a cpp subscription
    virtual inline void subscribeRaw(void*& rawSub, const std::string& channel,
                                     MsgHandler cb, void* usr);

    // Unsubscribes from a raw subscription. Effectively undoing the actions of subscribeRaw
    virtual inline void unsubscribeRaw(void*& rawSub);

  private:
    zcm_t* zcm;
    int _err;
    std::vector<Subscription*> subscriptions;
};

// TODO: why not use or inherit from the existing zcm data structures for the below

#ifndef ZCM_EMBEDDED
struct LogEvent
{
    int64_t     eventnum;
    int64_t     timestamp;
    std::string channel;
    int32_t     datalen;
    uint8_t*       data;
};

struct LogFile
{
    /**** Methods for ctor/dtor/check ****/
    inline LogFile(const std::string& path, const std::string& mode);
    inline ~LogFile();
    inline bool good() const;
    inline void close();

    /**** Methods general operations ****/
    inline int seekToTimestamp(int64_t timestamp);
    inline FILE* getFilePtr();

    /**** Methods for read/write ****/
    // NOTE: user should NOT hold-onto the returned ptr across successive calls
    inline const LogEvent* readNextEvent();
    inline const LogEvent* readPrevEvent();
    inline const LogEvent* readEventAtOffset(off_t offset);
    inline int             writeEvent(const LogEvent* event);

  private:
    inline const LogEvent* cplusplusIfyEvent(zcm_eventlog_event_t* le);
    LogEvent curEvent;
    zcm_eventlog_t* eventlog;
    zcm_eventlog_event_t* lastevent;
};
#endif

#define __zcm_cpp_impl_ok__
#include "zcm-cpp-impl.hpp"
#undef __zcm_cpp_impl_ok__

}
