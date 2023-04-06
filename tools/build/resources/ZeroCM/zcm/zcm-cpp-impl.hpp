// DO NOT EVER INCLUDE THIS HEADER FILE YOURSELF

#ifndef __zcm_cpp_impl_ok__
#error "Don't include this file"
#endif

#ifdef ZCM_EMBEDDED
#define nullptr NULL
#endif

#include <stdlib.h>

// =============== implementation ===============

// Note: To prevent compiler "redefinition" issues, all functions in this file must be declared
//       as `inline`

// New class required to allow the Handler callbacks and std::string channel names
class Subscription
{
    friend class ZCM;
    void* rawSub;

  protected:
    void* usr;
    void (*callback)(const ReceiveBuffer* rbuf, const std::string& channel, void* usr);

  public:
    virtual ~Subscription() {}

    void* getRawSub() const
    { return rawSub; }

    inline void dispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    { (*callback)(rbuf, channel, usr); }
};

static inline void SubscriptionDispatch(const ReceiveBuffer* rbuf, const char* channel, void* usr)
{ ((Subscription*)usr)->dispatch(rbuf, channel); }

#ifndef ZCM_EMBEDDED
inline ZCM::ZCM()
{
    zcm = zcm_create(nullptr);
    _err = ZCM_EOK;
}
#endif

#ifndef ZCM_EMBEDDED
inline ZCM::ZCM(const std::string& transport)
{
    zcm = zcm_create(transport.c_str());
    _err = ZCM_EOK;
}
#endif

inline ZCM::ZCM(zcm_trans_t* zt)
{
    zcm = (zcm_t*) malloc(sizeof(zcm_t));
    if (!zcm) {
        _err = ZCM_EMEMORY;
        return;
    }
    _err = zcm_init_from_trans(zcm, zt);
}

inline ZCM::~ZCM()
{
    if (zcm != nullptr) zcm_destroy(zcm);

    std::vector<Subscription*>::iterator end = subscriptions.end(),
                                          it = subscriptions.begin();
    for (;it != end; ++it) delete *it;

    zcm = nullptr;
}

inline bool ZCM::good() const
{
    return zcm != nullptr && err() == ZCM_EOK;
}

inline int ZCM::err() const
{
    return _err;
}

inline const char* ZCM::strerror() const
{
    return strerrno(_err);
}

inline const char* ZCM::strerrno(int err) const
{
    return zcm_strerrno(err);
}

#ifndef ZCM_EMBEDDED
inline void ZCM::run()
{
    zcm_run(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline void ZCM::start()
{
    zcm_start(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline void ZCM::stop()
{
    zcm_stop(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline void ZCM::pause()
{
    zcm_pause(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline void ZCM::resume()
{
    zcm_resume(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline int ZCM::handle()
{
    return zcm_handle(zcm);
}
#endif

#ifndef ZCM_EMBEDDED
inline void ZCM::setQueueSize(uint32_t sz)
{
    zcm_set_queue_size(zcm, sz);
}
#endif

#ifndef ZCM_EMBEDDED
inline int ZCM::writeTopology(const std::string& name)
{
    return zcm_write_topology(zcm, name.c_str());
}
#endif

inline int ZCM::handleNonblock()
{
    return zcm_handle_nonblock(zcm);
}

inline void ZCM::flush()
{
    zcm_flush(zcm);
}

inline int ZCM::publish(const std::string& channel, const uint8_t* data, uint32_t len)
{
    return publishRaw(channel, data, len);
}

template <class Msg>
inline int ZCM::publish(const std::string& channel, const Msg* msg)
{
    uint32_t len = msg->getEncodedSize();
    uint8_t* buf = new uint8_t[len];
    if (!buf) return ZCM_EMEMORY;
    int encodeRet = msg->encode(buf, 0, len);
    if (encodeRet < 0 || (uint32_t) encodeRet != len) {
        delete[] buf;
        return ZCM_EAGAIN;
    }
    int status = publishRaw(channel, buf, len);
    delete[] buf;
    return status;
}

inline Subscription* ZCM::subscribe(const std::string& channel,
                                    void (*cb)(const ReceiveBuffer* rbuf,
                                               const std::string& channel, void* usr),
                                    void* usr)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    Subscription* sub = new Subscription();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->usr = usr;
    sub->callback = cb;
    subscribeRaw(sub->rawSub, channel, &SubscriptionDispatch, sub);

    subscriptions.push_back(sub);
    return sub;
}

// Virtual inheritance to avoid ambiguous base class problem http://stackoverflow.com/a/139329
template<class Msg>
class TypedSubscription : public virtual Subscription
{
    friend class ZCM;

  protected:
    void (*typedCallback)(const ReceiveBuffer* rbuf, const std::string& channel, const Msg* msg,
                          void* usr);
    Msg msgMem; // Memory to decode this message into

  public:
    virtual ~TypedSubscription() {}

    inline int readMsg(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        int status = msgMem.decode(rbuf->data, 0, rbuf->data_size);
        if (status < 0) {
            #ifndef ZCM_EMBEDDED
            fprintf (stderr, "Error %d decoding %s on channel \"%s\"\n",
                             status, Msg::getTypeName(), channel.c_str());
            #endif
            return -1;
        }
        return 0;
    }

    inline void typedDispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        if (readMsg(rbuf, channel) != 0) return;
        (*typedCallback)(rbuf, channel, &msgMem, usr);
    }
};

template <class Msg>
static inline void TypedSubscriptionDispatch(const ReceiveBuffer* rbuf,
                                             const char* channel, void* usr)
{
    ((TypedSubscription<Msg>*)usr)->typedDispatch(rbuf, channel);
}

#if __cplusplus > 199711L
// Virtual inheritance to avoid ambiguous base class problem http://stackoverflow.com/a/139329
template<class Msg>
class TypedFunctionalSubscription : public virtual Subscription
{
    friend class ZCM;

  protected:
    std::function<void (const ReceiveBuffer* rbuf,
                        const std::string& channel,
                        const Msg* msg)> cb;
    Msg msgMem; // Memory to decode this message into

  public:
    virtual ~TypedFunctionalSubscription() {}

    inline int readMsg(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        int status = msgMem.decode(rbuf->data, 0, rbuf->data_size);
        if (status < 0) {
            #ifndef ZCM_EMBEDDED
            fprintf (stderr, "Error %d decoding %s on channel \"%s\"\n",
                             status, Msg::getTypeName(), channel.c_str());
            #endif
            return -1;
        }
        return 0;
    }

    inline void typedDispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        if (readMsg(rbuf, channel) != 0) return;
        cb(rbuf, channel, &msgMem);
    }
};

template <class Msg>
static inline void TypedFunctionalSubscriptionDispatch(const ReceiveBuffer* rbuf,
                                                       const char* channel, void* usr)
{
    ((TypedFunctionalSubscription<Msg>*)usr)->typedDispatch(rbuf, channel);
}

// Virtual inheritance to avoid ambiguous base class problem http://stackoverflow.com/a/139329
class FunctionalSubscription : public virtual Subscription
{
    friend class ZCM;

  protected:
    std::function<void (const ReceiveBuffer* rbuf,
                        const std::string& channel)> cb;

  public:
    virtual ~FunctionalSubscription() {}

    inline void dispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        cb(rbuf, channel);
    }
};

static inline void FunctionalSubscriptionDispatch(const ReceiveBuffer* rbuf,
                                                  const char* channel, void* usr)
{
    ((FunctionalSubscription*)usr)->dispatch(rbuf, channel);
}
#endif

// Virtual inheritance to avoid ambiguous base class problem http://stackoverflow.com/a/139329
template <class Handler>
class HandlerSubscription : public virtual Subscription
{
    friend class ZCM;

  protected:
    Handler* handler;
    void (Handler::*handlerCallback)(const ReceiveBuffer* rbuf, const std::string& channel);

  public:
    virtual ~HandlerSubscription() {}

    inline void handlerDispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        (handler->*handlerCallback)(rbuf, channel);
    }
};

template <class Handler>
static inline void HandlerSubscriptionDispatch(const ReceiveBuffer* rbuf,
                                               const char* channel, void* usr)
{
    ((HandlerSubscription<Handler>*)usr)->handlerDispatch(rbuf, channel);
}

template <class Msg, class Handler>
class TypedHandlerSubscription : public TypedSubscription<Msg>, HandlerSubscription<Handler>
{
    friend class ZCM;

  protected:
    void (Handler::*typedHandlerCallback)(const ReceiveBuffer* rbuf, const std::string& channel,
                                          const Msg* msg);

  public:
    virtual ~TypedHandlerSubscription() {}

    inline void typedHandlerDispatch(const ReceiveBuffer* rbuf, const std::string& channel)
    {
        // Unfortunately, we need to add "this" here to handle template inheritance:
        // https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members
        if (this->readMsg(rbuf, channel) != 0) return;
        (this->handler->*typedHandlerCallback)(rbuf, channel, &this->msgMem);
    }
};

template <class Msg, class Handler>
static inline void TypedHandlerSubscriptionDispatch(const ReceiveBuffer* rbuf,
                                                    const char* channel, void* usr)
{
    ((TypedHandlerSubscription<Msg, Handler>*)usr)->typedHandlerDispatch(rbuf, channel);
}

// TODO: lots of room to condense the implementations of the various subscribe functions
template <class Msg, class Handler>
inline Subscription* ZCM::subscribe(const std::string& channel,
                                    void (Handler::*cb)(const ReceiveBuffer* rbuf,
                                                        const std::string& channel,
                                                        const Msg* msg),
                                    Handler* handler)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    TypedHandlerSubscription<Msg, Handler>* sub = new TypedHandlerSubscription<Msg, Handler>();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->handler = handler;
    sub->typedHandlerCallback = cb;
    subscribeRaw(sub->rawSub, channel, &TypedHandlerSubscriptionDispatch<Msg, Handler>, sub);

    subscriptions.push_back(sub);
    return sub;
}

template <class Handler>
inline Subscription* ZCM::subscribe(const std::string& channel,
                                    void (Handler::*cb)(const ReceiveBuffer* rbuf,
                                                        const std::string& channel),
                                    Handler* handler)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    HandlerSubscription<Handler>* sub = new HandlerSubscription<Handler>();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->handler = handler;
    sub->handlerCallback = cb;
    subscribeRaw(sub->rawSub, channel, &HandlerSubscriptionDispatch<Handler>, sub);

    subscriptions.push_back(sub);
    return sub;
}

template <class Msg>
inline Subscription* ZCM::subscribe(const std::string& channel,
                                    void (*cb)(const ReceiveBuffer* rbuf,
                                               const std::string& channel,
                                               const Msg* msg, void* usr),
                                    void* usr)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    TypedSubscription<Msg>* sub = new TypedSubscription<Msg>();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->usr = usr;
    sub->typedCallback = cb;
    subscribeRaw(sub->rawSub, channel, &TypedSubscriptionDispatch<Msg>, sub);

    subscriptions.push_back(sub);
    return sub;
}

#if __cplusplus > 199711L
template <class Msg>
inline Subscription* ZCM::subscribe(const std::string& channel,
                                    std::function<void (const ReceiveBuffer* rbuf,
                                                        const std::string& channel,
                                                        const Msg* msg)> cb)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    TypedFunctionalSubscription<Msg>* sub = new TypedFunctionalSubscription<Msg>();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->usr = nullptr;
    sub->cb = cb;
    subscribeRaw(sub->rawSub, channel, &TypedFunctionalSubscriptionDispatch<Msg>, sub);

    subscriptions.push_back(sub);
    return sub;
}

inline Subscription* ZCM::subscribe(const std::string& channel,
                                    std::function<void (const ReceiveBuffer* rbuf,
                                                        const std::string& channel)> cb)
{
    if (!zcm) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr, "ZCM instance not initialized. Ignoring call to subscribe()\n");
        #endif
        _err = ZCM_EINVALID;
        return nullptr;
    }

    FunctionalSubscription* sub = new FunctionalSubscription();
    if (!sub) {
        _err = ZCM_EMEMORY;
        return nullptr;
    }
    sub->usr = nullptr;
    sub->cb = cb;
    subscribeRaw(sub->rawSub, channel, &FunctionalSubscriptionDispatch, sub);

    subscriptions.push_back(sub);
    return sub;
}
#endif

inline void ZCM::unsubscribe(Subscription* sub)
{
    std::vector<Subscription*>::iterator end = subscriptions.end(),
                                          it = subscriptions.begin();
    for (; it != end; ++it) {
        if (*it == sub) {
            unsubscribeRaw(sub->rawSub);
            subscriptions.erase(it);
            delete sub;
            break;
        }
    }
}

inline zcm_t* ZCM::getUnderlyingZCM()
{ return zcm; }

inline int ZCM::publishRaw(const std::string& channel, const uint8_t* data, uint32_t len)
{ return zcm_publish(zcm, channel.c_str(), data, len); }

inline void ZCM::subscribeRaw(void*& rawSub, const std::string& channel,
                              MsgHandler cb, void* usr)
{ rawSub = zcm_subscribe(zcm, channel.c_str(), cb, usr); }

inline void ZCM::unsubscribeRaw(void*& rawSub)
{ zcm_unsubscribe(zcm, (zcm_sub_t*) rawSub); rawSub = nullptr; }


// ***********************************
// LogFile and LogEvent implementation
// ***********************************
#ifndef ZCM_EMBEDDED
inline LogFile::LogFile(const std::string& path, const std::string& mode)
{
    this->eventlog = zcm_eventlog_create(path.c_str(), mode.c_str());
    this->lastevent = nullptr;
}

inline void LogFile::close()
{
    if (eventlog)
        zcm_eventlog_destroy(eventlog);
    eventlog = nullptr;
    if(lastevent)
        zcm_eventlog_free_event(lastevent);
    lastevent = nullptr;
}

inline LogFile::~LogFile()
{
    close();
}

inline bool LogFile::good() const
{
    return eventlog != nullptr;
}

inline int LogFile::seekToTimestamp(int64_t timestamp)
{
    return zcm_eventlog_seek_to_timestamp(eventlog, timestamp);
}

inline FILE* LogFile::getFilePtr()
{
    return zcm_eventlog_get_fileptr(eventlog);
}

inline const LogEvent* LogFile::cplusplusIfyEvent(zcm_eventlog_event_t* evt)
{
    if (lastevent)
        zcm_eventlog_free_event(lastevent);
    lastevent = evt;
    if (!evt) return nullptr;
    curEvent.eventnum = evt->eventnum;
    curEvent.channel.assign(evt->channel, evt->channellen);
    curEvent.timestamp = evt->timestamp;
    curEvent.datalen = evt->datalen;
    curEvent.data = evt->data;
    return &curEvent;
}

inline const LogEvent* LogFile::readNextEvent()
{
    zcm_eventlog_event_t* evt = zcm_eventlog_read_next_event(eventlog);
    return cplusplusIfyEvent(evt);
}

inline const LogEvent* LogFile::readPrevEvent()
{
    zcm_eventlog_event_t* evt = zcm_eventlog_read_prev_event(eventlog);
    return cplusplusIfyEvent(evt);
}

inline const LogEvent* LogFile::readEventAtOffset(off_t offset)
{
    zcm_eventlog_event_t* evt = zcm_eventlog_read_event_at_offset(eventlog, offset);
    return cplusplusIfyEvent(evt);
}

inline int LogFile::writeEvent(const LogEvent* event)
{
    zcm_eventlog_event_t evt;
    evt.eventnum = event->eventnum;
    evt.timestamp = event->timestamp;
    evt.channellen = event->channel.size();
    evt.datalen = event->datalen;
    // casting away constness okay because evt isn't used past the end of this function
    evt.channel = (char*) event->channel.c_str();
    evt.data = event->data;
    return zcm_eventlog_write_event(eventlog, &evt);
}
#endif
