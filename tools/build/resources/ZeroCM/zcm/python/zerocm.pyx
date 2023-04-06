# cython: language_level=2

from libc.stdint cimport int64_t, int32_t, uint32_t, uint8_t
from posix.unistd cimport off_t
from inspect import signature
import time

cdef extern from "zcm/python/zcm-python.h":
    void PyEval_InitThreads_CUSTOM()

cdef extern from "zcm/zcm.h":
    cpdef enum zcm_return_codes:
        ZCM_EOK,
        ZCM_EINVALID,
        ZCM_EAGAIN,
        ZCM_ECONNECT,
        ZCM_EINTR,
        ZCM_EUNKNOWN,
        ZCM_NUM_RETURN_CODES
    ctypedef struct zcm_t:
        pass
    ctypedef struct zcm_sub_t:
        pass
    ctypedef struct zcm_recv_buf_t:
        int64_t recv_utime
        uint8_t* data
        uint32_t data_size
        pass
    ctypedef void (*zcm_msg_handler_t)(const zcm_recv_buf_t* rbuf, const char* channel, void* usr)

    zcm_t* zcm_create (const char* url)
    void   zcm_destroy(zcm_t* zcm)

    const char* zcm_strerrno(int err)

    zcm_sub_t* zcm_try_subscribe  (zcm_t* zcm, const char* channel, zcm_msg_handler_t cb, void* usr)
    int        zcm_try_unsubscribe(zcm_t* zcm, zcm_sub_t* sub)

    int  zcm_publish(zcm_t* zcm, const char* channel, const uint8_t* data, uint32_t dlen)

    int  zcm_try_flush         (zcm_t* zcm)

    void zcm_run               (zcm_t* zcm)
    void zcm_start             (zcm_t* zcm)
    int  zcm_try_stop          (zcm_t* zcm)
    void zcm_pause             (zcm_t* zcm)
    void zcm_resume            (zcm_t* zcm)
    int  zcm_handle            (zcm_t* zcm)
    int  zcm_try_set_queue_size(zcm_t* zcm, uint32_t numMsgs)
    int  zcm_write_topology    (zcm_t* zcm, const char* name)

    int  zcm_handle_nonblock(zcm_t* zcm)

    ctypedef struct zcm_eventlog_t:
        pass
    ctypedef struct zcm_eventlog_event_t:
        int64_t  eventnum
        int64_t  timestamp
        int32_t  channellen
        int32_t  datalen
        char*    channel
        uint8_t* data

    zcm_eventlog_t* zcm_eventlog_create(const char* path, const char* mode)
    void            zcm_eventlog_destroy(zcm_eventlog_t* eventlog)

    int zcm_eventlog_seek_to_timestamp(zcm_eventlog_t* eventlog, int64_t ts)

    zcm_eventlog_event_t* zcm_eventlog_read_next_event(zcm_eventlog_t* eventlog)
    zcm_eventlog_event_t* zcm_eventlog_read_prev_event(zcm_eventlog_t* eventlog)
    zcm_eventlog_event_t* zcm_eventlog_read_event_at_offset(zcm_eventlog_t* eventlog, off_t offset)
    void                  zcm_eventlog_free_event(zcm_eventlog_event_t* event)
    int                   zcm_eventlog_write_event(zcm_eventlog_t* eventlog, \
                                                   const zcm_eventlog_event_t* event)

cdef class ZCMSubscription:
    cdef zcm_sub_t* sub
    cdef object handler
    cdef object msgtype

cdef void handler_cb(const zcm_recv_buf_t* rbuf, const char* channel, void* usr) with gil:
    subs = (<ZCMSubscription>usr)
    msg = subs.msgtype.decode(rbuf.data[:rbuf.data_size])
    subs.handler(channel.decode('utf-8'), msg, rbuf.recv_utime)

cdef void handler_cb_raw(const zcm_recv_buf_t* rbuf, const char* channel, void* usr) with gil:
    subs = (<ZCMSubscription>usr)
    subs.handler(channel.decode('utf-8'), rbuf.data[:rbuf.data_size], rbuf.recv_utime)

cdef void handler_cb_deprecated(const zcm_recv_buf_t* rbuf, const char* channel, void* usr) with gil:
    subs = (<ZCMSubscription>usr)
    msg = subs.msgtype.decode(rbuf.data[:rbuf.data_size])
    subs.handler(channel.decode('utf-8'), msg)

cdef void handler_cb_raw_deprecated(const zcm_recv_buf_t* rbuf, const char* channel, void* usr) with gil:
    subs = (<ZCMSubscription>usr)
    subs.handler(channel.decode('utf-8'), rbuf.data[:rbuf.data_size])

cdef class ZCM:
    cdef zcm_t* zcm
    cdef object subscriptions
    def __cinit__(self, str url=""):
        PyEval_InitThreads_CUSTOM()
        self.subscriptions = []
        self.zcm = zcm_create(url.encode('utf-8'))
    def __dealloc__(self):
        if self.zcm == NULL:
            return
        self.stop()
        while len(self.subscriptions) > 0:
            self.unsubscribe(self.subscriptions[0]);
        zcm_destroy(self.zcm)
    def good(self):
        return self.zcm != NULL
    def strerrno(self, err):
        return zcm_strerrno(err).decode('utf-8')
    def subscribe_raw(self, str channel, handler):
        cdef ZCMSubscription subs = ZCMSubscription()
        subs.handler = handler
        subs.msgtype = None
        sig = signature(handler)
        selected_handler_cb = handler_cb_raw_deprecated if len(sig.parameters) == 2 else handler_cb_raw
        while True:
            subs.sub = zcm_try_subscribe(self.zcm, channel.encode('utf-8'), selected_handler_cb, <void*> subs)
            if subs.sub != NULL:
                self.subscriptions.append(subs)
                return subs
            time.sleep(0) # yield the gil
    def subscribe(self, str channel, msgtype, handler):
        cdef ZCMSubscription subs = ZCMSubscription()
        subs.handler = handler
        subs.msgtype = msgtype
        sig = signature(handler)
        selected_handler_cb = handler_cb_deprecated if len(sig.parameters) == 2 else handler_cb
        while True:
            subs.sub = zcm_try_subscribe(self.zcm, channel.encode('utf-8'), selected_handler_cb, <void*> subs)
            if subs.sub != NULL:
                self.subscriptions.append(subs)
                return subs
            time.sleep(0) # yield the gil
    def unsubscribe(self, ZCMSubscription subs):
        while zcm_try_unsubscribe(self.zcm, subs.sub) != ZCM_EOK:
            time.sleep(0) # yield the gil
        self.subscriptions.remove(subs)
    def publish(self, str channel, object msg):
        _data = msg.encode()
        cdef const uint8_t* data = _data
        return zcm_publish(self.zcm, channel.encode('utf-8'), data, len(_data) * sizeof(uint8_t))
    def publish_raw(self, str channel, bytes data):
        cdef const uint8_t* _data = data
        return zcm_publish(self.zcm, channel.encode('utf-8'), _data, len(data) * sizeof(uint8_t))
    def flush(self):
        while zcm_try_flush(self.zcm) != ZCM_EOK:
            time.sleep(0) # yield the gil
    def run(self):
        zcm_run(self.zcm)
    def start(self):
        zcm_start(self.zcm)
    def stop(self):
        while zcm_try_stop(self.zcm) != ZCM_EOK:
            time.sleep(0) # yield the gil
    def pause(self):
        zcm_pause(self.zcm)
    def resume(self):
        zcm_resume(self.zcm)
    def handle(self):
        return zcm_handle(self.zcm)
    def setQueueSize(self, numMsgs):
        while zcm_try_set_queue_size(self.zcm, numMsgs) != ZCM_EOK:
            time.sleep(0) # yield the gil
    def writeTopology(self, str name):
        return zcm_write_topology(self.zcm, name.encode('utf-8'))
    def handleNonblock(self):
        return zcm_handle_nonblock(self.zcm)

cdef class LogEvent:
    cdef int64_t eventnum
    cdef int64_t timestamp
    cdef object  channel
    cdef object  data
    def __cinit__(self):
        pass
    def setEventnum(self, int64_t eventnum):
        self.eventnum = eventnum
    def getEventnum(self):
        return self.eventnum
    def setTimestamp(self, int64_t time):
        self.timestamp = time
    def getTimestamp(self):
        return self.timestamp
    def setChannel(self, str chan):
        self.channel = chan.encode('utf-8')
    def getChannel(self):
        return self.channel.decode('utf-8')
    def setData(self, bytes data):
        self.data = data
    def getData(self):
        return self.data

cdef class LogFile:
    cdef zcm_eventlog_t* eventlog
    cdef zcm_eventlog_event_t* lastevent
    def __cinit__(self, str path, str mode):
        self.eventlog = zcm_eventlog_create(path.encode('utf-8'), mode.encode('utf-8'))
        self.lastevent = NULL
    def __dealloc__(self):
        self.close()
    def close(self):
        if self.eventlog != NULL:
            zcm_eventlog_destroy(self.eventlog)
            self.eventlog = NULL
        if self.lastevent != NULL:
            zcm_eventlog_free_event(self.lastevent)
            self.lastevent = NULL
    def good(self):
        return self.eventlog != NULL
    def seekToTimestamp(self, int64_t timestamp):
        return zcm_eventlog_seek_to_timestamp(self.eventlog, timestamp)
    cdef __setCurrentEvent(self, zcm_eventlog_event_t* evt):
        if self.lastevent != NULL:
            zcm_eventlog_free_event(self.lastevent)
        self.lastevent = evt
        if evt == NULL:
            return None
        cdef LogEvent curEvent = LogEvent()
        curEvent.eventnum = evt.eventnum
        curEvent.setChannel   (evt.channel[:evt.channellen].decode('utf-8'))
        curEvent.setTimestamp (evt.timestamp)
        curEvent.setData      ((<uint8_t*>evt.data)[:evt.datalen])
        return curEvent
    def readNextEvent(self):
        cdef zcm_eventlog_event_t* evt = zcm_eventlog_read_next_event(self.eventlog)
        return self.__setCurrentEvent(evt)
    def readPrevEvent(self):
        cdef zcm_eventlog_event_t* evt = zcm_eventlog_read_prev_event(self.eventlog)
        return self.__setCurrentEvent(evt)
    def readEventOffset(self, off_t offset):
        cdef zcm_eventlog_event_t* evt = zcm_eventlog_read_event_at_offset(self.eventlog, offset)
        return self.__setCurrentEvent(evt)
    def writeEvent(self, LogEvent event):
        cdef zcm_eventlog_event_t evt
        evt.eventnum   = event.eventnum
        evt.timestamp  = event.timestamp
        evt.channellen = len(event.channel)
        evt.datalen    = len(event.data)
        evt.channel    = <char*> event.channel
        evt.data       = <uint8_t*> event.data
        return zcm_eventlog_write_event(self.eventlog, &evt);
