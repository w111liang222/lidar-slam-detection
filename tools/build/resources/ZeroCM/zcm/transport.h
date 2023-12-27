#ifndef _ZCM_TRANS_H
#define _ZCM_TRANS_H

/*******************************************************************************
 * ZCM Transport Abstract Interface
 *
 *     This file provides an abstract interface for implementing ZCM on top of a
 *     variety of different transport layers. ZCM attempts to be transport
 *     agnostic by its core design, striving to only require a message type and
 *     marshalling system. By this design, we allow end-users to decide amoungst
 *     the myriad of transport layer trade-offs. For some, reliability is most
 *     critical. For others, performance dominates as a constraint. For others,
 *     ease-of-use across various platforms is important (e.g. embedded, server,
 *     and web). Whatever the need, we indeed to provide the support.
 *
 *     The protocol is designed around the notion of messages. ZCM delivers full
 *     messages to the underlying transport. The underlying transport is
 *     responsible for providing a Maximum Transmission Unit (MTU) to ZCM. Any
 *     ZCM messages larger than the MTU will be *droppped* by ZCM. All messages
 *     sized within the MTU will be sent to the transport layer. However, this
 *     does *not* guarentee reliable delivery. After submitting a message to the
 *     transport, ZCM is no longer responsible for the message. Thus, reliablity
 *     guarentees are the domain of each individual transport layer. This design
 *     allows for a LOT of flexibility. Buffer over-run and congestion are
 *     handled in a similair way. The transport layer is responsible for these
 *     concerns.
 *
 *     This API is also designed with the following use-cases in mind:
 *         - blocking and non-blocking
 *         - minimal buffer copying
 *         - single and multi-threaded
 *
 *     IMPORTANT: Although there is only a single zcm_trans_t* type, it actually
 *     represents two different semantic modes (blocking and non-blocking).
 *     An implementation must up-front decide and commit-to one of the two types
 *     This distinction is specified by setting the zcm_trans_t 'trans_type'
 *     field to either ZCM_BLOCKING or ZCM_NONBLOCKING
 *
 *******************************************************************************
 * Blocking Transport API (ZCM_TRANS_BLOCK):
 *
 *      zcm_trans_t* create(zcm_url_t* url)
 *      --------------------------------------------------------------------
 *         This method cannot be called from a vtbl, but this documentation
 *         exists to make all create() methods more uniform. This method
 *         should construct a concrete zcm_trans_t, set it's vtbl and either
 *         upcast to zcm_trans_t* or (in C++) let it convert automatically.
 *         Internally, the vtbl field should be set to the appropriate
 *         table of function pointers.
 *
 *      size_t get_mtu(zcm_trans_t* zt)
 *      --------------------------------------------------------------------
 *         Returns the Maximum Transmission Unit supported by this transport
 *         The transport is allowed to ignore any message above this size
 *         Users of this transport should ensure that they never attempt to
 *         send messages larger than the MTU of their chosen transport
 *
 *      int sendmsg(zcm_trans_t* zt, zcm_msg_t msg)
 *      --------------------------------------------------------------------
 *         The caller to this method initiates a message send operation. The
 *         caller must populate the fields of the zcm_msg_t. The channel must
 *         be less than ZCM_CHANNEL_MAXLEN and 'len <= get_mtu()', otherwise
 *         this method can return ZCM_EINVALID. On receipt of valid params,
 *         this method should block until the message has been successfully
 *         sent and should return ZCM_EOK.
 *
 *      int recvmsg_enable(zcm_trans_t* zt, const char* channel, bool enable)
 *      --------------------------------------------------------------------
 *         This method will enable/disable the receipt of messages on the particular
 *         channel. This method is like a "suggestion", the transport is allowed to
 *         "enable" more channels without concern. This method only sets the
 *         "minimum set" of channels that the user expects to receive. It exists
 *         to provide the transport layer more information for optimization purposes
 *         (e.g. the transport may decide to send each channel over a different endpoint).
 *         NOTE: This method should work concurrently and correctly with
 *         recvmsg(). On success, this method should return ZCM_EOK
 *
 *      int recvmsg(zcm_trans_t* zt, zcm_msg_t* msg, int timeout)
 *      --------------------------------------------------------------------
 *         The caller to this method initiates a message recv operation. This
 *         methods blocks until it receives a message. It should return ZCM_EOK.
 *         Messages that have been 'enable'd with recvmsg_enable() *must* be received.
 *         Extra messages can also be received; the 'enable'd channels sets a minimum set.
 *         NOTE: This method should work concurrently and correctly with
 *         recvmsg_enable(). If 'timeout >= 0' then recvmsg()
 *         should return EAGAIN if it is unable to receive a message within
 *         'timeout' milliseconds.
 *         NOTE: We do *NOT* require a very accurate clock for this timeout feature
 *         and users should only expect accuracy within a few milliseconds. Users
 *         should *not* attempt to use this timing mechanism for real-time events.
 *
 *      int update(zcm_trans_t* zt);
 *      --------------------------------------------------------------------
 *         This method is unused (in this mode) and should not be called by the user.
 *         An implementation is allowed to set this field to NULL.
 *
 *      void destroy(zcm_trans_t* zt)
 *      --------------------------------------------------------------------
 *         Close the transport and cleanup any resources used.
 *
 *******************************************************************************
 * Non-Blocking Transport API:
 *
 *      General Note: None of the methods on this object must be thread-safe.
 *                    This API is designed for single-thread, non-blocking,
 *                    and minimalist transports (such as those found in embedded).
 *
 *      zcm_trans_t* create(zcm_url_t* url)
 *      --------------------------------------------------------------------
 *         This method cannot be called from a vtbl, but this documentation
 *         exists to make all create() methods more uniform. This method
 *         should construct a concrete zcm_trans_t, set it's vtbl and either
 *         upcast to zcm_trans_t* or (in C++) let it convert automatically.
 *         Internally, the vtbl field should be set to the appropriate
 *         table of function pointers.
 *
 *      size_t get_mtu(zcm_trans_t* zt)
 *      --------------------------------------------------------------------
 *         Returns the Maximum Transmission Unit supported by this transport
 *         The transport is allowed to ignore any message above this size
 *         Users of this transport should ensure that they never attempt to
 *         send messages larger than the MTU of their chosen transport
 *
 *      int sendmsg(zcm_trans_t* zt, zcm_msg_t msg)
 *      --------------------------------------------------------------------
 *         The caller to this method initiates a message send operation. The
 *         caller must populate the fields of the zcm_msg_t. The channel must
 *         be less than ZCM_CHANNEL_MAXLEN and 'len <= get_mtu()', otherwise
 *         this method can return ZCM_EINVALID. On receipt of valid params,
 *         this method should *never block*. If the transport cannot accept the
 *         message due to unavailability, ZCM_EAGAIN should be returned.
 *         On success ZCM_EOK should be returned.
 *
 *      int recvmsg_enable(zcm_trans_t* zt, const char* channel, bool enable)
 *      --------------------------------------------------------------------
 *         This method will enable/disable the receipt of messages on the particular
 *         channel. This method is like a "suggestion", the transport is allowed to
 *         "enable" more channels without concern. This method only sets the
 *         "minimum set" of channels that the user expects to receive. It exists to
 *         provide the transport layer more information for optimization purposes
 *         (e.g. the transport may decide to send each channel over a different endpoint).
 *         NOTE: This method does NOT have to work concurrently with recvmsg().
 *         On success, this method should return ZCM_EOK
 *
 *      int recvmsg(zcm_trans_t* zt, zcm_msg_t* msg, int timeout)
 *      --------------------------------------------------------------------
 *         The caller to this method initiates a message recv operation. This
 *         methods should *never block*. If a message has been received then
 *         EOK should be returned, otherwise it should return EAGAIN.
 *         Messages that have been 'enable'd with recvmsg_enable() *must* be received.
 *         Extra messages can also be received; the 'enable'd channels sets a minimum set.
 *         NOTE: This method does NOT have to work concurrently with recvmsg_enable()
 *         NOTE: The 'timeout' field is ignored
 *
 *      int update(zcm_trans_t* zt)
 *      --------------------------------------------------------------------
 *         This method is called from the zcm_handle_nonblock() function.
 *         This method provides a periodicly-running routine that can perform
 *         updates to the underlying hardware or other general mantainence to
 *         this transport. This method should *never block*. Again, this
 *         method is called from zcm_handle_nonblock() and thus runs at the same
 *         frequency as zcm_handle_nonblock(). Failure to call zcm_handle_nonblock()
 *         while using an nonblock transport may cause the transport to work
 *         incorrectly on both message send and recv.
 *
 *      void destroy(zcm_trans_t* zt)
 *      --------------------------------------------------------------------
 *         Close the transport and cleanup any resources used.
 *
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "zcm/zcm.h"

/* Only define inline for C99 builds or better */
#if (__STDC_VERSION__ >= 199901L) || (__cplusplus)
# define INLINE inline
#else
# define INLINE
#endif

typedef struct zcm_msg_t zcm_msg_t;
typedef struct zcm_trans_methods_t zcm_trans_methods_t;


/* TODO: Discuss the semantics of this datastruct depending on the context (send vs. recv) */
/* TODO: do we really need another structure for this (zcm_recv_buf_t is almost identiical) */
struct zcm_msg_t
{
    uint64_t utime;  /* 0 means invalid (caller should compute its own utime) */
    const char* channel;
    size_t len;
    uint8_t* buf;
};

struct zcm_trans_t
{
    enum zcm_type trans_type;
    zcm_trans_methods_t* vtbl;
};

struct zcm_trans_methods_t
{
    size_t  (*get_mtu)(zcm_trans_t* zt);
    int     (*sendmsg)(zcm_trans_t* zt, zcm_msg_t msg);
    int     (*recvmsg_enable)(zcm_trans_t* zt, const char* channel, bool enable);
    int     (*recvmsg)(zcm_trans_t* zt, zcm_msg_t* msg, int timeout);
    int     (*update)(zcm_trans_t* zt);
    void    (*destroy)(zcm_trans_t* zt);
};

/* Helper functions to make the VTbl dispatch cleaner */
static INLINE size_t zcm_trans_get_mtu(zcm_trans_t* zt)
{ return zt->vtbl->get_mtu(zt); }

static INLINE int zcm_trans_sendmsg(zcm_trans_t* zt, zcm_msg_t msg)
{ return zt->vtbl->sendmsg(zt, msg); }

static INLINE int zcm_trans_recvmsg_enable(zcm_trans_t* zt, const char* channel, bool enable)
{ return zt->vtbl->recvmsg_enable(zt, channel, enable); }

static INLINE int zcm_trans_recvmsg(zcm_trans_t* zt, zcm_msg_t* msg, int timeout)
{ return zt->vtbl->recvmsg(zt, msg, timeout); }

static INLINE int zcm_trans_update(zcm_trans_t* zt)
{ return zt->vtbl->update(zt); }

static INLINE void zcm_trans_destroy(zcm_trans_t* zt)
{ return zt->vtbl->destroy(zt); }

#ifdef __cplusplus
}
#endif

#endif /* _ZCM_TRANS_H */
