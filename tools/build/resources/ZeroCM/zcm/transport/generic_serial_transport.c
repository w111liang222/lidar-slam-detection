#include "zcm/zcm.h"
#include "zcm/transport.h"
#include "generic_serial_transport.h"
#include "generic_serial_circ_buff.h"
#include "generic_serial_fletcher.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef ZCM_GENERIC_SERIAL_ESCAPE_CHAR
#define ZCM_GENERIC_SERIAL_ESCAPE_CHAR (0xcc)
#endif

#define ASSERT(x)

// Framing (size = 9 + chan_len + data_len)
//   0xCC
//   0x00
//   chan_len
//   data_len  (4 bytes)
//   *chan
//   *data
//   sum1(*chan, *data)
//   sum2(*chan, *data)
#define FRAME_BYTES 9

typedef struct zcm_trans_generic_serial_t zcm_trans_generic_serial_t;
struct zcm_trans_generic_serial_t
{
    zcm_trans_t trans; // This must be first to preserve pointer casting

    circBuffer_t sendBuffer;
    circBuffer_t recvBuffer;
    uint8_t      recvChanName[ZCM_CHANNEL_MAXLEN + 1];
    size_t       mtu;
    uint8_t*     recvMsgData;

    size_t (*get)(uint8_t* data, size_t nData, void* usr);
    size_t (*put)(const uint8_t* data, size_t nData, void* usr);
    void* put_get_usr;

    uint64_t (*time)(void* usr);
    void* time_usr;
};

static zcm_trans_generic_serial_t *cast(zcm_trans_t *zt);

size_t serial_get_mtu(zcm_trans_generic_serial_t *zt)
{ return zt->mtu; }

int serial_sendmsg(zcm_trans_generic_serial_t *zt, zcm_msg_t msg)
{
    size_t chan_len = strlen(msg.channel);
    size_t nPushed = 0;

    if (chan_len > ZCM_CHANNEL_MAXLEN)                               return ZCM_EINVALID;
    if (msg.len > zt->mtu)                                           return ZCM_EINVALID;
    if (FRAME_BYTES + chan_len + msg.len > cb_room(&zt->sendBuffer)) return ZCM_EAGAIN;

    cb_push_back(&zt->sendBuffer, ZCM_GENERIC_SERIAL_ESCAPE_CHAR); ++nPushed;
    cb_push_back(&zt->sendBuffer, 0x00);                           ++nPushed;
    cb_push_back(&zt->sendBuffer, chan_len);                       ++nPushed;

    uint32_t len = (uint32_t)msg.len;
    cb_push_back(&zt->sendBuffer, (len>>24)&0xff); ++nPushed;
    cb_push_back(&zt->sendBuffer, (len>>16)&0xff); ++nPushed;
    cb_push_back(&zt->sendBuffer, (len>> 8)&0xff); ++nPushed;
    cb_push_back(&zt->sendBuffer, (len>> 0)&0xff); ++nPushed;

    uint16_t checksum = 0xffff;
    size_t i;
    for (i = 0; i < chan_len; ++i) {
        uint8_t c = (uint8_t) msg.channel[i];

        cb_push_back(&zt->sendBuffer, c); ++nPushed;

        if (c == ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
        	// the escape character doesn't count, so we have chan_len - i characters
        	// remaining in channel + the msg + the checksum.
            if (cb_room(&zt->sendBuffer) > chan_len - i + msg.len + 1) {
                cb_push_back(&zt->sendBuffer, c); ++nPushed;
            } else {
                cb_pop_back(&zt->sendBuffer, nPushed);
                return ZCM_EAGAIN;
            }
        }

        checksum = fletcherUpdate(c, checksum);
    }

    for (i = 0; i < msg.len; ++i) {
        uint8_t c = (uint8_t) msg.buf[i];

        cb_push_back(&zt->sendBuffer, c); ++nPushed;

        if (c == ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
        	// the escape character doesn't count, so we have msg.len - i characters
        	// remaining in the msg + the checksum.
            if (cb_room(&zt->sendBuffer) > msg.len - i + 1) {
                cb_push_back(&zt->sendBuffer, c); ++nPushed;
            } else {
                cb_pop_back(&zt->sendBuffer, nPushed);
                return ZCM_EAGAIN;
            }
        }

        checksum = fletcherUpdate(c, checksum);
    }

    cb_push_back(&zt->sendBuffer, (checksum >> 8) & 0xff); ++nPushed;
    cb_push_back(&zt->sendBuffer,  checksum       & 0xff); ++nPushed;

    return ZCM_EOK;
}

int serial_recvmsg_enable(zcm_trans_generic_serial_t *zt, const char *channel, bool enable)
{
    // NOTE: not implemented because it is unlikely that a microprocessor is
    //       going to be hearing messages on a USB comms that it doesn't want
    //       to hear
    return ZCM_EOK;
}

int serial_recvmsg(zcm_trans_generic_serial_t *zt, zcm_msg_t *msg, int timeout)
{
    uint64_t utime = zt->time(zt->time_usr);
    size_t incomingSize = cb_size(&zt->recvBuffer);
    if (incomingSize < FRAME_BYTES)
        return ZCM_EAGAIN;

    size_t consumed = 0;
    uint8_t chan_len = 0;
    uint16_t checksum = 0;
    uint8_t expectedHighCS = 0;
    uint8_t expectedLowCS  = 0;
    uint16_t receivedCS = 0;

    // Sync
    if (cb_front(&zt->recvBuffer, consumed++) != ZCM_GENERIC_SERIAL_ESCAPE_CHAR) goto fail;
    if (cb_front(&zt->recvBuffer, consumed++) != 0x00)                           goto fail;

    // Msg sizes
    chan_len  = cb_front(&zt->recvBuffer, consumed++);
    msg->len  = ((uint32_t) cb_front(&zt->recvBuffer, consumed++)) << 24;
    msg->len |= ((uint32_t) cb_front(&zt->recvBuffer, consumed++)) << 16;
    msg->len |= ((uint32_t) cb_front(&zt->recvBuffer, consumed++)) << 8;
    msg->len |= cb_front(&zt->recvBuffer, consumed++);

    if (chan_len > ZCM_CHANNEL_MAXLEN)     goto fail;
    if (msg->len > zt->mtu)                goto fail;

    if (incomingSize < FRAME_BYTES + chan_len + msg->len) return ZCM_EAGAIN;

    memset(&zt->recvChanName, '\0', ZCM_CHANNEL_MAXLEN);

    checksum = 0xffff;
    int i;
    for (i = 0; i < chan_len; ++i) {

        uint8_t c = cb_front(&zt->recvBuffer, consumed++);

        if (c == ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
        	// the escape character doesn't count, so we have chan_len - i characters
        	// remaining in channel + the msg + the checksum.
            if (consumed + chan_len - i + msg->len + 2 > incomingSize) return ZCM_EAGAIN;

            c = cb_front(&zt->recvBuffer, consumed++);

            if (c != ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
                consumed-=2;
                goto fail;
            }
        }

        zt->recvChanName[i] = c;
        checksum = fletcherUpdate(c, checksum);
    }

    zt->recvChanName[chan_len] = '\0';

    for (i = 0; i < msg->len; ++i) {
        if (consumed > incomingSize) return ZCM_EAGAIN;

        uint8_t c = cb_front(&zt->recvBuffer, consumed++);

        if (c == ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
        	// the escape character doesn't count, so we have msg.len - i characters
        	// remaining in the msg + the checksum.
            if (consumed + msg->len - i + 2 > incomingSize) return ZCM_EAGAIN;

            c = cb_front(&zt->recvBuffer, consumed++);

            if (c != ZCM_GENERIC_SERIAL_ESCAPE_CHAR) {
                consumed-=2;
                goto fail;
            }
        }

        zt->recvMsgData[i] = c;
        checksum = fletcherUpdate(c, checksum);
    }

    expectedHighCS = cb_front(&zt->recvBuffer, consumed++);
    expectedLowCS  = cb_front(&zt->recvBuffer, consumed++);
    receivedCS = (expectedHighCS << 8) | expectedLowCS;
    if (receivedCS == checksum) {
        msg->channel = (char*) zt->recvChanName;
        msg->buf     = zt->recvMsgData;
        msg->utime   = utime;
        cb_pop_front(&zt->recvBuffer, consumed);
        return ZCM_EOK;
    }

  fail:
    cb_pop_front(&zt->recvBuffer, consumed);
    // Note: because this is a nonblocking transport, timeout is ignored, so we don't need
    //       to subtract the time used here
    return serial_recvmsg(zt, msg, timeout);
}

int serial_update_rx(zcm_trans_t *_zt)
{
    zcm_trans_generic_serial_t* zt = cast(_zt);
    cb_flush_in(&zt->recvBuffer, zt->get, zt->put_get_usr);
    return ZCM_EOK;
}

int serial_update_tx(zcm_trans_t *_zt)
{
    zcm_trans_generic_serial_t* zt = cast(_zt);
    cb_flush_out(&zt->sendBuffer, zt->put, zt->put_get_usr);
    return ZCM_EOK;
}

/********************** STATICS **********************/
static size_t _serial_get_mtu(zcm_trans_t *zt)
{ return serial_get_mtu(cast(zt)); }

static int _serial_sendmsg(zcm_trans_t *zt, zcm_msg_t msg)
{ return serial_sendmsg(cast(zt), msg); }

static int _serial_recvmsg_enable(zcm_trans_t *zt, const char *channel, bool enable)
{ return serial_recvmsg_enable(cast(zt), channel, enable); }

static int _serial_recvmsg(zcm_trans_t *zt, zcm_msg_t *msg, int timeout)
{ return serial_recvmsg(cast(zt), msg, timeout); }

static int _serial_update(zcm_trans_t *zt)
{
    int rxRet = serial_update_rx(zt);
    int txRet = serial_update_tx(zt);
    return rxRet == ZCM_EOK ? txRet : rxRet;
}

static zcm_trans_methods_t methods = {
    &_serial_get_mtu,
    &_serial_sendmsg,
    &_serial_recvmsg_enable,
    &_serial_recvmsg,
    &_serial_update,
    &zcm_trans_generic_serial_destroy,
};

static zcm_trans_generic_serial_t *cast(zcm_trans_t *zt)
{
    assert(zt->vtbl == &methods);
    return (zcm_trans_generic_serial_t*)zt;
}

zcm_trans_t *zcm_trans_generic_serial_create(
        size_t (*get)(uint8_t* data, size_t nData, void* usr),
        size_t (*put)(const uint8_t* data, size_t nData, void* usr),
        void* put_get_usr,
        uint64_t (*timestamp_now)(void* usr),
        void* time_usr,
        size_t MTU,
        size_t bufSize)
{
    if (MTU == 0 || bufSize < FRAME_BYTES + MTU) return NULL;
    zcm_trans_generic_serial_t *zt = malloc(sizeof(zcm_trans_generic_serial_t));
    if (zt == NULL) return NULL;
    zt->mtu = MTU;
    zt->recvMsgData = malloc(zt->mtu * sizeof(uint8_t));
    if (zt->recvMsgData == NULL) {
        free(zt);
        return NULL;
    }

    zt->trans.trans_type = ZCM_NONBLOCKING;
    zt->trans.vtbl = &methods;
    if (!cb_init(&zt->sendBuffer, bufSize)) {
        free(zt->recvMsgData);
        free(zt);
        return NULL;
    }
    if (!cb_init(&zt->recvBuffer, bufSize)) {
        cb_deinit(&zt->sendBuffer);
        free(zt->recvMsgData);
        free(zt);
        return NULL;
    }

    zt->get = get;
    zt->put = put;
    zt->put_get_usr = put_get_usr;

    zt->time = timestamp_now;
    zt->time_usr = time_usr;

    return (zcm_trans_t*) zt;
}

void zcm_trans_generic_serial_destroy(zcm_trans_t* _zt)
{
    zcm_trans_generic_serial_t *zt = cast(_zt);
    cb_deinit(&zt->recvBuffer);
    cb_deinit(&zt->sendBuffer);
    free(zt->recvMsgData);
    free(zt);
}
