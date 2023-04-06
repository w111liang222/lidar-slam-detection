#ifndef _CB_TEST_H
#define _CB_TEST_H

#include "zcm/zcm.h"

#ifdef __cplusplus
extern "C" {
#endif

struct uv_zcm_msg_handler_t;

uv_zcm_msg_handler_t* uv_zcm_msg_handler_create(zcm_msg_handler_t cb, void* usr);
void                  uv_zcm_msg_handler_trigger(const zcm_recv_buf_t* rbuf,
                                                 const char* channel, void* _uvCb);
void                  uv_zcm_msg_handler_destroy(uv_zcm_msg_handler_t* uvCb);

#ifdef __cplusplus
}
#endif

#endif /* _CB_TEST_H */
