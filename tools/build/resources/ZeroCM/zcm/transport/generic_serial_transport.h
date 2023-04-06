#ifndef _ZCM_TRANS_NONBLOCKING_SERIAL_H
#define _ZCM_TRANS_NONBLOCKING_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include "zcm/zcm.h"
#include "zcm/transport.h"

zcm_trans_t *zcm_trans_generic_serial_create(
        size_t (*get)(uint8_t* data, size_t nData, void* usr),
        size_t (*put)(const uint8_t* data, size_t nData, void* usr),
        void* put_get_usr,
        uint64_t (*timestamp_now)(void* usr),
        void* time_usr,
        size_t MTU, size_t bufSize);

// frees all resources inside of zt and frees zt itself
void zcm_trans_generic_serial_destroy(zcm_trans_t* zt);

int serial_update_rx(zcm_trans_t *zt);
int serial_update_tx(zcm_trans_t *zt);

#ifdef __cplusplus
}
#endif

#endif /* _ZCM_TRANS_NONBLOCKING_SERIAL_H */
