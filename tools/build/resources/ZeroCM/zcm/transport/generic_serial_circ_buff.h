#ifndef _ZCM_TRANS_NONBLOCKING_SERIAL_CIRC_BUFF_H
#define _ZCM_TRANS_NONBLOCKING_SERIAL_CIRC_BUFF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


// Note: there is little to no error checking in this class, misuse will cause problems


typedef struct circBuffer_t circBuffer_t;
struct circBuffer_t
{
    uint8_t* data;
    size_t capacity;
    size_t front;
    size_t back;
};

bool cb_init(circBuffer_t* cb, size_t sz);

void cb_deinit(circBuffer_t* cb);

size_t cb_size(const circBuffer_t* cb);

size_t cb_room(const circBuffer_t* cb);

void cb_push_back(circBuffer_t* cb, uint8_t d);

uint8_t cb_front(const circBuffer_t* cb, size_t offset);

void cb_pop_back(circBuffer_t* cb, size_t num);

void cb_pop_front(circBuffer_t* cb, size_t num);

size_t cb_flush_out(circBuffer_t* cb,
                    size_t (*write)(const uint8_t* data, size_t num, void* usr),
                    void* usr);

size_t cb_flush_in(circBuffer_t* cb,
                   size_t (*read)(uint8_t* data, size_t num, void* usr),
                   void* usr);

#ifdef __cplusplus
}
#endif

#endif /* _ZCM_TRANS_NONBLOCKING_SERIAL_CIRC_BUFF_H */
