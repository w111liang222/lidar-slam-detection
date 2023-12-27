#include "generic_serial_circ_buff.h"

#include <stdlib.h>

#define ASSERT(x)

bool cb_init(circBuffer_t* cb, size_t sz)
{
    cb->capacity = sz;
    cb->front = 0;
    cb->back  = 0;
    if (cb->capacity == 0) return false;
    cb->data = malloc(cb->capacity * sizeof(uint8_t));
    if (cb->data == NULL) {
        cb->capacity = 0;
        return false;
    }
    return true;
}

void cb_deinit(circBuffer_t* cb)
{
    free(cb->data);
    cb->data = NULL;
    cb->capacity = 0;
}

size_t cb_size(const circBuffer_t* cb)
{
    if (cb->back >= cb->front) return cb->back - cb->front;
    else                       return cb->capacity - (cb->front - cb->back);
}

size_t cb_room(const circBuffer_t* cb)
{
    return cb->capacity - 1 - cb_size(cb);
}

void cb_push_back(circBuffer_t* cb, uint8_t d)
{
    ASSERT((cb->capacity > cb_size(cb) + 1) && "cb_push_back 1");
    ASSERT((cb_room(cb) > 0) && "cb_push_back 2");
    cb->data[cb->back++] = d;
    ASSERT((cb->back <= cb->capacity) && "cb_push_back 3");
    if (cb->back == cb->capacity) cb->back = 0;
}

uint8_t cb_front(const circBuffer_t* cb, size_t offset)
{
    ASSERT((cb_size(cb) > offset) && "cb_front 1");
    size_t idx = cb->front + offset;
    if (idx >= cb->capacity) idx -= cb->capacity;
    return cb->data[idx];
}

void cb_pop_front(circBuffer_t* cb, size_t num)
{
    ASSERT((cb_size(cb) >= num) && "cb_pop_front 1");
    cb->front += num;
    if (cb->front >= cb->capacity) cb->front -= cb->capacity;
}

void cb_pop_back(circBuffer_t* cb, size_t num)
{
    ASSERT((cb_size(cb) >= num) && "cb_pop_back 1");
    if (cb->back < num) cb->back = cb->capacity - (num - cb->back);
    else cb->back -= num;
}

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
size_t cb_flush_out(circBuffer_t* cb,
                    size_t (*write)(const uint8_t* data, size_t num, void* usr),
                    void* usr)
{
	size_t written = 0;
	size_t n;
    size_t sz = cb_size(cb);

    if (sz == 0) return 0;

    size_t contiguous = MIN(cb->capacity - cb->front, sz);
    size_t wrapped    = sz - contiguous;

    n = write(cb->data + cb->front, contiguous, usr);
    written += n;
    cb_pop_front(cb, n);

    // If we failed to write everything we tried to write, or if there's nothing
    // left to write, return.
    if (written != contiguous || wrapped == 0) return written;

    n = write(cb->data, wrapped, usr);
    written += n;
    cb_pop_front(cb, n);
    return written;
}

size_t cb_flush_in(circBuffer_t* cb,
                   size_t (*read)(uint8_t* data, size_t num, void* usr),
                   void* usr)
{
	size_t bytesRead = 0;
	size_t n;

    size_t room = cb_room(cb);

    // Find out how much room is left between back and end of buffer or back and front
    // of buffer. Because we already know there's room for whatever we're about to place,
    // if back < front, we can just read in every byte starting at "back".
    if (cb->back < cb->front) {
    	bytesRead += read(cb->data + cb->back, room, usr);
        cb->back += bytesRead;
        return bytesRead;
    }

    // Otherwise, we need to be a bit more careful about overflowing the back of the buffer.
    size_t contiguous = MIN(cb->capacity - cb->back, room);
    size_t wrapped    = room - contiguous;

    n = read(cb->data + cb->back, contiguous, usr);
    ASSERT((n <= contiguous) && "cb_flush_in 1");
    bytesRead += n;
    cb->back += n;
    if (n != contiguous) return bytesRead; // back could NOT have hit BUFFER_SIZE in this case

    // may need to wrap back here (if bytes >= BUFFER_SIZE - cb->back) but not otherwise
    ASSERT((cb->back <= cb->capacity) && "cb_flush_in 2");
    if (cb->back == cb->capacity) cb->back = 0;
    if (wrapped == 0) return bytesRead;

    n = read(cb->data, wrapped, usr);
    ASSERT((n <= wrapped) && "cb_flush_in 3");
    bytesRead += n;
    cb->back += n;
    return bytesRead;
}
#undef MIN
