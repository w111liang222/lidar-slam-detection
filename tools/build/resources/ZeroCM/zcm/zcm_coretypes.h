#ifndef _ZCM_LIB_INLINE_H
#define _ZCM_LIB_INLINE_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define   ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS (1)
#define  ZCM_CORETYPES_INT16_NUM_BYTES_ON_BUS (2)
#define  ZCM_CORETYPES_INT32_NUM_BYTES_ON_BUS (4)
#define  ZCM_CORETYPES_INT64_NUM_BYTES_ON_BUS (8)
#define  ZCM_CORETYPES_FLOAT_NUM_BYTES_ON_BUS (4)
#define ZCM_CORETYPES_DOUBLE_NUM_BYTES_ON_BUS (8)

#define   ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS  (8)

static inline void *zcm_malloc(uint32_t sz)
{
    if (sz) return malloc(sz);
    return NULL;
}

static inline void zcm_free(void* mem)
{
    free(mem);
}

typedef struct ___zcm_hash_ptr __zcm_hash_ptr;
struct ___zcm_hash_ptr
{
    const __zcm_hash_ptr *parent;
    void *v;
};

static inline uint32_t __bitfield_encoded_size(uint32_t numbits)
{
    uint32_t size = numbits / ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS;
    if (numbits - size * ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS != 0) ++size;
    return size;
}

// Convert numbits into an appropriate increment of offset_byte and offset_bit
static inline void __bitfield_advance_offset(uint32_t* offset_byte, uint32_t* offset_bit, uint32_t numbits)
{
    uint32_t tmp_num_bytes, tmp_num_bits;

    tmp_num_bytes = numbits / ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS;
    tmp_num_bits = numbits - (tmp_num_bytes * ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS);
    *offset_byte += tmp_num_bytes;
    *offset_bit += tmp_num_bits;
    if (*offset_bit >= ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS) {
        *offset_bit -= 8;
        ++*offset_byte;
    }
}

// returns number of bits consumed
#define X(NAME, TYPE, UNSIGNED_TYPE)                                                                                \
static inline int __ ## NAME ## _encode_array_bits(void *_buf,                                                      \
                                                   uint32_t offset_bytes, uint32_t offset_bits, uint32_t maxbytes,  \
                                                   const TYPE *p, uint32_t elements, uint32_t numbits)              \
{                                                                                                                   \
    uint32_t total_bits = elements * numbits;                                                                       \
    if (maxbytes < __bitfield_encoded_size(total_bits + offset_bits)) return -1;                                    \
                                                                                                                    \
    uint32_t pos_byte = offset_bytes;                                                                               \
    uint32_t pos_bit = offset_bits;                                                                                 \
    uint32_t element;                                                                                               \
    uint8_t *buf = (uint8_t*) _buf;                                                                                 \
    UNSIGNED_TYPE* unsigned_p = (UNSIGNED_TYPE*)p;                                                                  \
                                                                                                                    \
    for (element = 0; element < elements; ++element) {                                                              \
        uint32_t bits_left = numbits;                                                                               \
        while (bits_left > 0) {                                                                                     \
            if (pos_bit == 0) buf[pos_byte] = 0;                                                                    \
            int32_t shift = (int32_t)(pos_bit + bits_left) - ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS;                    \
            if (shift < 0) {                                                                                        \
                uint8_t mask = (1 << bits_left) - 1;                                                                \
                shift = -shift;                                                                                     \
                buf[pos_byte] |= (unsigned_p[element] & mask) << shift;                                             \
                pos_bit += bits_left;                                                                               \
                break;                                                                                              \
            }                                                                                                       \
            /* the cast here just needs to be bigger than a uint8_t */                                              \
            uint8_t mask = ((uint16_t)1 << (bits_left - shift)) - 1;                                                \
            buf[pos_byte] |= (unsigned_p[element] >> shift) & mask;                                                 \
            bits_left = shift;                                                                                      \
            pos_bit = 0;                                                                                            \
            ++pos_byte;                                                                                             \
        }                                                                                                           \
    }                                                                                                               \
                                                                                                                    \
    return total_bits;                                                                                              \
}
    X(byte, uint8_t, uint8_t)
    X(int8_t, int8_t, uint8_t)
    X(int16_t, int16_t, uint16_t)
    X(int32_t, int32_t, uint32_t)
    X(int64_t, int64_t, uint64_t)
#undef X

#define X(NAME, TYPE, SIGN, INT8_SIGN_TYPE)                                                     \
static inline int __ ## NAME ##_decode_array_ ## SIGN(                                          \
    const void *_buf,                                                                           \
    uint32_t offset_bytes, uint32_t offset_bits, uint32_t maxbytes,                             \
    TYPE* p, uint32_t elements, uint32_t numbits                                                \
)                                                                                               \
{                                                                                               \
    uint32_t total_bits = elements * numbits;                                                   \
    if (maxbytes < __bitfield_encoded_size(total_bits + offset_bits)) return -1;                \
                                                                                                \
    uint32_t pos_byte = offset_bytes;                                                           \
    uint32_t pos_bit = offset_bits;                                                             \
    uint32_t element;                                                                           \
                                                                                                \
    const uint8_t *buf = (const uint8_t*) _buf;                                                 \
                                                                                                \
    for (element = 0; element < elements; ++element) {                                          \
        uint32_t bits_left = numbits;                                                           \
        while (bits_left > 0) {                                                                 \
            uint32_t available_bits = ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS - pos_bit;             \
            uint32_t bits_covered = available_bits < bits_left ? available_bits : bits_left;    \
            uint8_t mask = ((1 << bits_covered) - 1) <<                                         \
                           (ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS - bits_covered - pos_bit);       \
            uint8_t payload = (buf[pos_byte] & mask) << pos_bit;                                \
            int32_t shift = ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS - bits_left;                     \
            /* Sign extend the first shift and none after that */                               \
            if (bits_left == numbits) {                                                         \
                if (shift < 0) p[element] = ((int64_t)((INT8_SIGN_TYPE)payload)) << -shift;     \
                else           p[element] = ((int64_t)((INT8_SIGN_TYPE)payload)) >>  shift;     \
            } else {                                                                            \
                if (shift < 0) p[element] |= ((uint64_t)payload) << -shift;                     \
                else           p[element] |= payload >>  shift;                                 \
            }                                                                                   \
            bits_left -= bits_covered;                                                          \
            pos_bit += bits_covered;                                                            \
            if (pos_bit == ZCM_CORETYPES_INT8_NUM_BITS_ON_BUS) {                                \
                pos_bit = 0;                                                                    \
                ++pos_byte;                                                                     \
            }                                                                                   \
        };                                                                                      \
    }                                                                                           \
                                                                                                \
    return total_bits;                                                                          \
}
    X(byte,    uint8_t, bits,             uint8_t)
    X(int8_t,  int8_t,  bits,             uint8_t)
    X(int16_t, int16_t, bits,             uint8_t)
    X(int32_t, int32_t, bits,             uint8_t)
    X(int64_t, int64_t, bits,             uint8_t)
    X(int8_t,  int8_t,  bits_sign_extend,  int8_t)
    X(int16_t, int16_t, bits_sign_extend,  int8_t)
    X(int32_t, int32_t, bits_sign_extend,  int8_t)
    X(int64_t, int64_t, bits_sign_extend,  int8_t)
#undef X

/**
 * BOOLEAN
 */
#define __boolean_hash_recursive __int8_t_hash_recursive
#define __boolean_decode_array_cleanup __int8_t_decode_array_cleanup
#define __boolean_encoded_array_size __int8_t_encoded_array_size
#define __boolean_encode_array __int8_t_encode_array
#define __boolean_decode_array __int8_t_decode_array
#define __boolean_encode_little_endian_array __int8_t_encode_little_endian_array
#define __boolean_decode_little_endian_array __int8_t_decode_little_endian_array
#define __boolean_clone_array __int8_t_clone_array

/**
 * BYTE
 */
#define __byte_hash_recursive(p) 0
#define __byte_decode_array_cleanup(p, sz) {}

static inline uint32_t __byte_encoded_array_size(const uint8_t *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS * elements;
}

static inline int __byte_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const uint8_t *p, uint32_t elements)
{
    uint32_t total_size = __byte_encoded_array_size(p, elements);

    if (maxlen < total_size) return -1;

    uint8_t *buf = (uint8_t*) _buf;
    memcpy(&buf[offset], p, elements);

    return elements;
}

static inline int __byte_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, uint8_t *p, uint32_t elements)
{
    uint32_t total_size = __byte_encoded_array_size(p, elements);

    if (maxlen < total_size) return -1;

    uint8_t *buf = (uint8_t*) _buf;
    memcpy(p, &buf[offset], elements);

    return elements;
}

static inline int __byte_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const uint8_t *p, uint32_t elements)
{
    return __byte_encode_array(_buf, offset, maxlen, p, elements);
}

static inline int __byte_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, uint8_t *p, uint32_t elements)
{
    return __byte_decode_array(_buf, offset, maxlen, p, elements);
}

static inline uint32_t __byte_clone_array(const uint8_t *p, uint8_t *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(uint8_t);
    memcpy(q, p, n);
    return n;
}
/**
 * INT8_T
 */
#define __int8_t_hash_recursive(p) 0
#define __int8_t_decode_array_cleanup(p, sz) {}

static inline uint32_t __int8_t_encoded_array_size(const int8_t *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS * elements;
}

static inline int __int8_t_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const int8_t *p, uint32_t elements)
{
    uint32_t total_size = __int8_t_encoded_array_size(p, elements);

    if (maxlen < total_size) return -1;

    int8_t *buf = (int8_t*) _buf;
    memcpy(&buf[offset], p, elements);

    return elements;
}

static inline int __int8_t_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, int8_t *p, uint32_t elements)
{
    uint32_t total_size = __int8_t_encoded_array_size(p, elements);

    if (maxlen < total_size) return -1;

    int8_t *buf = (int8_t*) _buf;
    memcpy(p, &buf[offset], elements);

    return elements;
}

static inline int __int8_t_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const int8_t *p, uint32_t elements)
{
    return __int8_t_encode_array(_buf, offset, maxlen, p, elements);
}

static inline int __int8_t_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, int8_t *p, uint32_t elements)
{
    return __int8_t_decode_array(_buf, offset, maxlen, p, elements);
}

static inline uint32_t __int8_t_clone_array(const int8_t *p, int8_t *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(int8_t);
    memcpy(q, p, n);
    return n;
}

/**
 * INT16_T
 */
#define __int16_t_hash_recursive(p) 0
#define __int16_t_decode_array_cleanup(p, sz) {}

static inline uint32_t __int16_t_encoded_array_size(const int16_t *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_INT16_NUM_BYTES_ON_BUS * elements;
}

static inline int __int16_t_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const int16_t *p, uint32_t elements)
{
    uint32_t total_size = __int16_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint16_t *unsigned_p = (uint16_t*)p;
    for (element = 0; element < elements; ++element) {
        uint16_t v = unsigned_p[element];
        buf[pos++] = (v>>8) & 0xff;
        buf[pos++] = (v & 0xff);
    }

    return total_size;
}

static inline int __int16_t_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, int16_t *p, uint32_t elements)
{
    uint32_t total_size = __int16_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        p[element] = (buf[pos]<<8) + buf[pos+1];
        pos+=2;
    }

    return total_size;
}

static inline int __int16_t_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const int16_t *p, uint32_t elements)
{
    uint32_t total_size = __int16_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint16_t *unsigned_p = (uint16_t*)p;
    for (element = 0; element < elements; ++element) {
        uint16_t v = unsigned_p[element];
        buf[pos++] = (v & 0xff);
        buf[pos++] = (v>>8) & 0xff;
    }

    return total_size;
}

static inline int __int16_t_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, int16_t *p, uint32_t elements)
{
    uint32_t total_size = __int16_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        p[element] = (buf[pos+1]<<8) + buf[pos];
        pos+=2;
    }

    return total_size;
}

static inline uint32_t __int16_t_clone_array(const int16_t *p, int16_t *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(int16_t);
    memcpy(q, p, n);
    return n;
}

/**
 * INT32_T
 */
#define __int32_t_hash_recursive(p) 0
#define __int32_t_decode_array_cleanup(p, sz) {}

static inline uint32_t __int32_t_encoded_array_size(const int32_t *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_INT32_NUM_BYTES_ON_BUS * elements;
}

static inline int __int32_t_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const int32_t *p, uint32_t elements)
{
    uint32_t total_size = __int32_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint32_t* unsigned_p = (uint32_t*)p;
    for (element = 0; element < elements; ++element) {
        uint32_t v = unsigned_p[element];
        buf[pos++] = (v>>24)&0xff;
        buf[pos++] = (v>>16)&0xff;
        buf[pos++] = (v>>8)&0xff;
        buf[pos++] = (v & 0xff);
    }

    return total_size;
}

static inline int __int32_t_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, int32_t *p, uint32_t elements)
{
    uint32_t total_size = __int32_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        p[element] = (((uint32_t)buf[pos+0])<<24) +
                     (((uint32_t)buf[pos+1])<<16) +
                     (((uint32_t)buf[pos+2])<<8) +
                      ((uint32_t)buf[pos+3]);
        pos+=4;
    }

    return total_size;
}

static inline int __int32_t_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const int32_t *p, uint32_t elements)
{
    uint32_t total_size = __int32_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint32_t* unsigned_p = (uint32_t*)p;
    for (element = 0; element < elements; ++element) {
        uint32_t v = unsigned_p[element];
        buf[pos++] = (v & 0xff);
        buf[pos++] = (v>>8)&0xff;
        buf[pos++] = (v>>16)&0xff;
        buf[pos++] = (v>>24)&0xff;
    }

    return total_size;
}

static inline int __int32_t_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, int32_t *p, uint32_t elements)
{
    uint32_t total_size = __int32_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        p[element] = (((uint32_t)buf[pos+3])<<24) +
                      (((uint32_t)buf[pos+2])<<16) +
                      (((uint32_t)buf[pos+1])<<8) +
                       ((uint32_t)buf[pos+0]);
        pos+=4;
    }

    return total_size;
}

static inline uint32_t __int32_t_clone_array(const int32_t *p, int32_t *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(int32_t);
    memcpy(q, p, n);
    return n;
}

/**
 * INT64_T
 */
#define __int64_t_hash_recursive(p) 0
#define __int64_t_decode_array_cleanup(p, sz) {}

static inline uint32_t __int64_t_encoded_array_size(const int64_t *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_INT64_NUM_BYTES_ON_BUS * elements;
}

static inline int __int64_t_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const int64_t *p, uint32_t elements)
{
    uint32_t total_size = __int64_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint64_t* unsigned_p = (uint64_t*)p;
    for (element = 0; element < elements; ++element) {
        uint64_t v = unsigned_p[element];
        buf[pos++] = (v>>56)&0xff;
        buf[pos++] = (v>>48)&0xff;
        buf[pos++] = (v>>40)&0xff;
        buf[pos++] = (v>>32)&0xff;
        buf[pos++] = (v>>24)&0xff;
        buf[pos++] = (v>>16)&0xff;
        buf[pos++] = (v>>8)&0xff;
        buf[pos++] = (v & 0xff);
    }

    return total_size;
}

static inline int __int64_t_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, int64_t *p, uint32_t elements)
{
    uint32_t total_size = __int64_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        uint64_t a = (((uint32_t)buf[pos+0])<<24) +
                     (((uint32_t)buf[pos+1])<<16) +
                     (((uint32_t)buf[pos+2])<<8) +
                      ((uint32_t)buf[pos+3]);
        pos+=4;
        uint64_t b = (((uint32_t)buf[pos+0])<<24) +
                     (((uint32_t)buf[pos+1])<<16) +
                     (((uint32_t)buf[pos+2])<<8) +
                      ((uint32_t)buf[pos+3]);
        pos+=4;
        p[element] = (a<<32) + (b&0xffffffff);
    }

    return total_size;
}

static inline int __int64_t_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const int64_t *p, uint32_t elements)
{
    uint32_t total_size = __int64_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    const uint64_t* unsigned_p = (uint64_t*)p;
    for (element = 0; element < elements; ++element) {
        uint64_t v = unsigned_p[element];
        buf[pos++] = (v & 0xff);
        buf[pos++] = (v>>8)&0xff;
        buf[pos++] = (v>>16)&0xff;
        buf[pos++] = (v>>24)&0xff;
        buf[pos++] = (v>>32)&0xff;
        buf[pos++] = (v>>40)&0xff;
        buf[pos++] = (v>>48)&0xff;
        buf[pos++] = (v>>56)&0xff;
    }

    return total_size;
}

static inline int __int64_t_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, int64_t *p, uint32_t elements)
{
    uint32_t total_size = __int64_t_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        uint64_t b = (((uint32_t)buf[pos+3])<<24) +
                     (((uint32_t)buf[pos+2])<<16) +
                     (((uint32_t)buf[pos+1])<<8) +
                      ((uint32_t)buf[pos+0]);
        pos+=4;
        uint64_t a = (((uint32_t)buf[pos+3])<<24) +
                     (((uint32_t)buf[pos+2])<<16) +
                     (((uint32_t)buf[pos+1])<<8) +
                      ((uint32_t)buf[pos+0]);
        pos+=4;
        p[element] = (a<<32) + (b&0xffffffff);
    }

    return total_size;
}

static inline uint32_t __int64_t_clone_array(const int64_t *p, int64_t *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(int64_t);
    memcpy(q, p, n);
    return n;
}

/**
 * FLOAT
 */
typedef union __zcm__float_uint32_t {
    float flt;
    uint32_t uint;
} __zcm__float_uint32_t;

#define __float_hash_recursive(p) 0
#define __float_decode_array_cleanup(p, sz) {}

static inline uint32_t __float_encoded_array_size(const float *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_FLOAT_NUM_BYTES_ON_BUS * elements;
}

static inline int __float_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const float *p, uint32_t elements)
{
    uint32_t total_size = __float_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__float_uint32_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.flt = p[element];
        buf[pos++] = (tmp.uint >> 24) & 0xff;
        buf[pos++] = (tmp.uint >> 16) & 0xff;
        buf[pos++] = (tmp.uint >>  8) & 0xff;
        buf[pos++] = (tmp.uint      ) & 0xff;
    }

    return total_size;
}

static inline int __float_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, float *p, uint32_t elements)
{
    uint32_t total_size = __float_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__float_uint32_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.uint = (((uint32_t)buf[pos + 0]) << 24) |
                   (((uint32_t)buf[pos + 1]) << 16) |
                   (((uint32_t)buf[pos + 2]) <<  8) |
                    ((uint32_t)buf[pos + 3]);
        p[element] = tmp.flt;
        pos += 4;
    }

    return total_size;
}

static inline int __float_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const float *p, uint32_t elements)
{
    uint32_t total_size = __float_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__float_uint32_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.flt = p[element];
        buf[pos++] = (tmp.uint      ) & 0xff;
        buf[pos++] = (tmp.uint >>  8) & 0xff;
        buf[pos++] = (tmp.uint >> 16) & 0xff;
        buf[pos++] = (tmp.uint >> 24) & 0xff;
    }

    return total_size;
}

static inline int __float_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, float *p, uint32_t elements)
{
    uint32_t total_size = __float_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__float_uint32_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.uint = (((uint32_t)buf[pos + 3]) << 24) |
                   (((uint32_t)buf[pos + 2]) << 16) |
                   (((uint32_t)buf[pos + 1]) <<  8) |
                    ((uint32_t)buf[pos + 0]);
        p[element] = tmp.flt;
        pos += 4;
    }

    return total_size;
}

static inline uint32_t __float_clone_array(const float *p, float *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(float);
    memcpy(q, p, n);
    return n;
}

/**
 * DOUBLE
 */
typedef union __zcm__double_uint64_t {
    double dbl;
    uint64_t uint;
} __zcm__double_uint64_t;

#define __double_hash_recursive(p) 0
#define __double_decode_array_cleanup(p, sz) {}

static inline uint32_t __double_encoded_array_size(const double *p, uint32_t elements)
{
    (void)p;
    return ZCM_CORETYPES_DOUBLE_NUM_BYTES_ON_BUS * elements;
}

static inline int __double_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, const double *p, uint32_t elements)
{
    uint32_t total_size = __double_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__double_uint64_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.dbl = p[element];
        buf[pos++] = (tmp.uint >> 56) & 0xff;
        buf[pos++] = (tmp.uint >> 48) & 0xff;
        buf[pos++] = (tmp.uint >> 40) & 0xff;
        buf[pos++] = (tmp.uint >> 32) & 0xff;
        buf[pos++] = (tmp.uint >> 24) & 0xff;
        buf[pos++] = (tmp.uint >> 16) & 0xff;
        buf[pos++] = (tmp.uint >>  8) & 0xff;
        buf[pos++] = (tmp.uint      ) & 0xff;
    }

    return total_size;
}

static inline int __double_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, double *p, uint32_t elements)
{
    uint32_t total_size = __double_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__double_uint64_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        uint64_t a = (((uint32_t) buf[pos + 0]) << 24) +
                     (((uint32_t) buf[pos + 1]) << 16) +
                     (((uint32_t) buf[pos + 2]) <<  8) +
                      ((uint32_t) buf[pos + 3]);
        pos += 4;
        uint64_t b = (((uint32_t) buf[pos + 0]) << 24) +
                     (((uint32_t) buf[pos + 1]) << 16) +
                     (((uint32_t) buf[pos + 2]) <<  8) +
                      ((uint32_t) buf[pos + 3]);
        pos += 4;
        tmp.uint = (a << 32) + (b & 0xffffffff);
        p[element] = tmp.dbl;
    }

    return total_size;
}

static inline int __double_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, const double *p, uint32_t elements)
{
    uint32_t total_size = __double_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__double_uint64_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        tmp.dbl = p[element];
        buf[pos++] = (tmp.uint      ) & 0xff;
        buf[pos++] = (tmp.uint >>  8) & 0xff;
        buf[pos++] = (tmp.uint >> 16) & 0xff;
        buf[pos++] = (tmp.uint >> 24) & 0xff;
        buf[pos++] = (tmp.uint >> 32) & 0xff;
        buf[pos++] = (tmp.uint >> 40) & 0xff;
        buf[pos++] = (tmp.uint >> 48) & 0xff;
        buf[pos++] = (tmp.uint >> 56) & 0xff;
    }

    return total_size;
}

static inline int __double_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, double *p, uint32_t elements)
{
    uint32_t total_size = __double_encoded_array_size(p, elements);
    uint8_t *buf = (uint8_t*) _buf;
    uint32_t pos = offset;
    uint32_t element;
    __zcm__double_uint64_t tmp;

    if (maxlen < total_size) return -1;

    for (element = 0; element < elements; ++element) {
        uint64_t b = (((uint32_t)buf[pos + 3]) << 24) +
                     (((uint32_t)buf[pos + 2]) << 16) +
                     (((uint32_t)buf[pos + 1]) <<  8) +
                      ((uint32_t)buf[pos + 0]);
        pos += 4;
        uint64_t a = (((uint32_t)buf[pos + 3]) << 24) +
                     (((uint32_t)buf[pos + 2]) << 16) +
                     (((uint32_t)buf[pos + 1]) <<  8) +
                      ((uint32_t)buf[pos + 0]);
        pos += 4;
        tmp.uint = (a << 32) + (b & 0xffffffff);
        p[element] = tmp.dbl;
    }

    return total_size;
}

static inline uint32_t __double_clone_array(const double *p, double *q, uint32_t elements)
{
    uint32_t n = elements * sizeof(double);
    memcpy(q, p, n);
    return n;
}

/**
 * STRING
 */
#define __string_hash_recursive(p) 0

static inline int __string_decode_array_cleanup(char **s, uint32_t elements)
{
    uint32_t element;
    for (element = 0; element < elements; ++element)
        free(s[element]);
    return 0;
}

// TODO: Figure out why "const char * const * p" doesn't work
static inline uint32_t __string_encoded_array_size(char * const *s, uint32_t elements)
{
    uint32_t size = 0, element;
    for (element = 0; element < elements; ++element)
        size += ZCM_CORETYPES_INT32_NUM_BYTES_ON_BUS + strlen(s[element]) + ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS;

    return size;
}

// TODO: Figure out why "const char * const * p" doesn't work
static inline int __string_encode_array(void *_buf, uint32_t offset, uint32_t maxlen, char * const *p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {
        int32_t length = strlen(p[element]) + ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS; // length includes \0

        thislen = __int32_t_encode_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(_buf, offset + pos, maxlen - pos, (int8_t*) p[element], length);
        if (thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

static inline int __string_decode_array(const void *_buf, uint32_t offset, uint32_t maxlen, char **p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {
        int32_t length;

        // read length including \0
        thislen = __int32_t_decode_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element] = (char*) zcm_malloc(length);
        thislen = __int8_t_decode_array(_buf, offset + pos, maxlen - pos, (int8_t*) p[element], length);
        if (thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

// TODO: Figure out why "const char * const * p" doesn't work
static inline int __string_encode_little_endian_array(void *_buf, uint32_t offset, uint32_t maxlen, char * const *p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {
        int32_t length = strlen(p[element]) + 1; // length includes \0

        thislen = __int32_t_encode_little_endian_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_little_endian_array(_buf, offset + pos, maxlen - pos, (int8_t*) p[element], length);
        if (thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

static inline int __string_decode_little_endian_array(const void *_buf, uint32_t offset, uint32_t maxlen, char **p, uint32_t elements)
{
    uint32_t pos = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {
        int32_t length;

        // read length including \0
        thislen = __int32_t_decode_little_endian_array(_buf, offset + pos, maxlen - pos, &length, 1);
        if (thislen < 0) return thislen; else pos += thislen;

        p[element] = (char*) zcm_malloc(length);
        thislen = __int8_t_decode_little_endian_array(_buf, offset + pos, maxlen - pos, (int8_t*) p[element], length);
        if (thislen < 0) return thislen; else pos += thislen;
    }

    return pos;
}

// TODO: Figure out why "const char * const * p" doesn't work
static inline uint32_t __string_clone_array(char * const *p, char **q, uint32_t elements)
{
    uint32_t ret = 0, element;
    for (element = 0; element < elements; ++element) {
        // because strdup is not C99
        uint32_t len = strlen(p[element]) + 1;
        ret += len;
        q[element] = (char*) zcm_malloc (len);
        memcpy (q[element], p[element], len);
    }
    return ret;
}

/**
 * Describes the type of a single field in an ZCM message.
 */
typedef enum {
    ZCM_FIELD_INT8_T,
    ZCM_FIELD_INT16_T,
    ZCM_FIELD_INT32_T,
    ZCM_FIELD_INT64_T,
    ZCM_FIELD_BYTE,
    ZCM_FIELD_FLOAT,
    ZCM_FIELD_DOUBLE,
    ZCM_FIELD_STRING,
    ZCM_FIELD_BOOLEAN,
    ZCM_FIELD_USER_TYPE
} zcm_field_type_t;

#define ZCM_TYPE_FIELD_MAX_DIM 50

/**
 * Describes a single zcmtype field's datatype and array dimmensions
 */
typedef struct _zcm_field_t zcm_field_t;
struct _zcm_field_t
{
    /**
     * name of the field
     */
    const char *name;

    /**
     * datatype of the field
     **/
    zcm_field_type_t type;

    /**
     * datatype of the field (in string format)
     * this should be the same as in the zcm type decription file
     */
    const char *typestr;

    /**
     * number of array dimensions
     * if the field is scalar, num_dim should equal 0
     */
    int num_dim;

    /**
     * the size of each dimension. Valid on [0:num_dim-1].
     */
    int32_t dim_size[ZCM_TYPE_FIELD_MAX_DIM];

    /**
     * a boolean describing whether the dimension is
     * variable. Valid on [0:num_dim-1].
     */
    int8_t  dim_is_variable[ZCM_TYPE_FIELD_MAX_DIM];

    /**
     * a data pointer to the start of this field
     */
    void *data;
};

typedef int        (*zcm_encode_t)(void *buf, uint32_t offset, uint32_t maxlen, const void *p);
typedef int        (*zcm_decode_t)(const void *buf, uint32_t offset, uint32_t maxlen, void *p);
typedef int        (*zcm_decode_cleanup_t)(void *p);
typedef uint32_t   (*zcm_encoded_size_t)(const void *p);
typedef uint32_t   (*zcm_struct_size_t)(void);
typedef uint32_t   (*zcm_num_fields_t)(void);
typedef int        (*zcm_get_field_t)(const void *p, uint32_t i, zcm_field_t *f);
typedef int64_t    (*zcm_get_hash_t)(void);

/**
 * Describes an zcmtype info, enabling introspection
 */
typedef struct _zcm_type_info_t zcm_type_info_t;
struct _zcm_type_info_t
{
    zcm_encode_t          encode;
    zcm_decode_t          decode;
    zcm_decode_cleanup_t  decode_cleanup;
    zcm_encoded_size_t    encoded_size;
    zcm_struct_size_t     struct_size;
    zcm_num_fields_t      num_fields;
    zcm_get_field_t       get_field;
    zcm_get_hash_t        get_hash;

};

#ifdef __cplusplus
}
#endif

#endif // _ZCM_LIB_INLINE_H
