// THIS IS AN AUTOMATICALLY GENERATED FILE.
// DO NOT MODIFY BY HAND!!
//
// Generated by zcm-gen

#include <string.h>
#ifndef ZCM_EMBEDDED
#include <stdio.h>
#endif
#include "sensor_msgs/sensor_msgs_NavSatFix.h"

static int __sensor_msgs_NavSatFix_hash_computed = 0;
static uint64_t __sensor_msgs_NavSatFix_hash;

uint64_t __sensor_msgs_NavSatFix_hash_recursive(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __sensor_msgs_NavSatFix_get_hash)
            return 0;

    __zcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__sensor_msgs_NavSatFix_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xa6e2262989466c08LL
         + __std_msgs_Header_hash_recursive(&cp)
         + __sensor_msgs_NavSatStatus_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __sensor_msgs_NavSatFix_get_hash(void)
{
    if (!__sensor_msgs_NavSatFix_hash_computed) {
        __sensor_msgs_NavSatFix_hash = (int64_t)__sensor_msgs_NavSatFix_hash_recursive(NULL);
        __sensor_msgs_NavSatFix_hash_computed = 1;
    }

    return __sensor_msgs_NavSatFix_hash;
}

int __sensor_msgs_NavSatFix_encode_array(void* buf, uint32_t offset, uint32_t maxlen, const sensor_msgs_NavSatFix* p, uint32_t elements)
{
    uint32_t pos_byte = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        /* header */
        thislen = __std_msgs_Header_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].header), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* status */
        thislen = __sensor_msgs_NavSatStatus_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].status), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* latitude */
        thislen = __double_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].latitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* longitude */
        thislen = __double_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].longitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* altitude */
        thislen = __double_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].altitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* position_covariance */
        thislen = __double_encode_array(buf, offset + pos_byte, maxlen - pos_byte, p[element].position_covariance, 64);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* position_covariance_type */
        thislen = __int8_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].position_covariance_type), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

    }
    return pos_byte;
}

int sensor_msgs_NavSatFix_encode(void* buf, uint32_t offset, uint32_t maxlen, const sensor_msgs_NavSatFix* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_msgs_NavSatFix_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __sensor_msgs_NavSatFix_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t __sensor_msgs_NavSatFix_encoded_array_size(const sensor_msgs_NavSatFix* p, uint32_t elements)
{
    uint32_t size = 0, element;
    for (element = 0; element < elements; ++element) {

        size += __std_msgs_Header_encoded_array_size(&(p[element].header), 1); // header

        size += __sensor_msgs_NavSatStatus_encoded_array_size(&(p[element].status), 1); // status

        size += __double_encoded_array_size(&(p[element].latitude), 1); // latitude

        size += __double_encoded_array_size(&(p[element].longitude), 1); // longitude

        size += __double_encoded_array_size(&(p[element].altitude), 1); // altitude

        size += __double_encoded_array_size(p[element].position_covariance, 64); // position_covariance

        size += __int8_t_encoded_array_size(&(p[element].position_covariance_type), 1); // position_covariance_type

    }
    return size;
}

uint32_t sensor_msgs_NavSatFix_encoded_size(const sensor_msgs_NavSatFix* p)
{
    return 8 + __sensor_msgs_NavSatFix_encoded_array_size(p, 1);
}

uint32_t sensor_msgs_NavSatFix_struct_size(void)
{
    return sizeof(sensor_msgs_NavSatFix);
}

uint32_t sensor_msgs_NavSatFix_num_fields(void)
{
    return 7;
}

int sensor_msgs_NavSatFix_get_field(const sensor_msgs_NavSatFix* p, uint32_t i, zcm_field_t* f)
{
    if (i >= sensor_msgs_NavSatFix_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            /* Header */
            f->name = "header";
            f->type = ZCM_FIELD_USER_TYPE;
            f->typestr = "std_msgs.Header";
            f->num_dim = 0;
            f->data = (void*) &p->header;
            return 0;
        }
        
        case 1: {
            /* NavSatStatus */
            f->name = "status";
            f->type = ZCM_FIELD_USER_TYPE;
            f->typestr = "sensor_msgs.NavSatStatus";
            f->num_dim = 0;
            f->data = (void*) &p->status;
            return 0;
        }
        
        case 2: {
            f->name = "latitude";
            f->type = ZCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void*) &p->latitude;
            return 0;
        }
        
        case 3: {
            f->name = "longitude";
            f->type = ZCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void*) &p->longitude;
            return 0;
        }
        
        case 4: {
            f->name = "altitude";
            f->type = ZCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void*) &p->altitude;
            return 0;
        }
        
        case 5: {
            f->name = "position_covariance";
            f->type = ZCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 1;
            f->dim_size[0] = 64;
            f->dim_is_variable[0] = 0;
            f->data = (void*) &p->position_covariance;
            return 0;
        }
        
        case 6: {
            f->name = "position_covariance_type";
            f->type = ZCM_FIELD_INT8_T;
            f->typestr = "int8_t";
            f->num_dim = 0;
            f->data = (void*) &p->position_covariance_type;
            return 0;
        }
        
        default:
            return 1;
    }
}

const zcm_type_info_t* sensor_msgs_NavSatFix_get_type_info(void)
{
    static int init = 0;
    static zcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (zcm_encode_t) sensor_msgs_NavSatFix_encode;
        typeinfo.decode         = (zcm_decode_t) sensor_msgs_NavSatFix_decode;
        typeinfo.decode_cleanup = (zcm_decode_cleanup_t) sensor_msgs_NavSatFix_decode_cleanup;
        typeinfo.encoded_size   = (zcm_encoded_size_t) sensor_msgs_NavSatFix_encoded_size;
        typeinfo.struct_size    = (zcm_struct_size_t)  sensor_msgs_NavSatFix_struct_size;
        typeinfo.num_fields     = (zcm_num_fields_t) sensor_msgs_NavSatFix_num_fields;
        typeinfo.get_field      = (zcm_get_field_t) sensor_msgs_NavSatFix_get_field;
        typeinfo.get_hash       = (zcm_get_hash_t) __sensor_msgs_NavSatFix_get_hash;
    }
    
    return &typeinfo;
}

int __sensor_msgs_NavSatFix_decode_array(const void* buf, uint32_t offset, uint32_t maxlen, sensor_msgs_NavSatFix* p, uint32_t elements)
{
    uint32_t pos_byte = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        /* header */
        thislen = __std_msgs_Header_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].header), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* status */
        thislen = __sensor_msgs_NavSatStatus_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].status), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* latitude */
        thislen = __double_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].latitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* longitude */
        thislen = __double_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].longitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* altitude */
        thislen = __double_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].altitude), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* position_covariance */
        thislen = __double_decode_array(buf, offset + pos_byte, maxlen - pos_byte, p[element].position_covariance, 64);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* position_covariance_type */
        thislen = __int8_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].position_covariance_type), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

    }
    return pos_byte;
}

int __sensor_msgs_NavSatFix_decode_array_cleanup(sensor_msgs_NavSatFix* p, uint32_t elements)
{
    uint32_t element;
    for (element = 0; element < elements; ++element) {

        __std_msgs_Header_decode_array_cleanup(&(p[element].header), 1);

        __sensor_msgs_NavSatStatus_decode_array_cleanup(&(p[element].status), 1);

        __double_decode_array_cleanup(&(p[element].latitude), 1);

        __double_decode_array_cleanup(&(p[element].longitude), 1);

        __double_decode_array_cleanup(&(p[element].altitude), 1);

        __double_decode_array_cleanup(p[element].position_covariance, 64);

        __int8_t_decode_array_cleanup(&(p[element].position_covariance_type), 1);

    }
    return 0;
}

int sensor_msgs_NavSatFix_decode(const void* buf, uint32_t offset, uint32_t maxlen, sensor_msgs_NavSatFix* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_msgs_NavSatFix_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __sensor_msgs_NavSatFix_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sensor_msgs_NavSatFix_decode_cleanup(sensor_msgs_NavSatFix* p)
{
    return __sensor_msgs_NavSatFix_decode_array_cleanup(p, 1);
}

uint32_t __sensor_msgs_NavSatFix_clone_array(const sensor_msgs_NavSatFix* p, sensor_msgs_NavSatFix* q, uint32_t elements)
{
    uint32_t n = 0, element;
    for (element = 0; element < elements; ++element) {

        n += __std_msgs_Header_clone_array(&(p[element].header), &(q[element].header), 1);

        n += __sensor_msgs_NavSatStatus_clone_array(&(p[element].status), &(q[element].status), 1);

        n += __double_clone_array(&(p[element].latitude), &(q[element].latitude), 1);

        n += __double_clone_array(&(p[element].longitude), &(q[element].longitude), 1);

        n += __double_clone_array(&(p[element].altitude), &(q[element].altitude), 1);

        n += __double_clone_array(p[element].position_covariance, q[element].position_covariance, 64);

        n += __int8_t_clone_array(&(p[element].position_covariance_type), &(q[element].position_covariance_type), 1);

    }
    return n;
}

sensor_msgs_NavSatFix* sensor_msgs_NavSatFix_copy(const sensor_msgs_NavSatFix* p)
{
    sensor_msgs_NavSatFix* q = (sensor_msgs_NavSatFix*) malloc(sizeof(sensor_msgs_NavSatFix));
    __sensor_msgs_NavSatFix_clone_array(p, q, 1);
    return q;
}

void sensor_msgs_NavSatFix_destroy(sensor_msgs_NavSatFix* p)
{
    __sensor_msgs_NavSatFix_decode_array_cleanup(p, 1);
    free(p);
}

int sensor_msgs_NavSatFix_publish(zcm_t* zcm, const char* channel, const sensor_msgs_NavSatFix* p)
{
      uint32_t max_data_size = sensor_msgs_NavSatFix_encoded_size (p);
      uint8_t* buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = sensor_msgs_NavSatFix_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = zcm_publish (zcm, channel, buf, (uint32_t)data_size);
      free (buf);
      return status;
}

struct _sensor_msgs_NavSatFix_subscription_t {
    sensor_msgs_NavSatFix_handler_t user_handler;
    void* userdata;
    zcm_sub_t* z_sub;
};
static
void sensor_msgs_NavSatFix_handler_stub (const zcm_recv_buf_t* rbuf,
                            const char* channel, void* userdata)
{
    int status;
    sensor_msgs_NavSatFix p;
    memset(&p, 0, sizeof(sensor_msgs_NavSatFix));
    status = sensor_msgs_NavSatFix_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr, "error %d decoding sensor_msgs_NavSatFix!!!\n", status);
        #endif
        return;
    }

    sensor_msgs_NavSatFix_subscription_t* h = (sensor_msgs_NavSatFix_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    sensor_msgs_NavSatFix_decode_cleanup (&p);
}

sensor_msgs_NavSatFix_subscription_t* sensor_msgs_NavSatFix_subscribe (zcm_t* zcm,
                    const char* channel,
                    sensor_msgs_NavSatFix_handler_t f, void* userdata)
{
    sensor_msgs_NavSatFix_subscription_t* n = (sensor_msgs_NavSatFix_subscription_t*)
                       malloc(sizeof(sensor_msgs_NavSatFix_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->z_sub = zcm_subscribe (zcm, channel,
                              sensor_msgs_NavSatFix_handler_stub, n);
    if (n->z_sub == NULL) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr,"couldn't reg sensor_msgs_NavSatFix ZCM handler!\n");
        #endif
        free (n);
        return NULL;
    }
    return n;
}

int sensor_msgs_NavSatFix_unsubscribe(zcm_t* zcm, sensor_msgs_NavSatFix_subscription_t* hid)
{
    int status = zcm_unsubscribe (zcm, hid->z_sub);
    if (0 != status) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr,
           "couldn't unsubscribe sensor_msgs_NavSatFix_handler %p!\n", hid);
        #endif
        return -1;
    }
    free (hid);
    return 0;
}

