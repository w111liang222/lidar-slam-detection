// THIS IS AN AUTOMATICALLY GENERATED FILE.
// DO NOT MODIFY BY HAND!!
//
// Generated by zcm-gen

#include <string.h>
#ifndef ZCM_EMBEDDED
#include <stdio.h>
#endif
#include "sensor_msgs/sensor_msgs_Image.h"

static int __sensor_msgs_Image_hash_computed = 0;
static uint64_t __sensor_msgs_Image_hash;

uint64_t __sensor_msgs_Image_hash_recursive(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __sensor_msgs_Image_get_hash)
            return 0;

    __zcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__sensor_msgs_Image_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x77873de2583fe560LL
         + __std_msgs_Header_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __string_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int32_t_hash_recursive(&cp)
         + __int64_t_hash_recursive(&cp)
         + __byte_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __sensor_msgs_Image_get_hash(void)
{
    if (!__sensor_msgs_Image_hash_computed) {
        __sensor_msgs_Image_hash = (int64_t)__sensor_msgs_Image_hash_recursive(NULL);
        __sensor_msgs_Image_hash_computed = 1;
    }

    return __sensor_msgs_Image_hash;
}

int __sensor_msgs_Image_encode_array(void* buf, uint32_t offset, uint32_t maxlen, const sensor_msgs_Image* p, uint32_t elements)
{
    uint32_t pos_byte = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        /* header */
        thislen = __std_msgs_Header_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].header), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* height */
        thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].height), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* width */
        thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].width), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* encoding */
        thislen = __string_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].encoding), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* is_bigendian */
        thislen = __int8_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].is_bigendian), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* step */
        thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].step), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* size */
        thislen = __int64_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].size), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* data */
        thislen = __byte_encode_array(buf, offset + pos_byte, maxlen - pos_byte, p[element].data, p[element].size);
        if (thislen < 0) return thislen; else pos_byte += thislen;

    }
    return pos_byte;
}

int sensor_msgs_Image_encode(void* buf, uint32_t offset, uint32_t maxlen, const sensor_msgs_Image* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_msgs_Image_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __sensor_msgs_Image_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t __sensor_msgs_Image_encoded_array_size(const sensor_msgs_Image* p, uint32_t elements)
{
    uint32_t size = 0, element;
    for (element = 0; element < elements; ++element) {

        size += __std_msgs_Header_encoded_array_size(&(p[element].header), 1); // header

        size += __int32_t_encoded_array_size(&(p[element].height), 1); // height

        size += __int32_t_encoded_array_size(&(p[element].width), 1); // width

        size += __string_encoded_array_size(&(p[element].encoding), 1); // encoding

        size += __int8_t_encoded_array_size(&(p[element].is_bigendian), 1); // is_bigendian

        size += __int32_t_encoded_array_size(&(p[element].step), 1); // step

        size += __int64_t_encoded_array_size(&(p[element].size), 1); // size

        size += __byte_encoded_array_size(p[element].data, p[element].size); // data

    }
    return size;
}

uint32_t sensor_msgs_Image_encoded_size(const sensor_msgs_Image* p)
{
    return 8 + __sensor_msgs_Image_encoded_array_size(p, 1);
}

uint32_t sensor_msgs_Image_struct_size(void)
{
    return sizeof(sensor_msgs_Image);
}

uint32_t sensor_msgs_Image_num_fields(void)
{
    return 8;
}

int sensor_msgs_Image_get_field(const sensor_msgs_Image* p, uint32_t i, zcm_field_t* f)
{
    if (i >= sensor_msgs_Image_num_fields())
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
            f->name = "height";
            f->type = ZCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void*) &p->height;
            return 0;
        }
        
        case 2: {
            f->name = "width";
            f->type = ZCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void*) &p->width;
            return 0;
        }
        
        case 3: {
            f->name = "encoding";
            f->type = ZCM_FIELD_STRING;
            f->typestr = "string";
            f->num_dim = 0;
            f->data = (void*) &p->encoding;
            return 0;
        }
        
        case 4: {
            f->name = "is_bigendian";
            f->type = ZCM_FIELD_INT8_T;
            f->typestr = "int8_t";
            f->num_dim = 0;
            f->data = (void*) &p->is_bigendian;
            return 0;
        }
        
        case 5: {
            f->name = "step";
            f->type = ZCM_FIELD_INT32_T;
            f->typestr = "int32_t";
            f->num_dim = 0;
            f->data = (void*) &p->step;
            return 0;
        }
        
        case 6: {
            f->name = "size";
            f->type = ZCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void*) &p->size;
            return 0;
        }
        
        case 7: {
            f->name = "data";
            f->type = ZCM_FIELD_BYTE;
            f->typestr = "byte";
            f->num_dim = 1;
            f->dim_size[0] = p->size;
            f->dim_is_variable[0] = 1;
            f->data = (void*) &p->data;
            return 0;
        }
        
        default:
            return 1;
    }
}

const zcm_type_info_t* sensor_msgs_Image_get_type_info(void)
{
    static int init = 0;
    static zcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (zcm_encode_t) sensor_msgs_Image_encode;
        typeinfo.decode         = (zcm_decode_t) sensor_msgs_Image_decode;
        typeinfo.decode_cleanup = (zcm_decode_cleanup_t) sensor_msgs_Image_decode_cleanup;
        typeinfo.encoded_size   = (zcm_encoded_size_t) sensor_msgs_Image_encoded_size;
        typeinfo.struct_size    = (zcm_struct_size_t)  sensor_msgs_Image_struct_size;
        typeinfo.num_fields     = (zcm_num_fields_t) sensor_msgs_Image_num_fields;
        typeinfo.get_field      = (zcm_get_field_t) sensor_msgs_Image_get_field;
        typeinfo.get_hash       = (zcm_get_hash_t) __sensor_msgs_Image_get_hash;
    }
    
    return &typeinfo;
}

int __sensor_msgs_Image_decode_array(const void* buf, uint32_t offset, uint32_t maxlen, sensor_msgs_Image* p, uint32_t elements)
{
    uint32_t pos_byte = 0, element;
    int thislen;

    for (element = 0; element < elements; ++element) {

        /* header */
        thislen = __std_msgs_Header_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].header), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* height */
        thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].height), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* width */
        thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].width), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* encoding */
        thislen = __string_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].encoding), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* is_bigendian */
        thislen = __int8_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].is_bigendian), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* step */
        thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].step), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* size */
        thislen = __int64_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &(p[element].size), 1);
        if (thislen < 0) return thislen; else pos_byte += thislen;

        /* data */
        p[element].data = (uint8_t*) zcm_malloc(sizeof(uint8_t) * p[element].size);
        thislen = __byte_decode_array(buf, offset + pos_byte, maxlen - pos_byte, p[element].data, p[element].size);
        if (thislen < 0) return thislen; else pos_byte += thislen;

    }
    return pos_byte;
}

int __sensor_msgs_Image_decode_array_cleanup(sensor_msgs_Image* p, uint32_t elements)
{
    uint32_t element;
    for (element = 0; element < elements; ++element) {

        __std_msgs_Header_decode_array_cleanup(&(p[element].header), 1);

        __int32_t_decode_array_cleanup(&(p[element].height), 1);

        __int32_t_decode_array_cleanup(&(p[element].width), 1);

        __string_decode_array_cleanup(&(p[element].encoding), 1);

        __int8_t_decode_array_cleanup(&(p[element].is_bigendian), 1);

        __int32_t_decode_array_cleanup(&(p[element].step), 1);

        __int64_t_decode_array_cleanup(&(p[element].size), 1);

        __byte_decode_array_cleanup(p[element].data, p[element].size);
        if (p[element].data) free(p[element].data);

    }
    return 0;
}

int sensor_msgs_Image_decode(const void* buf, uint32_t offset, uint32_t maxlen, sensor_msgs_Image* p)
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = __sensor_msgs_Image_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __sensor_msgs_Image_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int sensor_msgs_Image_decode_cleanup(sensor_msgs_Image* p)
{
    return __sensor_msgs_Image_decode_array_cleanup(p, 1);
}

uint32_t __sensor_msgs_Image_clone_array(const sensor_msgs_Image* p, sensor_msgs_Image* q, uint32_t elements)
{
    uint32_t n = 0, element;
    for (element = 0; element < elements; ++element) {

        n += __std_msgs_Header_clone_array(&(p[element].header), &(q[element].header), 1);

        n += __int32_t_clone_array(&(p[element].height), &(q[element].height), 1);

        n += __int32_t_clone_array(&(p[element].width), &(q[element].width), 1);

        n += __string_clone_array(&(p[element].encoding), &(q[element].encoding), 1);

        n += __int8_t_clone_array(&(p[element].is_bigendian), &(q[element].is_bigendian), 1);

        n += __int32_t_clone_array(&(p[element].step), &(q[element].step), 1);

        n += __int64_t_clone_array(&(p[element].size), &(q[element].size), 1);

        q[element].data = (uint8_t*) zcm_malloc(sizeof(uint8_t) * q[element].size);
        n += __byte_clone_array(p[element].data, q[element].data, p[element].size);

    }
    return n;
}

sensor_msgs_Image* sensor_msgs_Image_copy(const sensor_msgs_Image* p)
{
    sensor_msgs_Image* q = (sensor_msgs_Image*) malloc(sizeof(sensor_msgs_Image));
    __sensor_msgs_Image_clone_array(p, q, 1);
    return q;
}

void sensor_msgs_Image_destroy(sensor_msgs_Image* p)
{
    __sensor_msgs_Image_decode_array_cleanup(p, 1);
    free(p);
}

int sensor_msgs_Image_publish(zcm_t* zcm, const char* channel, const sensor_msgs_Image* p)
{
      uint32_t max_data_size = sensor_msgs_Image_encoded_size (p);
      uint8_t* buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = sensor_msgs_Image_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = zcm_publish (zcm, channel, buf, (uint32_t)data_size);
      free (buf);
      return status;
}

struct _sensor_msgs_Image_subscription_t {
    sensor_msgs_Image_handler_t user_handler;
    void* userdata;
    zcm_sub_t* z_sub;
};
static
void sensor_msgs_Image_handler_stub (const zcm_recv_buf_t* rbuf,
                            const char* channel, void* userdata)
{
    int status;
    sensor_msgs_Image p;
    memset(&p, 0, sizeof(sensor_msgs_Image));
    status = sensor_msgs_Image_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr, "error %d decoding sensor_msgs_Image!!!\n", status);
        #endif
        return;
    }

    sensor_msgs_Image_subscription_t* h = (sensor_msgs_Image_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    sensor_msgs_Image_decode_cleanup (&p);
}

sensor_msgs_Image_subscription_t* sensor_msgs_Image_subscribe (zcm_t* zcm,
                    const char* channel,
                    sensor_msgs_Image_handler_t f, void* userdata)
{
    sensor_msgs_Image_subscription_t* n = (sensor_msgs_Image_subscription_t*)
                       malloc(sizeof(sensor_msgs_Image_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->z_sub = zcm_subscribe (zcm, channel,
                              sensor_msgs_Image_handler_stub, n);
    if (n->z_sub == NULL) {
        #ifndef ZCM_EMBEDDED
        fprintf (stderr,"couldn't reg sensor_msgs_Image ZCM handler!\n");
        #endif
        free (n);
        return NULL;
    }
    return n;
}

int sensor_msgs_Image_unsubscribe(zcm_t* zcm, sensor_msgs_Image_subscription_t* hid)
{
    int status = zcm_unsubscribe (zcm, hid->z_sub);
    if (0 != status) {
        #ifndef ZCM_EMBEDDED
        fprintf(stderr,
           "couldn't unsubscribe sensor_msgs_Image_handler %p!\n", hid);
        #endif
        return -1;
    }
    free (hid);
    return 0;
}

