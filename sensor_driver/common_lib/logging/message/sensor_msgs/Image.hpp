/** THIS IS AN AUTOMATICALLY GENERATED FILE.
 *  DO NOT MODIFY BY HAND!!
 *
 *  Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __sensor_msgs_Image_hpp__
#define __sensor_msgs_Image_hpp__

#include <string>
#include <vector>
#include "std_msgs/Header.hpp"


namespace sensor_msgs {
class Image
{
    public:
        std_msgs::Header header;

        int32_t    height;

        int32_t    width;

        std::string encoding;

        int8_t     is_bigendian;

        int32_t    step;

        int64_t    size;

        std::vector< uint8_t > data;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~Image() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to read while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "Image"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int Image::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int Image::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t Image::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t Image::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* Image::getTypeName()
{
    return "Image";
}

int Image::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos_byte = 0;
    int thislen;

    thislen = this->header._encodeNoHash(buf, offset + pos_byte, maxlen - pos_byte);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->height, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->width, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    char* encoding_cstr = (char*) this->encoding.c_str();
    thislen = __string_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &encoding_cstr, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int8_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->is_bigendian, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->step, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int64_t_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->size, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    if(this->size > 0) {
        thislen = __byte_encode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->data[0], this->size);
        if(thislen < 0) return thislen; else pos_byte += thislen;
    }

    return pos_byte;
}

int Image::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos_byte = 0;
    int thislen;

    thislen = this->header._decodeNoHash(buf, offset + pos_byte, maxlen - pos_byte);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->height, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->width, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    int32_t __encoding_len__;
    thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &__encoding_len__, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;
    if((uint32_t)__encoding_len__ > maxlen - pos_byte) return -1;
    this->encoding.assign(((const char*)buf) + offset + pos_byte, __encoding_len__ - ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS);
    pos_byte += __encoding_len__;

    thislen = __int8_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->is_bigendian, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int32_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->step, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    thislen = __int64_t_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->size, 1);
    if(thislen < 0) return thislen; else pos_byte += thislen;

    if(this->size > 0) {
        this->data.resize(this->size);
        thislen = __byte_decode_array(buf, offset + pos_byte, maxlen - pos_byte, &this->data[0], this->size);
        if(thislen < 0) return thislen; else pos_byte += thislen;
    }

    return pos_byte;
}

uint32_t Image::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += this->header._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += this->encoding.size() + ZCM_CORETYPES_INT32_NUM_BYTES_ON_BUS + ZCM_CORETYPES_INT8_NUM_BYTES_ON_BUS;
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __byte_encoded_array_size(NULL, this->size);
    return enc_size;
}

uint64_t Image::_computeHash(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == Image::getHash)
            return 0;
    const __zcm_hash_ptr cp = { p, (void*)Image::getHash };

    uint64_t hash = (uint64_t)0x77873de2583fe560LL +
         std_msgs::Header::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
