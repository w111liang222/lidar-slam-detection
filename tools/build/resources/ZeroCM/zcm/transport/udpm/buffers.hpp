#pragma once

#include "udpm.hpp"
#include "mempool.hpp"

/************************* Packet Headers *******************/

struct MsgHeaderShort
{
    // Layout
  private:
    // NOTE: These are set to private only because they are in network format.
    //       Thus, they are not safe to access directly.
    u32 magic;
    u32 msg_seqno;

    // Converted data
  public:
    u32  getMagic()         { return ntohl(magic); }
    void setMagic(u32 v)    { magic = htonl(v); }
    u32  getMsgSeqno()      { return ntohl(msg_seqno); }
    void setMsgSeqno(u32 v) { msg_seqno = htonl(v); }

    // Computed data
  public:
    // Note: Channel starts after the header
    const char *getChannelPtr() { return (char*)(this+1); }
    size_t getChannelLen() { return strlen(getChannelPtr()); }

    // Note: Data starts after the channel and null
    char *getDataPtr() { return (char*)getChannelPtr() + getChannelLen() + 1; }
    size_t getDataOffset() { return sizeof(*this) + getChannelLen() + 1; }
    size_t getDataLen(size_t pktsz) { return pktsz - getDataOffset(); }
};

struct MsgHeaderLong
{
    // Layout
  // TODO: make this private
  //private:
    u32 magic;
    u32 msg_seqno;
    u32 msg_size;
    u32 fragment_offset;
    u16 fragment_no;
    u16 fragments_in_msg;

    // Converted data
  public:
    u32 getMagic()          { return ntohl(magic); }
    u32 getMsgSeqno()       { return ntohl(msg_seqno); }
    u32 getMsgSize()        { return ntohl(msg_size); }
    u32 getFragmentOffset() { return ntohl(fragment_offset); }
    u16 getFragmentNo()     { return ntohs(fragment_no); }
    u16 getFragmentsInMsg() { return ntohs(fragments_in_msg); }

    // Computed data
  public:
    u32 getFragmentSize(size_t pktsz) { return pktsz - sizeof(*this); }
    char *getDataPtr() { return (char*)(this+1); }
};

// if fragment_no == 0, then header is immediately followed by NULL-terminated
// ASCII-encoded channel name, followed by the payload data
// if fragment_no > 0, then header is immediately followed by the payload data

/******************** message buffer **********************/
struct Buffer
{
    char *data = nullptr;
    size_t size = 0;

    Buffer(){}
  private:
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

  public:
    Buffer(Buffer&& o) : data(o.data), size(o.size)
    {
        o.data = nullptr;
    }

    Buffer& operator=(Buffer&& other)
    {
        assert(this->data == nullptr &&
               "Error: Buffer MUST be deallocated before a move can occur");
        this->data = other.data;
        this->size = other.size;
        other.data = nullptr;
        return *this;
    }
};

struct Message
{
    i64               utime = 0;          // timestamp of first datagram receipt

    const char       *channel = nullptr;  // points into 'buf'
    size_t            channellen = 0;     // length of channel

    char             *data = nullptr;     // points into 'buf'
    size_t            datalen = 0;        // length of data

    // Backing store buffer that contains the actual data
    Buffer buf = {};

    Message() { }
};

struct Packet
{
    i64             utime = 0;      // timestamp of first datagram receipt
    size_t          sz = 0;         // size received

    struct sockaddr from = {};      // sender
    socklen_t       fromlen = {};

    // Backing store buffer that contains the actual data
    Buffer          buf = {};

    Packet() {}
    MsgHeaderShort *asHeaderShort() { return (MsgHeaderShort*)buf.data; }
    MsgHeaderLong  *asHeaderLong()  { return (MsgHeaderLong* )buf.data; }
};

/******************** fragment buffer **********************/
struct FragBuf
{
    i64     last_packet_utime;
    u32     msg_seqno;
    u16     fragments_remaining;

    // The channel starts at the beginning of the buffer. The data
    // follows immediately after the channel and its NULL
    size_t  channellen;
    struct sockaddr_in from;

    // Fields set by the allocator object
    Buffer buf;

    bool matchesSockaddr(struct sockaddr_in *addr);
};

/************** A pool to handle every alloc/dealloc operation on Message objects ******/
struct MessagePool
{
    MessagePool(size_t maxSize, size_t maxBuffers);
    ~MessagePool();

    // Buffer
    Buffer allocBuffer(size_t sz);
    void freeBuffer(Buffer& buf);

    // Packet
    Packet *allocPacket(size_t maxsz);
    void freePacket(Packet *p);

    // Message
    Message *allocMessage();
    Message *allocMessageEmpty();
    void freeMessage(Message *b);

    // FragBuf
    FragBuf *addFragBuf(u32 data_size);
    FragBuf *lookupFragBuf(struct sockaddr_in *key);
    void removeFragBuf(FragBuf *fbuf);

    void transferBufffer(Message *to, FragBuf *from);
    void moveBuffer(Buffer& to, Buffer& from);

  private:
    void _freeMessageBuffer(Message *b);
    void _removeFragBuf(size_t index);

  private:
    MemPool mempool;
    vector<FragBuf*> fragbufs;
    size_t maxSize;
    size_t maxBuffers;
    size_t totalSize = 0;
};
