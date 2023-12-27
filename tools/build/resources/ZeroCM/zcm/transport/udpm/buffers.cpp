#include "buffers.hpp"

static bool sockaddrEqual(struct sockaddr_in *a, struct sockaddr_in *b)
{
    return a->sin_addr.s_addr == b->sin_addr.s_addr &&
           a->sin_port        == b->sin_port &&
           a->sin_family      == b->sin_family;
}

bool FragBuf::matchesSockaddr(struct sockaddr_in *addr)
{
    return sockaddrEqual(&from, addr);
}

MessagePool::MessagePool(size_t maxSize, size_t maxBuffers)
    : maxSize(maxSize), maxBuffers(maxBuffers)
{
}

MessagePool::~MessagePool()
{
}

Buffer MessagePool::allocBuffer(size_t sz)
{
    Buffer buf;
    buf.data = mempool.alloc(sz);
    buf.size = sz;
    return buf;
}

void MessagePool::freeBuffer(Buffer& buf)
{
    if (buf.data) {
        mempool.free(buf.data, buf.size);
        buf.data = NULL;
    }
}

void MessagePool::moveBuffer(Buffer& to, Buffer& from)
{
    freeBuffer(to);
    to.data = from.data;
    to.size = from.size;
    from.data = NULL;
}

Packet *MessagePool::allocPacket(size_t maxsz)
{
    Packet *p = new (mempool.alloc<Packet>()) Packet{};
    p->buf = this->allocBuffer(maxsz);
    return p;
}

void MessagePool::freePacket(Packet *p)
{
    freeBuffer(p->buf);
    mempool.free(p);
}

Message *MessagePool::allocMessage()
{
    Message *zcmb = new (mempool.alloc<Message>()) Message{};
    zcmb->buf = this->allocBuffer(ZCM_MAX_UNFRAGMENTED_PACKET_SIZE);
    return zcmb;
}

Message *MessagePool::allocMessageEmpty()
{
    return new (mempool.alloc<Message>()) Message{};
}

void MessagePool::_freeMessageBuffer(Message *b)
{
    freeBuffer(b->buf);
}

void MessagePool::freeMessage(Message *b)
{
    _freeMessageBuffer(b);
    mempool.free(b);
}


FragBuf *MessagePool::addFragBuf(u32 data_size)
{
    FragBuf *fbuf = new (mempool.alloc<FragBuf>()) FragBuf{};
    fbuf->buf = this->allocBuffer(data_size);

    while (totalSize > maxSize || fragbufs.size() > maxBuffers) {
        // find and remove the least recently updated fragment buffer
        int idx = -1;
        FragBuf *eldest = nullptr;
        for (size_t i = 0; i < fragbufs.size(); i++) {
            FragBuf *f = fragbufs[i];
            if (idx == -1 || f->last_packet_utime < eldest->last_packet_utime) {
                idx = (int)i;
                eldest = f;
            }
        }
        if (eldest) {
            _removeFragBuf((size_t)idx);
            // XXX Need to free the removed FragBuf*
        }
    }

    fragbufs.push_back(fbuf);
    totalSize += data_size;

    return fbuf;
}

FragBuf *MessagePool::lookupFragBuf(struct sockaddr_in *key)
{
    for (auto& elt : fragbufs)
        if (elt->matchesSockaddr(key))
            return elt;
    return nullptr;
}

void MessagePool::_removeFragBuf(size_t index)
{
    assert(index < fragbufs.size());

    // Update the total_size of the fragment buffers
    FragBuf *fbuf = fragbufs[index];
    totalSize -= fbuf->buf.size;

    // delete old element, move last element to this slot, and shrink by 1
    size_t lastIdx = fragbufs.size()-1;
    fragbufs[index] = fragbufs[lastIdx];
    fragbufs.pop_back();

    this->freeBuffer(fbuf->buf);
    mempool.free(fbuf);
}

void MessagePool::removeFragBuf(FragBuf *fbuf)
{
    // NOTE: this is kinda slow...
    // Search for the fragbuf index
    for (size_t idx = 0; idx < fragbufs.size(); idx++)
        if (fragbufs[idx] == fbuf)
            return this->_removeFragBuf(idx);

    // Did not find
    assert(0 && "Tried to remove invalid fragbuf");
}

void MessagePool::transferBufffer(Message *to, FragBuf *from)
{
    _freeMessageBuffer(to);
    to->buf = std::move(from->buf);
}

/************************* Linux Specific Functions *******************/
// #ifdef __linux__
// void linux_check_routing_table(struct in_addr zcm_mcaddr);
// #endif


// #ifdef __linux__
// static inline int _parse_inaddr(const char *addr_str, struct in_addr *addr)
// {
//     char buf[] = {
//         '0', 'x',
//         addr_str[6], addr_str[7],
//         addr_str[4], addr_str[5],
//         addr_str[2], addr_str[3],
//         addr_str[0], addr_str[1],
//         0
//     };
//     return inet_aton(buf, addr);
// }

// void
// linux_check_routing_table(struct in_addr zcm_mcaddr)
// {
//     FILE *fp = fopen("/proc/net/route", "r");
//     if(!fp) {
//         perror("Unable to open routing table (fopen)");
//         goto show_route_cmds;
//     }

//     // read and ignore the first line of the routing table file
//     char buf[1024];
//     if(!fgets(buf, sizeof(buf), fp)) {
//         perror("Unable to read routing table (fgets)");
//         fclose(fp);
//         goto show_route_cmds;
//     }

//     // each line is a routing table entry
//     while(!feof(fp)) {
//         memset(buf, 0, sizeof(buf));
//         if(!fgets(buf, sizeof(buf)-1, fp))
//             break;
//         gchar **words = g_strsplit(buf, "\t", 0);

//         // each line should have 11 words
//         int nwords;
//         for(nwords=0; words[nwords] != NULL; nwords++);
//         if(nwords != 11) {
//             g_strfreev(words);
//             fclose(fp);
//             fprintf(stderr, "Unable to parse routing table!  Strange format.");
//             goto show_route_cmds;
//         }

//         // destination is 2nd word, netmask is 8th word
//         struct in_addr dest, mask;
//         if(!_parse_inaddr(words[1], &dest) || !_parse_inaddr(words[7], &mask)) {
//             fprintf(stderr, "Unable to parse routing table!");
//             g_strfreev(words);
//             fclose(fp);
//             goto show_route_cmds;
//         }
//         g_strfreev(words);

// //        fprintf(stderr, "checking route (%s/%X)\n", inet_ntoa(dest),
// //                ntohl(mask.s_addr));

//         // does this routing table entry match the ZCM URL?
//         if((zcm_mcaddr.s_addr & mask.s_addr) == (dest.s_addr & mask.s_addr)) {
//             // yes, so there is a valid multicast route
//             fclose(fp);
//             return;
//         }
//     }
//     fclose(fp);

// show_route_cmds:
//     // if we get here, then none of the routing table entries matched the
//     // ZCM destination URL.
//     fprintf(stderr,
// "\nNo route to %s\n\n"
// "ZCM requires a valid multicast route.  If this is a Linux computer and is\n"
// "simply not connected to a network, the following commands are usually\n"
// "sufficient as a temporary solution:\n"
// "\n"
// "   sudo ifconfig lo multicast\n"
// "   sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo\n"
// "\n"
// "For more information, visit:\n"
// "   http://zcm-proj.github.io/multicast_setup.html\n\n",
// inet_ntoa(zcm_mcaddr));
// }
// #endif
