#include "MsgInfo.hpp"
#include "Debug.hpp"

#include <limits>

MsgInfo::MsgInfo(TypeDb& db, const char *channel) : db(db), channel(channel) {}

MsgInfo::~MsgInfo()
{
    if (last_msg) {
        if (metadata)
            metadata->info->decode_cleanup(last_msg);
        free(last_msg);
        last_msg = NULL;
    }
}

size_t MsgInfo::getViewDepth()
{
    return disp_state.recur_table.size();
}

void MsgInfo::incViewDepth(size_t viewid)
{
    disp_state.recur_table.push_back(viewid);
}

void MsgInfo::decViewDepth()
{
    assert(disp_state.recur_table.size() != 0);
    disp_state.recur_table.pop_back();
}

void MsgInfo::display()
{
    const char *name = NULL;
    i64 hash = 0;
    if (metadata) {
        name = metadata->name.c_str();
        hash = metadata->info->get_hash();
    }

    printf("         Decoding %s (%s) %" PRIi64 ":\n", channel.c_str(), name, hash);

    if (last_msg)
        msg_display(db, *metadata, last_msg,  disp_state);
}

void MsgInfo::ensureHash(i64 h)
{
    if (hash == h)
        return;

    // if this not the first message, warn user
    if (hash != 0) {
        DEBUG(1, "WRN: hash changed, searching for new zcmtype on channel %s\n", channel.c_str());
    }

    // cleanup old memory if needed
    if (metadata && last_msg) {
        metadata->info->decode_cleanup(last_msg);
        free(last_msg);
        last_msg = NULL;
    }

    hash = h;
    metadata = db.getByHash(h);
    if (metadata == NULL) {
        DEBUG(1, "WRN: failed to find zcmtype for hash: 0x%" PRIx64 "\n", hash);
        return;
    }
}

u64 MsgInfo::latestUtime()
{
    return queue.last().first;
}

u64 MsgInfo::oldestUtime()
{
    return queue.first().first;
}

void MsgInfo::removeOld()
{
    u64 oldest_allowed = TimeUtil::utime() - QUEUE_PERIOD;
    while (!queue.isEmpty() && queue.first().first < oldest_allowed)
        queue.dequeue();
}

void MsgInfo::addMessage(u64 utime, const zcm_recv_buf_t *rbuf)
{
    if (queue.isFull())
        queue.dequeue();
    queue.enqueue({ utime, rbuf->data_size });

    num_msgs++;

    /* decode the data */
    i64 hash = 0;
    __int64_t_decode_array(rbuf->data, 0, rbuf->data_size, &hash, 1);
    ensureHash(hash);

    if (metadata) {
        // do we need to allocate memory for 'last_msg' ?
        if (!last_msg) {
            size_t sz = metadata->info->struct_size();
            last_msg = malloc(sz);
        } else {
            metadata->info->decode_cleanup(last_msg);
        }

        // actually decode it
        metadata->info->decode(rbuf->data, 0, rbuf->data_size, last_msg);

        DEBUG(1, "INFO: successful decode on %s\n", channel.c_str());
    }
}

float MsgInfo::getHertz()
{
    removeOld();
    if (queue.isEmpty())
        return 0.0;

    int sz = queue.getSize();
    if (sz < 2) return 0.0;
    u64 oldest = oldestUtime();
    u64 latest = latestUtime();
    u64 dt = latest - oldest;
    if (dt == 0.0)
        return std::numeric_limits<float>::infinity();

    return (float) (sz - 1) / ((float) dt / 1000000.0);
}

float MsgInfo::getBandwidthBps()
{
    removeOld();
    if (queue.isEmpty())
        return 0.0;

    size_t total = 0;
    size_t i = 0;
    size_t sz = queue.getSize();
    for (; i < sz; ++i) total += queue[i].second;

    return (double)total / (double)i;
}
