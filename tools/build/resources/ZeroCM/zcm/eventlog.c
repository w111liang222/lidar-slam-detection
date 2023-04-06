#include "zcm/eventlog.h"
#include "zcm/util/debug.h"
#include "zcm/util/ioutils.h"
#include <assert.h>
#include <string.h>

#define MAGIC ((int32_t) 0xEDA1DA01L)

zcm_eventlog_t *zcm_eventlog_create(const char *path, const char *mode)
{
    assert(!strcmp(mode, "r") || !strcmp(mode, "w") || !strcmp(mode, "a"));
    if(*mode == 'w')
        mode = "wb";
    else if(*mode == 'r')
        mode = "rb";
    else if(*mode == 'a')
        mode = "ab";
    else
        return NULL;

    zcm_eventlog_t *l = (zcm_eventlog_t*) calloc(1, sizeof(zcm_eventlog_t));

    l->f = fopen(path, mode);
    if (!l->f) {
        free (l);
        return NULL;
    }

    l->eventcount = 0;

    return l;
}

void zcm_eventlog_destroy(zcm_eventlog_t *l)
{
    fflush(l->f);
    fclose(l->f);
    free(l);
}

FILE *zcm_eventlog_get_fileptr(zcm_eventlog_t *l)
{
    return l->f;
}

// Returns 0 on success -1 on failure
static int sync_stream(zcm_eventlog_t *l)
{
    uint32_t magic = 0;
    int r;
    do {
        r = fgetc(l->f);
        if (r < 0) return -1;
        magic = (magic << 8) | r;
    } while( (int32_t)magic != MAGIC );
    return 0;
}

static int sync_stream_backwards(zcm_eventlog_t *l)
{
    uint32_t magic = 0;
    int r;
    do {
        if (ftello (l->f) < 2) return -1;
        fseeko (l->f, -2, SEEK_CUR);
        r = fgetc(l->f);
        if (r < 0) return 1;
        magic = ((magic >> 8) & 0x00ffffff) | (((uint32_t)r << 24) & 0xff000000);
    } while( (int32_t)magic != MAGIC );
    fseeko (l->f, sizeof(uint32_t) - 1, SEEK_CUR);
    return 0;
}

static int64_t get_next_event_time(zcm_eventlog_t *l)
{
    if (sync_stream(l)) return -1;

    int64_t event_num;
    int64_t timestamp;
    if (0 != fread64(l->f, &event_num)) return -1;
    if (0 != fread64(l->f, &timestamp)) return -1;
    fseeko (l->f, -(sizeof(int64_t) * 2 + sizeof(int32_t)), SEEK_CUR);

    l->eventcount = event_num;

    return timestamp;
}

int zcm_eventlog_seek_to_timestamp(zcm_eventlog_t *l, int64_t timestamp)
{
    fseeko (l->f, 0, SEEK_END);
    off_t file_len = ftello(l->f);

    int64_t cur_time;
    double frac1 = 0;               // left bracket
    double frac2 = 1;               // right bracket
    double prev_frac = -1;
    double frac;                    // current position

    while (1) {
        frac = 0.5 * (frac1 + frac2);
        off_t offset = (off_t)(frac * file_len);
        fseeko(l->f, offset, SEEK_SET);
        cur_time = get_next_event_time(l);
        if (cur_time < 0)
            return -1;

        if ((frac > frac2) || (frac < frac1) || (frac1 >= frac2))
            break;

        double df = frac - prev_frac;
        if (df < 0) df = -df;
        if (df < 1e-12) break;

        if (cur_time == timestamp) break;

        if (cur_time < timestamp) frac1 = frac;
        else frac2 = frac;

        prev_frac = frac;
    }

    return 0;
}

static zcm_eventlog_event_t *zcm_event_read_helper(zcm_eventlog_t *l, int rewindWhenDone)
{
    zcm_eventlog_event_t *le =
        (zcm_eventlog_event_t*) calloc(1, sizeof(zcm_eventlog_event_t));

    off_t startOffset = ftello(l->f);

    if (fread64(l->f, &le->eventnum)) {
        free(le);
        le = NULL;
        goto done;
    }

    if (fread64(l->f, &le->timestamp)) {
        free(le);
        le = NULL;
        goto done;
    }

    if (fread32(l->f, &le->channellen)) {
        free(le);
        le = NULL;
        goto done;
    }

    if (fread32(l->f, &le->datalen)) {
        free(le);
        le = NULL;
        goto done;
    }

    // Sanity check the channel length and data length
    if (le->channellen <= 0 || le->channellen >= 1000) {
        ZCM_DEBUG("Log event has invalid channel length: %d", le->channellen);
        free(le);
        le = NULL;
        goto done;
    }
    if (le->datalen < 0) {
        ZCM_DEBUG("Log event has invalid data length: %d", le->datalen);
        free(le);
        le = NULL;
        goto done;
    }

    le->channel = (char *) calloc(1, le->channellen+1);
    size_t readRet = fread(le->channel, 1, le->channellen, l->f);
    if (readRet != (size_t) le->channellen) {
        free(le->channel);
        free(le);
        le = NULL;
        goto done;
    }

    le->data = calloc(1, le->datalen+1);
    readRet = fread(le->data, 1, le->datalen, l->f);
    if (readRet != (size_t) le->datalen) {
        free(le->channel);
        free(le->data);
        free(le);
        le = NULL;
        goto done;
    }

    int32_t next_magic;
    if (0 == fread32(l->f, &next_magic)) {
        if (next_magic != MAGIC) {
            ZCM_DEBUG("Invalid header after log data");
            free(le->channel);
            free(le->data);
            free(le);
            le = NULL;
            goto done;
        }
        fseeko (l->f, -4, SEEK_CUR);
    }

done:
    if (rewindWhenDone) {
        off_t numRead = ftello (l->f) - startOffset;
        fseeko (l->f, -(off_t)(numRead + 4), SEEK_CUR);
    } else if (le == NULL) {
        off_t numRead = ftello (l->f) - startOffset;
        fseeko (l->f, -(off_t)numRead, SEEK_CUR);
    }
    return le;
}

zcm_eventlog_event_t *zcm_eventlog_read_next_event(zcm_eventlog_t *l)
{
    zcm_eventlog_event_t* ret = NULL;
    while (ret == NULL) {
        if (sync_stream(l)) break;
        ret = zcm_event_read_helper(l, 0);
    }
    return ret;
}

zcm_eventlog_event_t *zcm_eventlog_read_prev_event(zcm_eventlog_t *l)
{
    zcm_eventlog_event_t* ret = NULL;
    while (ret == NULL) {
        if (sync_stream_backwards(l) < 0) break;
        ret = zcm_event_read_helper(l, 1);
    }
    return ret;
}

zcm_eventlog_event_t *zcm_eventlog_read_event_at_offset(zcm_eventlog_t *l, off_t offset)
{
    fseeko(l->f, offset, SEEK_SET);
    zcm_eventlog_event_t* ret = NULL;
    while (ret == NULL) {
        if (sync_stream(l)) break;
        ret = zcm_event_read_helper(l, 0);
    }
    return ret;
}

void zcm_eventlog_free_event(zcm_eventlog_event_t *le)
{
    if (le->data) free(le->data);
    if (le->channel) free(le->channel);
    memset(le,0,sizeof(zcm_eventlog_event_t));
    free(le);
}

int zcm_eventlog_write_event(zcm_eventlog_t *l, const zcm_eventlog_event_t *le)
{
    if (0 != fwrite32(l->f, MAGIC)) return -1;

    if (0 != fwrite64(l->f, l->eventcount)) return -1;

    if (0 != fwrite64(l->f, le->timestamp)) return -1;
    if (0 != fwrite32(l->f, le->channellen)) return -1;
    if (0 != fwrite32(l->f, le->datalen)) return -1;

    if (le->channellen != fwrite(le->channel, 1, le->channellen, l->f))
        return -1;
    if (le->datalen != fwrite(le->data, 1, le->datalen, l->f))
        return -1;

    l->eventcount++;

    return 0;
}
