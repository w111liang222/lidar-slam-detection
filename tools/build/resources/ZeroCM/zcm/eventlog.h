#ifndef _ZCM_EVENTLOG_H
#define _ZCM_EVENTLOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

/* Note: some language bindings depend on the specific memory layout
 *       of ZCM structures. If you change these, be sure to update
 *       language bindings to match. */

typedef struct _zcm_eventlog_event_t zcm_eventlog_event_t;
struct _zcm_eventlog_event_t
{
    int64_t  eventnum;   /* populated by write_event */
    int64_t  timestamp;
    int32_t  channellen;
    int32_t  datalen;
    char*    channel;
    uint8_t* data;
};

typedef struct _zcm_eventlog_t zcm_eventlog_t;
struct _zcm_eventlog_t
{
    FILE* f;
    int64_t eventcount;
};

/**** Methods for creation/deletion ****/
zcm_eventlog_t* zcm_eventlog_create(const char* path, const char* mode);
void zcm_eventlog_destroy(zcm_eventlog_t* eventlog);


/**** Methods for general operations ****/
FILE* zcm_eventlog_get_fileptr(zcm_eventlog_t* eventlog);
int zcm_eventlog_seek_to_timestamp(zcm_eventlog_t* eventlog, int64_t ts);


/**** Methods for read/write ****/
// NOTE: The returned zcm_eventlog_event_t must be freed by zcm_eventlog_free_event()
zcm_eventlog_event_t* zcm_eventlog_read_next_event(zcm_eventlog_t* eventlog);
zcm_eventlog_event_t* zcm_eventlog_read_prev_event(zcm_eventlog_t* eventlog);
zcm_eventlog_event_t* zcm_eventlog_read_event_at_offset(zcm_eventlog_t* eventlog, off_t offset);
void zcm_eventlog_free_event(zcm_eventlog_event_t* event);
int zcm_eventlog_write_event(zcm_eventlog_t* eventlog, const zcm_eventlog_event_t* event);


#ifdef __cplusplus
}
#endif

#endif /* _ZCM_EVENTLOG_H */
