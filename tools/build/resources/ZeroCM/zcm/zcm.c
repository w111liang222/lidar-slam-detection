
/*******************************************************************************
 * ZCM Core API
 *
 *     This file implements the core ZCM API for both blocking and non-blocking modes
 *     The code is a bit hairy here because it must be C89 and be able to conditionally
 *     build only non-blocking. These requirements are needed to support embedded.
 *
 *******************************************************************************/

#include "zcm/zcm.h"
#include "zcm/zcm_private.h"
#include "zcm/nonblocking.h"

#ifndef ZCM_EMBEDDED
#include <stdlib.h>
#include <string.h>

# include "zcm/blocking.h"
# include "zcm/transport_registrar.h"
# include "zcm/util/debug.h"
#else
/** Note: here's to hoping that variadic macros are "portable enough" **/
# define ZCM_DEBUG(...)
#endif

#ifndef ZCM_EMBEDDED
int zcm_retcode_name_to_enum(const char* zcm_retcode_name)
{
    #define X(n, v, s) if (strcmp(#n, zcm_retcode_name) == 0) return v;
    ZCM_RETURN_CODES
    #undef X
    return ZCM_NUM_RETURN_CODES;
}
#endif

#ifndef ZCM_EMBEDDED
zcm_t* zcm_create(const char* url)
{
    zcm_t* z = NULL;
    ZCM_ASSERT(zcm_try_create(&z, url) == ZCM_EOK);
    ZCM_ASSERT(z);
    return z;
}

int zcm_try_create(zcm_t** z, const char* url)
{
    *z = malloc(sizeof(zcm_t));
    if (!*z) return ZCM_EMEMORY;
    int ret = zcm_init(*z, url);
    if (ret != ZCM_EOK) {
        free(*z);
        *z = NULL;
        return ret;
    }
    return ZCM_EOK;
}
#endif

zcm_t* zcm_create_from_trans(zcm_trans_t* zt)
{
    zcm_t* z = NULL;
    ZCM_ASSERT(zcm_try_create_from_trans(&z, zt) == ZCM_EOK);
    ZCM_ASSERT(z);
    return z;
}

int zcm_try_create_from_trans(zcm_t** z, zcm_trans_t* zt)
{
    *z = malloc(sizeof(zcm_t));
    if (!*z) return ZCM_EMEMORY;
    int ret = zcm_init_from_trans(*z, zt);
    if (ret != ZCM_EOK) {
        free(*z);
        *z = NULL;
        return ret;
    }
    return ZCM_EOK;
}

void zcm_destroy(zcm_t* zcm)
{
    zcm_cleanup(zcm);
    free(zcm);
}

#ifndef ZCM_EMBEDDED
int zcm_init(zcm_t* zcm, const char* url)
{
    /* If we have no url, try to use the env var */
    if (!url || url[0] == '\0') {
        url = getenv("ZCM_DEFAULT_URL");
        if (!url) {
            fprintf(stderr, "Please specify zcm url when creating zcm or "
                            "set environment variable ZCM_DEFAULT_URL\n");
            return ZCM_EUNKNOWN;
        }
    }
    zcm_url_t* u = zcm_url_create(url);
    if (!u) {
        return ZCM_EMEMORY;
    }
    const char* protocol = zcm_url_protocol(u);

    int ret = ZCM_ECONNECT;
    zcm_trans_create_func* creator = zcm_transport_find(protocol);
    if (creator) {
        zcm_trans_t* trans = creator(u);
        if (trans) {
            ret = zcm_init_from_trans(zcm, trans);
        } else {
            ZCM_DEBUG("failed to create transport for '%s'", url);
        }
    } else {
        ZCM_DEBUG("failed to find transport for '%s'", url);
    }

    zcm_url_destroy(u);
    return ret;
}
#endif

int zcm_init_from_trans(zcm_t* zcm, zcm_trans_t* zt)
{
    /* Check for valid transport */
    if (zt == NULL) {
        zcm->impl = NULL;
        return ZCM_EINVALID;
    }

    int ret = ZCM_EUNKNOWN;
    switch (zt->trans_type) {
        case ZCM_BLOCKING:
            #ifndef ZCM_EMBEDDED
                zcm->type = ZCM_BLOCKING;
                ret = zcm_blocking_try_create((zcm_blocking_t**) &zcm->impl, zcm, zt);
                break;
            #else
                zcm->impl = NULL;
                return ZCM_EINVALID;
            #endif
        case ZCM_NONBLOCKING:
            zcm->type = ZCM_NONBLOCKING;
            ret = zcm_nonblocking_try_create((zcm_nonblocking_t**) &zcm->impl, zcm, zt);
            break;
    }

    /* Just a safeguard in case impl is accidentally not
       initialized to NULL on any failures */
    if (ret != ZCM_EOK) zcm->impl = NULL;

    return ret;
}

void zcm_cleanup(zcm_t* zcm)
{
    if (!zcm) return;

#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:    zcm_blocking_destroy(zcm->impl);    return;
        case ZCM_NONBLOCKING: zcm_nonblocking_destroy(zcm->impl); return;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    zcm_nonblocking_destroy(zcm->impl);
#endif
}

static const char* errcode_str[] = {
    #define X(n, v, s) s,
    ZCM_RETURN_CODES
    #undef X
};

const char* zcm_strerrno(int err)
{
    if (((unsigned) err) >= ZCM_NUM_RETURN_CODES) err = ZCM_NUM_RETURN_CODES;

    return errcode_str[(unsigned) err];
}

int zcm_publish(zcm_t* zcm, const char* channel, const uint8_t* data, uint32_t len)
{
    int ret = ZCM_EUNKNOWN;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_publish(zcm->impl, channel, data, len);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_publish(zcm->impl, channel, data, len);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_publish(zcm->impl, channel, data, len);
#endif
    return ret;
}

void zcm_flush(zcm_t* zcm)
{
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:    zcm_blocking_flush(zcm->impl);    break;
        case ZCM_NONBLOCKING: zcm_nonblocking_flush(zcm->impl); break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    zcm_nonblocking_flush(zcm->impl);
#endif
}

zcm_sub_t* zcm_subscribe(zcm_t* zcm, const char* channel, zcm_msg_handler_t cb, void* usr)
{
    zcm_sub_t* ret = NULL;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_subscribe(zcm->impl, channel, cb, usr);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_subscribe(zcm->impl, channel, cb, usr);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_subscribe(zcm->impl, channel, cb, usr);
#endif
    return ret;
}

int zcm_unsubscribe(zcm_t* zcm, zcm_sub_t* sub)
{
    int ret = ZCM_EUNKNOWN;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_unsubscribe(zcm->impl, sub);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_unsubscribe(zcm->impl, sub);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_unsubscribe(zcm->impl, sub);
#endif
    return ret;
}

#ifndef ZCM_EMBEDDED
void zcm_start(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_start(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
void zcm_stop(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_stop(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
void zcm_run(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_run(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
void zcm_pause(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_pause(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
void zcm_resume(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_resume(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
int zcm_handle(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_handle(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
void zcm_set_queue_size(zcm_t* zcm, uint32_t numMsgs)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_set_queue_size(zcm->impl, numMsgs);
}
#endif

#ifndef ZCM_EMBEDDED
int zcm_write_topology(zcm_t* zcm, const char* name)
{
    switch (zcm->type) {
        case ZCM_BLOCKING:    return zcm_blocking_write_topology(zcm->impl, name);
        case ZCM_NONBLOCKING: return zcm_nonblocking_write_topology(zcm->impl, name);
    }
    ZCM_ASSERT(0 && "Not possible");
    return ZCM_EUNKNOWN;
}
#endif

int zcm_handle_nonblock(zcm_t* zcm)
{
    int ret = ZCM_EUNKNOWN;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_handle_nonblock(zcm->impl);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_handle_nonblock(zcm->impl);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_handle_nonblock(zcm->impl);
#endif
    return ret;
}





/****************************************************************************/
/*    NOT FOR GENERAL USE. USED FOR LANGUAGE-SPECIFIC BINDINGS WITH VERY    */
/*                     SPECIFIC THREADING CONSTRAINTS                       */
/****************************************************************************/
zcm_sub_t* zcm_try_subscribe(zcm_t* zcm, const char* channel, zcm_msg_handler_t cb, void* usr)
{
    zcm_sub_t* ret = NULL;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_try_subscribe(zcm->impl, channel, cb, usr);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_subscribe(zcm->impl, channel, cb, usr);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_subscribe(zcm->impl, channel, cb, usr);
#endif
    return ret;
}

int zcm_try_unsubscribe(zcm_t* zcm, zcm_sub_t* sub)
{
    int ret = ZCM_EUNKNOWN;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_try_unsubscribe(zcm->impl, sub);
            break;
        case ZCM_NONBLOCKING:
            ret = zcm_nonblocking_unsubscribe(zcm->impl, sub);
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    ret = zcm_nonblocking_unsubscribe(zcm->impl, sub);
#endif
    return ret;
}

int zcm_try_flush(zcm_t* zcm)
{
    int ret = ZCM_EUNKNOWN;
#ifndef ZCM_EMBEDDED
    switch (zcm->type) {
        case ZCM_BLOCKING:
            ret = zcm_blocking_try_flush(zcm->impl);
            break;
        case ZCM_NONBLOCKING:
            zcm_nonblocking_flush(zcm->impl);
            ret = ZCM_EOK;
            break;
    }
#else
    ZCM_ASSERT(zcm->type == ZCM_NONBLOCKING);
    zcm_nonblocking_flush(zcm->impl);
    ret = ZCM_EOK;
#endif
    return ret;
}

#ifndef ZCM_EMBEDDED
int zcm_try_stop(zcm_t* zcm)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_try_stop(zcm->impl);
}
#endif

#ifndef ZCM_EMBEDDED
int  zcm_try_set_queue_size(zcm_t* zcm, uint32_t numMsgs)
{
    ZCM_ASSERT(zcm->type == ZCM_BLOCKING);
    return zcm_blocking_try_set_queue_size(zcm->impl, numMsgs);
}
#endif
/****************************************************************************/
