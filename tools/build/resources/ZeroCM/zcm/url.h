#ifndef _ZCM_URL_H
#define _ZCM_URL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#define ZCM_OPTS_MAX 128

typedef struct zcm_url_opts zcm_url_opts_t;
struct zcm_url_opts
{
    size_t numopts;
    const char *name[ZCM_OPTS_MAX];
    const char *value[ZCM_OPTS_MAX];
};

typedef struct zcm_url zcm_url_t;
zcm_url_t *zcm_url_create(const char *url);
void zcm_url_destroy(zcm_url_t *u);

const char *zcm_url_protocol(zcm_url_t *u);
const char *zcm_url_address(zcm_url_t *u);
zcm_url_opts_t *zcm_url_opts(zcm_url_t *u);

#ifdef __cplusplus
}
#endif

#endif /* _ZCM_URL_H */
