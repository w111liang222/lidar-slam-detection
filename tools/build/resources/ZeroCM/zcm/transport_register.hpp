#ifndef _ZCM_TRANS_REGISTER_H
#define _ZCM_TRANS_REGISTER_H

#include <zcm/transport_registrar.h>

struct TransportRegister {
    TransportRegister(const char* name, const char* desc, zcm_trans_create_func *creator)
    {
        zcm_transport_register(name, desc, creator);
    }
};

#endif  /* _ZCM_TRANS_REGISTER_H */
