#include "uv_zcm_msg_handler.h"

#include <iostream>

#include <thread>
#include "uv.h"

using namespace std;

struct uv_zcm_msg_handler_t
{
    zcm_msg_handler_t cb;
    void* usr;
    const zcm_recv_buf_t* rbuf;
    const char* channel;

	uv_barrier_t blocker;
    uv_async_t handle;
    std::thread::id main_thread_id;
};

#ifdef __cplusplus
extern "C" {
#endif

uv_zcm_msg_handler_t* uv_zcm_msg_handler_create(zcm_msg_handler_t cb, void* usr)
{
    uv_zcm_msg_handler_t* ret = new uv_zcm_msg_handler_t();
    ret->cb = cb;
    ret->usr = usr;
    ret->rbuf = nullptr;
    ret->channel = nullptr;

    // Set the usr pointer of the async handler to be the "this" pointer
	ret->handle.data = ret;

    ret->main_thread_id = std::this_thread::get_id();

    return ret;
}

void uv_zcm_msg_handler_trigger(const zcm_recv_buf_t* rbuf, const char* channel, void* _uvCb)
{
    uv_zcm_msg_handler_t* uvCb = (uv_zcm_msg_handler_t*) _uvCb;

    uvCb->rbuf = rbuf;
    uvCb->channel = channel;

    if (std::this_thread::get_id() == uvCb->main_thread_id) {
		uvCb->cb(uvCb->rbuf, uvCb->channel, uvCb->usr);
        return;
    }

    // Init the barrier to wait for 2 people to be waiting on it before continuing
	uv_barrier_init(&uvCb->blocker, 2);

    // Initialize the handle object with the main julia loop (uv_default_loop()),
    // and our callback.
    uv_async_init(uv_default_loop(), &uvCb->handle, [](uv_async_t *handle) {
		uv_zcm_msg_handler_t* uvCb = (uv_zcm_msg_handler_t*) handle->data;
        // Call the callback
		uvCb->cb(uvCb->rbuf, uvCb->channel, uvCb->usr);

        // TODO: We should probably call "uv_async_init" and "uv_close" on create
        //       and destroy of the uv zcm callback. The way we're doing it now
        //       works but is probably far less efficient than it needs to be
        // close async handle
        uv_close((uv_handle_t*)&uvCb->handle, [](uv_handle_t* handle){
            uv_zcm_msg_handler_t* uvCb = (uv_zcm_msg_handler_t*) handle->data;
            // wait on the barrier (should instantly wake up)
            uv_barrier_wait(&uvCb->blocker);
        });
    });

    // Schedule the callback to be called
    uv_async_send(&uvCb->handle);

    // Wait for callback to complete
    uv_barrier_wait(&uvCb->blocker);
    uv_barrier_destroy(&uvCb->blocker);
}

void uv_zcm_msg_handler_destroy(uv_zcm_msg_handler_t* uvCb)
{ delete uvCb; }

#ifdef __cplusplus
}
#endif
