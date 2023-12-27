#include <stdio.h>
#include "zcm/java/jni/zcm_zcm_ZCMJNI.h"
#include "zcm/zcm.h"
#include "zcm/util/debug.h"
#include <assert.h>
#include <stdbool.h>

#define PASS_THROUGH_FUNC(NAME, NATIVE_NAME, RET, SIG) \
/* \
 * Class:     zcm_zcm_ZCMJNI \
 * Method:    NAME \
 * Signature: SIG \
 */ \
JNIEXPORT RET JNICALL Java_zcm_zcm_ZCMJNI_ ## NAME \
(JNIEnv *env, jobject self) \
{ \
    Internal *I = getNativePtr(env, self); \
    assert(I && I->zcm); \
    return zcm_ ## NATIVE_NAME(I->zcm); \
}

typedef struct Internal Internal;
struct Internal
{
    JavaVM *jvm;
    zcm_t *zcm;
};

typedef struct Subscription Subscription;
struct Subscription {
    Internal* I;
    jobject self;
    zcm_sub_t* zcmsub;
    jobject javaUsr;
};

// J is the type signature for long
static jfieldID getNativePtrField(JNIEnv *env, jobject self)
{
    jclass cls = (*env)->GetObjectClass(env, self);
    return (*env)->GetFieldID(env, cls, "nativePtr", "J");
}

static Internal *getNativePtr(JNIEnv *env, jobject self)
{
    jfieldID fld = getNativePtrField(env, self);
    jlong nativePtr = (*env)->GetLongField(env, self, fld);
    return (Internal*) (intptr_t)nativePtr;
}

static void setNativePtr(JNIEnv *env, jobject self, Internal *zcm)
{
    jfieldID fld = getNativePtrField(env, self);
    (*env)->SetLongField(env, self, fld, (jlong)(intptr_t)zcm);
}

/*
 * Class:     zcm_zcm_ZCMJNI
 * Method:    initializeNative
 * Signature: (Ljava/lang/String;)Z
 */
JNIEXPORT jboolean JNICALL Java_zcm_zcm_ZCMJNI_initializeNative
(JNIEnv *env, jobject self, jstring urlJ)
{
    Internal *I = calloc(1, sizeof(Internal));
    int rc = (*env)->GetJavaVM(env, &I->jvm);
    assert(rc == 0);

    const char *url = NULL;
    if (urlJ)
        url = (*env)->GetStringUTFChars(env, urlJ, 0);
    I->zcm = zcm_create(url);
    if (url)
        (*env)->ReleaseStringUTFChars(env, urlJ, url);

    setNativePtr(env, self, I);

    return I->zcm ? 1 : 0;
}

PASS_THROUGH_FUNC(destroy, destroy, void, ()V)
PASS_THROUGH_FUNC(start, start, void, ()V)
PASS_THROUGH_FUNC(stop, stop, void, ()V)

/*
 * Class:     zcm_zcm_ZCMJNI
 * Method:    publish
 * Signature: (Ljava/lang/String;[BII)I
 */
JNIEXPORT jint JNICALL Java_zcm_zcm_ZCMJNI_publish
(JNIEnv *env, jobject self, jstring channelJ, jbyteArray dataJ, jint offsetJ, jint lenJ)
{
    Internal *I = getNativePtr(env, self);
    assert(I);

    const char *channel = (*env)->GetStringUTFChars(env, channelJ, 0);

    jbyte* data = (*env)->GetByteArrayElements(env, dataJ, NULL);

    assert(offsetJ == 0);
    int ret = zcm_publish(I->zcm, channel, (uint8_t*)data, lenJ);

    (*env)->ReleaseStringUTFChars(env, channelJ, channel);

    (*env)->ReleaseByteArrayElements(env, dataJ, data, JNI_ABORT);

    return ret;
}

static void handler(const zcm_recv_buf_t *rbuf, const char *channel, void *_usr)
{
    Subscription* subs = (Subscription*)_usr;
    Internal *I = (Internal *)subs->I;
    jobject self = subs->self;

    bool isAttached = false;
    JavaVM *vm = I->jvm;
    JNIEnv *env;

    int rc = (*vm)->GetEnv(vm, (void **)&env, JNI_VERSION_1_6);
    if (rc == JNI_EDETACHED) {
        if ((*vm)->AttachCurrentThread(vm, (void **)&env, NULL) != 0) {
            fprintf(stderr, "ZCMJNI: getEnv: Failed to attach Thread in JNI!\n");
        } else {
            isAttached = true;
        }
    } else if (rc == JNI_EVERSION) {
        fprintf(stderr, "ZCMJNI: getEnv: JNI version not supported!\n");
    }

    jstring channelJ = (*env)->NewStringUTF(env, channel);

    jbyteArray dataJ = (*env)->NewByteArray(env, rbuf->data_size);
    (*env)->SetByteArrayRegion(env, dataJ, 0, rbuf->data_size, (signed char*)rbuf->data);

    jint offsetJ = 0;
    jint lenJ = rbuf->data_size;

    jclass cls = (*env)->GetObjectClass(env, self);
    assert(cls);
    jmethodID receiveMessage =
        (*env)->GetMethodID(env, cls, "receiveMessage",
                            "(Ljava/lang/String;[BIILzcm/zcm/ZCM$Subscription;)V");
    assert(receiveMessage);

    (*env)->CallVoidMethod(env, self, receiveMessage,
                           channelJ, dataJ, offsetJ, lenJ, subs->javaUsr);

    // NOTE: if we attached this thread to dispatch up to java, we need to make sure
    //       that we detach before returning so the references get freed.
    //       Forgetting to do this step can cause references to be lost and memory to be leaked,
    //       ultimately crashing the entire process due to Out-of-Memory.
    if (isAttached)
        (*vm)->DetachCurrentThread(vm);
}

/*
 * Class:     zcm_zcm_ZCMJNI
 * Method:    subscribe
 * Signature: (Ljava/lang/String;Lzcm/zcm/ZCM;)I
 */
JNIEXPORT jobject JNICALL Java_zcm_zcm_ZCMJNI_subscribe
(JNIEnv *env, jobject self, jstring channelJ, jobject zcmObjJ, jobject usr)
{
    Internal *I = getNativePtr(env, self);
    assert(I);
    // TODO: Should delete usr and call DeleteGlobalRef on an unsubscribe call
    Subscription* subs = malloc(sizeof(Subscription));
    subs->self = (*env)->NewGlobalRef(env, zcmObjJ);
    subs->I = I;
    subs->javaUsr = (*env)->NewGlobalRef(env, usr);

    const char *channel = (*env)->GetStringUTFChars(env, channelJ, 0);

    // TODO: need to handle the subscription type returned from subscribe
    subs->zcmsub = zcm_subscribe(I->zcm, channel, handler, (void*)subs);

    (*env)->ReleaseStringUTFChars(env, channelJ, channel);

    return (*env)->NewDirectByteBuffer(env, (void*)subs, 0);
}

/*
 * Class:     zcm_zcm_ZCMJNI
 * Method:    unsubscribe
 * Signature: (Lzcm/zcm/ZCM;Lzcm/zcm/ZCM/Subscription;)I
 */
JNIEXPORT jint JNICALL Java_zcm_zcm_ZCMJNI_unsubscribe
(JNIEnv *env, jobject self, jobject _subs)
{
    Internal *I = getNativePtr(env, self);
    assert(I);

    Subscription* subs = (Subscription*) (*env)->GetDirectBufferAddress(env, _subs);
    (*env)->DeleteGlobalRef(env, subs->javaUsr);
    (*env)->DeleteGlobalRef(env, subs->self);

    int ret = zcm_unsubscribe(I->zcm, subs->zcmsub);

    free(subs);

    return ret;
}

PASS_THROUGH_FUNC(flush, flush, void, ()V)
PASS_THROUGH_FUNC(pause, pause, void, ()V)
PASS_THROUGH_FUNC(resume, resume, void, ()V)
PASS_THROUGH_FUNC(handle, handle, jint, ()I)
PASS_THROUGH_FUNC(handleNonblock, handle_nonblock, jint, ()I)
