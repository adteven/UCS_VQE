#include <jni.h>
#include <android/log.h>
#include <stdlib.h>
#include <stdio.h>

#include "ucs_vqe.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "UCSVQE_JNI", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "UCSVQE_JNI", __VA_ARGS__))
#ifdef JNI_DEBUG
#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_INFO, "UCSVQE_JNI", __VA_ARGS__))
#else
#define LOGD(...)
#endif

static JavaVM *gJavaVM;

static void* g_in_neer = NULL;
static void* g_in_far = NULL;
static void* g_out_data = NULL;

#ifdef __cplusplus
extern "C" {
#endif

jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    JNIEnv *env;
    gJavaVM = vm;
    LOGI("JNI_OnLoad called");

    return JNI_VERSION_1_4;
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeInit
 * Signature: (ILcom/ucpaas/ucsvqe/UcsVqeConfig;)I
 */
JNIEXPORT jint JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeInit
  (JNIEnv *env, jobject thiz, jint sample_rate, jobject config)
{
    UcsVqeConfig vqeConfig;    
    jclass cfgCls = NULL;

    LOGI("UcsVqeInit()");
    if (NULL == config)
    {
        LOGE("UcsVqeInit() failed with null config");
        return -1;
    }

    cfgCls = (jclass)(env)->GetObjectClass(config);
    if (!cfgCls)
    {
        LOGE("UcsVqeInit() couldn't get UcsVqeConfig class.");
        return -1;
    }

    if (sample_rate != kUcsSampleRate8kHz
        && sample_rate != kUcsSampleRate16kHz)
    {
        LOGE("UcsVqeInit() unsupported sample rate(%d)", sample_rate);
        return -1;
    }

    memset(&vqeConfig, 0x00, sizeof(UcsVqeConfig));
        
    jfieldID aec_enable_id = (env)->GetFieldID(cfgCls, "aec_enable", "Z");
    jfieldID agc_enable_id = (env)->GetFieldID(cfgCls, "agc_enable", "Z");
    jfieldID ns_enable_id = (env)->GetFieldID(cfgCls, "ns_enable", "Z");
    jfieldID aec_level_id = (env)->GetFieldID(cfgCls, "aec_level", "I");
    jfieldID agc_level_id = (env)->GetFieldID(cfgCls, "agc_level", "I");
    jfieldID ns_level_id = (env)->GetFieldID(cfgCls, "ns_level", "I");

    vqeConfig.aec_enable = (ucs_bool)(env)->GetBooleanField(config, aec_enable_id);
    vqeConfig.agc_enable = (ucs_bool)(env)->GetBooleanField(config, agc_enable_id);
    vqeConfig.ns_enable = (ucs_bool)(env)->GetBooleanField(config, ns_enable_id);
    vqeConfig.aec_config.level = (UcsEcSuppressionLevel)(env)->GetIntField(config, aec_level_id);
    vqeConfig.agc_config.level = (UcsAgcLevel)(env)->GetIntField(config, agc_level_id);
    vqeConfig.ns_config.level = (UcsNsLevel)(env)->GetIntField(config, ns_level_id);

    return UCSVQE_Init(sample_rate, &vqeConfig);
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeClosed
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeClosed
  (JNIEnv *env, jobject thiz)
{
    LOGI("UcsVqeClosed()");
    UCSVQE_Closed();
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeSetDirectBufferAddress
 * Signature: (Ljava/nio/ByteBuffer;Ljava/nio/ByteBuffer;Ljava/nio/ByteBuffer;)V
 */
JNIEXPORT void JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeSetDirectBufferAddress
  (JNIEnv *env, jobject thiz, jobject jIn_neer, jobject jIn_far, jobject jOut_data)
{
    g_in_neer = env->GetDirectBufferAddress(jIn_neer);
    g_in_far = env->GetDirectBufferAddress(jIn_far);
    g_out_data = env->GetDirectBufferAddress(jOut_data);

    if ((NULL == g_in_neer) ||
        (NULL == g_in_far) ||
        (NULL == g_out_data))
    {
        LOGE("UcsVqeSetDirectBufferAddress() failed.");
    }
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeProc
 * Signature: ([BS[B)I
 */
JNIEXPORT jint JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeProc
  (JNIEnv *env, jobject thiz, jint delay_ms)
{
    LOGD("UcsVqeProc()");
    return UCSVQE_Process((short *)g_in_neer, delay_ms, (short *)g_out_data);
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeFarend
 * Signature: ([B)I
 */
JNIEXPORT jint JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeFarend
  (JNIEnv *env, jobject thiz)
{
    LOGD("UcsVqeFarend()");
    return UCSVQE_FarendAnalysis((short *)g_in_far);
}

/*
 * Class:     com_ucpaas_ucsvqe_UcsVqeInterface
 * Method:    UcsVqeSetLoudSpeakerMode
 * Signature: (Z)I
 */
JNIEXPORT jint JNICALL Java_com_ucpaas_ucsvqe_UcsVqeInterface_UcsVqeSetSpeakerEnable
  (JNIEnv *env, jobject thiz, jboolean enable)
{
    LOGI("UcsVqeSetSpeakerEnable()");
    return UCSVQE_SetSpeakerEnable((ucs_bool)enable);
}
#ifdef __cplusplus
}
#endif


