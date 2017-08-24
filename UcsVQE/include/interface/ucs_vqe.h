#ifndef __UCS_VQE_H__
#define __UCS_VQE_H__

#if defined(_MSC_VER)
#define UCS_PUBLIC	__declspec(dllexport)
#else
#define UCS_PUBLIC
#endif

typedef enum{ucs_false, ucs_true} ucs_bool;

typedef enum
{
    kUcsSampleRate8kHz = 8000,
    kUcsSampleRate16kHz = 16000
} UcsVqeRate;

/* AEC Non-linear level*/
typedef enum {
    kUcsEcSuppressionDefault,
    /* only for Non-mobile */
    kUcsEcSuppressionLow,
    kUcsEcSuppressionModerate,
    kUcsEcSuppressionHigh,
    /* only for mobile */
    kUcsEcSuppressionVeryHigh
} UcsEcSuppressionLevel;

typedef struct
{
    UcsEcSuppressionLevel level;
} UcsEcConfig;

typedef enum {
    kUcsNsDefault,
    kUcsNsLow,
    kUcsNsModerate,
    kUcsNsHigh,
    kUcsNsVeryHigh
} UcsNsLevel;

typedef struct
{
    UcsNsLevel level;
} UcsNsConfig;

typedef enum
{
    kUcsAgcDefault,
    kUcsAgcLow,
    kUcsAgcModerate,
    kUcsAgcHigh
} UcsAgcLevel;

typedef struct
{
    UcsAgcLevel level;
} UcsAgcConfig;

typedef struct UcsVqeConfig
{
    ucs_bool aec_enable;
    UcsEcConfig aec_config;
    
    ucs_bool agc_enable;
    UcsAgcConfig agc_config;

    ucs_bool ns_enable;
    UcsNsConfig ns_config;
} UcsVqeConfig;

typedef void(*ucs_log_cb)(const char* message, int length);

#ifdef __cplusplus
extern "C" {
#endif
UCS_PUBLIC int UCSVQE_Init(int sample_rate, UcsVqeConfig* config);
UCS_PUBLIC void UCSVQE_Closed();
UCS_PUBLIC int UCSVQE_Process(short* in_neer, int delay_ms, short* out_data);
UCS_PUBLIC int UCSVQE_FarendAnalysis(short* in_far);
UCS_PUBLIC int UCSVQE_SetSpeakerEnable(ucs_bool enable);
UCS_PUBLIC void UCSVQE_SetLogCb(ucs_log_cb log_cb);
#ifdef __cplusplus
}
#endif
#endif // __UCS_VQE_H__

