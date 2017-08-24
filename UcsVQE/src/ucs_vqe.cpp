
#include "webrtc/modules/audio_processing/include/audio_processing.h"
#include "webrtc/system_wrappers/interface/logging.h"
#include "webrtc/common_types.h"
#include "webrtc/modules/interface/module_common_types.h"
#include "webrtc/system_wrappers/interface/trace.h"

#include "ucs_vqe.h"
#include "ucs_trace.h"

using namespace webrtc;

// Audio processing
const NoiseSuppression::Level kDefaultNsMode = NoiseSuppression::kVeryHigh;
const GainControl::Mode kDefaultAgcMode =
#if defined(WEBRTC_ANDROID) || defined(WEBRTC_IOS)
GainControl::kAdaptiveDigital;
#else
GainControl::kAdaptiveAnalog;
#endif
const bool kDefaultAgcState =
#if defined(WEBRTC_ANDROID) || defined(WEBRTC_IOS)
false;
#else
true;
#endif
const GainControl::Mode kDefaultRxAgcMode = GainControl::kAdaptiveDigital;
const int kUcsVqeLengthMs = 10;

static AudioProcessing* audioproc = NULL;
static UcsTrace* ucsTrace = NULL;

static int sampleRate = kUcsSampleRate16kHz;

int UCSVQE_Init(int sample_rate, UcsVqeConfig* config)
{   
    int err = 0;
    int targetLevel = 6;
    int gain = 9;
    short nlpLevel = 1;
    NoiseSuppression::Level nsLevel = kDefaultNsMode;

    ucsTrace = new UcsTrace();
    Trace::CreateTrace();
    Trace::SetTraceCallback(ucsTrace);

    if (sample_rate != kUcsSampleRate8kHz
        && sample_rate != kUcsSampleRate16kHz)
    {
        LOG_FERR1(LS_ERROR, UcsVqeInit, sample_rate);
        err = -1;
        goto ERR1;
    }

    if (NULL == config)
    {
        LOG(LS_ERROR) << "UCSVQE_Init() invalid config.";
        err = -2;
        goto ERR1;
    }

    audioproc = AudioProcessing::Create();
    if (NULL == audioproc)
    {
        LOG_FERR0(LS_ERROR, AudioProcessing::Create());
        err = -3;
        goto ERR1;
    }

    // Configure AudioProcessing components.
    audioproc->Initialize(sample_rate,
                        sample_rate,
                        sample_rate,
                        AudioProcessing::kMono,
                        AudioProcessing::kMono,
                        AudioProcessing::kMono);

    audioproc->set_stream_key_pressed(0);

    if (audioproc->high_pass_filter()->Enable(true) != 0) {
        LOG_FERR1(LS_ERROR, high_pass_filter()->Enable, true);
        err = -4;
        goto ERR2;
    }

    if (audioproc->echo_cancellation()->enable_drift_compensation(false) != 0) {
        LOG_FERR1(LS_ERROR, enable_drift_compensation, false);
        err = -5;
        goto ERR2;
    }

    if (audioproc->echo_cancellation()->is_drift_compensation_enabled()) {
        audioproc->echo_cancellation()->set_stream_drift_samples(0);
    }

#if defined(_WIN32)
    EchoCancellation::SuppressionLevel ecLevel = EchoCancellation::kModerateSuppression;
    switch (config->aec_config.level)
    {
    case kUcsEcSuppressionLow:
        ecLevel = EchoCancellation::kLowSuppression;
        break;
    case kUcsEcSuppressionModerate:
        ecLevel = EchoCancellation::kModerateSuppression;
        break;
    case kUcsEcSuppressionHigh:
    case kUcsEcSuppressionVeryHigh:
        ecLevel = EchoCancellation::kHighSuppression;
        break;
    default:
        break;
    }
    if (audioproc->echo_cancellation()->Enable(config->aec_enable) != 0)
    {
        LOG_FERR1(LS_ERROR, echo_cancellation()->Enable, config->aec_enable);
        err = -5;
        goto ERR2;
    }

    if (audioproc->echo_cancellation()->set_suppression_level(ecLevel) != 0)
    {
        LOG_FERR1(LS_ERROR, echo_cancellation()->set_suppression_level, ecLevel);
        err = -5;
        goto ERR2;
    }
#else // defined(ANDROID) || defined(__APPLE__)
    // Disable the AEC before enable the AECM
    audioproc->echo_cancellation()->Enable(false);
    if (audioproc->echo_control_mobile()->Enable(config->aec_enable) != 0)
    {
        LOG_FERR1(LS_ERROR, echo_control_mobile()->Enable, config->aec_enable);
        err = -5;
        goto ERR2;
    }

    if (audioproc->echo_control_mobile()->enable_comfort_noise(true) != 0) {
        LOG(LS_ERROR) << "UCSVQE_Init() failed to set comfort noise state for AECM";
        err = -5;
        goto ERR2;
    }

    switch (config->aec_config.level)
    {
    case kUcsEcSuppressionLow:
    case kUcsEcSuppressionModerate:
        nlpLevel = 1;
        break;
    case kUcsEcSuppressionHigh:
        nlpLevel = 2;
        break;
    case kUcsEcSuppressionVeryHigh:
        nlpLevel = 3;
        break;
    default:
        break;
    }
    if (audioproc->echo_control_mobile()->set_nlp_level(nlpLevel) != 0) {
        LOG(LS_ERROR) << "UCSVQE_Init() failed to set nlpLevel for AECM";
        err = -5;
        goto ERR2;
    }

    if (audioproc->echo_control_mobile()->set_tail_len_ms(128) != 0) {
        LOG(LS_ERROR) << "UCSVQE_Init() failed to set tailLenMs for AECM";
        err = -5;
        goto ERR2;
    }

    UCSVQE_SetSpeakerEnable(ucs_false);

#if 0
#if defined(WEBRTC_ANDROID)
    if (audioproc->echo_control_mobile()->SetPhoneModelName(model, 16) != 0) {
        LOG_FERR1(LS_ERROR, echo_control_mobile()->SetPhoneModelName, false);
        err = -5;
        goto ERR2;
    }
#endif
#endif
#endif

    switch (config->ns_config.level)
    {
    case kUcsNsLow:
        nsLevel = NoiseSuppression::kLow;
        break;
    case kUcsNsModerate:
        nsLevel = NoiseSuppression::kModerate;
        break;
    case kUcsNsHigh:
        nsLevel = NoiseSuppression::kHigh;
        break;
    case kUcsNsVeryHigh:
        nsLevel = NoiseSuppression::kVeryHigh;
        break;
    default:
        break;
    }
    if (audioproc->noise_suppression()->set_level(nsLevel) != 0) {
        LOG_FERR1(LS_ERROR, noise_suppression()->set_level, nsLevel);
        err = -6;
        goto ERR2;
    }
    if (audioproc->noise_suppression()->Enable(config->ns_enable) != 0)
    {
        LOG_FERR1(LS_ERROR, noise_suppression()->Enable, config->ns_enable);
        err = -6;
        goto ERR2;
    }

    if (audioproc->gain_control()->Enable(config->agc_enable) != 0) {
        LOG_FERR1(LS_ERROR, gain_control()->Enable, config->agc_enable);
        err = -7;
        goto ERR2;
    }

    if (audioproc->gain_control()->set_analog_level_limits(kMinVolumeLevel, kMaxVolumeLevel) != 0) {
        LOG_FERR2(LS_ERROR, gain_control()->set_analog_level_limits, kMinVolumeLevel,
            kMaxVolumeLevel);
        err = -7;
        goto ERR2;
    }
    
    if (audioproc->gain_control()->set_stream_analog_level(0) != 0) {
        LOG_FERR1(LS_ERROR, gain_control()->set_stream_analog_level, 0);
        err = -7;
        goto ERR2;
    }

    if (audioproc->gain_control()->set_mode(kDefaultAgcMode) != 0) {
        LOG_FERR1(LS_ERROR, gain_control()->set_mode, kDefaultAgcMode);
        err = -7;
        goto ERR2;
    }

    switch (config->agc_config.level)
    {
    case kUcsAgcLow:
    {
        targetLevel = 9;
        gain = 6;
    }
        break;
    case kUcsAgcHigh:
    {
        targetLevel = 3;
        gain = 12;
    }
        break;
    default:
        break;
    }
    if (audioproc->gain_control()->set_target_level_dbfs(targetLevel) != 0) {
        LOG_FERR1(LS_ERROR, gain_control()->set_target_level_dbfs, targetLevel);
        err = -7;
        goto ERR2;
    }
    if (audioproc->gain_control()->set_compression_gain_db(gain) != 0) {
        LOG_FERR1(LS_ERROR, gain_control()->set_compression_gain_db, gain);
        err = -7;
        goto ERR2;
    }
    if (audioproc->gain_control()->enable_limiter(true) != 0) {
        LOG_FERR0(LS_ERROR, gain_control()->enable_limiter);
        err = -7;
        goto ERR2;
    }

    sampleRate = sample_rate;

    audioproc->SetPcmRecordPath("E:\\svn\\audio_3A\\test\\");

    return 0;

ERR2:
    if (audioproc)
    {
        delete audioproc;
        audioproc = NULL;
    }
ERR1:
    if (ucsTrace)
    {
        delete ucsTrace;
        ucsTrace = NULL;
    }

    return err;
}

void UCSVQE_Closed()
{
    if (audioproc)
    {
        delete audioproc;
        audioproc = NULL;
    }
}

int UCSVQE_Process(short* in_neer, int delay_ms, short* out_data)
{   
    AudioFrame audioFrame;
    if (NULL == audioproc)
    {
        return -1;
    }

    if (audioproc->set_stream_delay_ms(delay_ms) != 0) {        
        LOG_FERR1(LS_VERBOSE, set_stream_delay_ms, delay_ms);
    }

    audioFrame.Reset();
    audioFrame.sample_rate_hz_ = sampleRate;
    audioFrame.num_channels_ = 1;
    audioFrame.samples_per_channel_ = (sampleRate / (1000 / kUcsVqeLengthMs));

    memset(&audioFrame.data_, 0x00, AudioFrame::kMaxDataSizeSamples);
    memcpy(audioFrame.data_, in_neer, audioFrame.samples_per_channel_ * 2);
    audioproc->gain_control()->set_stream_analog_level(0);
    int err = audioproc->ProcessStream(&audioFrame);
    if (err != 0) {
        LOG(LS_ERROR) << "UCSVQE_Process() error: " << err;
        return -2;
    }

    memcpy(out_data, audioFrame.data_, audioFrame.samples_per_channel_ * 2);
    return 0;
}

int UCSVQE_FarendAnalysis(short* in_far)
{
    AudioFrame farframe;
    
    if (NULL == in_far || NULL == audioproc)
    {
        return -1;
    }

    farframe.Reset();
    farframe.sample_rate_hz_ = sampleRate;
    farframe.num_channels_ = 1;
    farframe.samples_per_channel_ = (sampleRate / (1000 / kUcsVqeLengthMs));
    memset(&farframe.data_, 0x00, AudioFrame::kMaxDataSizeSamples);
    memcpy(farframe.data_, in_far, farframe.samples_per_channel_ * 2);

    int err = audioproc->AnalyzeReverseStream(&farframe);
    if (err != 0)
    {
        LOG(LS_ERROR) << "AudioProcessingModule::AnalyzeReverseStream() => error" << err;
        return -2;
    }

    return 0;
}

int UCSVQE_SetSpeakerEnable(ucs_bool enable)
{
    if ( NULL == audioproc)
    {
        return -1;
    }

#ifndef _WIN32
    EchoControlMobile::RoutingMode aecmMode = 
        enable ? EchoControlMobile::kLoudSpeakerphone : EchoControlMobile::kEarpiece;
    if (audioproc->echo_control_mobile()->set_routing_mode(aecmMode) != 0) {
        LOG(LS_ERROR) << "UcsVqeSetLoudSpeakerMode() failed to set AECM routing mode";
        return -1;
    }    
    return 0;
#else
    LOG(LS_ERROR) << "UcsVqeSetLoudSpeakerMode() EC is not supported";
    return -1;
#endif
}

void UCSVQE_SetLogCb(ucs_log_cb log_cb)
{
    if (ucsTrace)
    {
        ucsTrace->RegisterTraceCb(log_cb);
    }
}

