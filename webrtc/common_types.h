/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_COMMON_TYPES_H_
#define WEBRTC_COMMON_TYPES_H_

#include <stddef.h>
#include <string.h>

#include <string>
#include <vector>

#include "webrtc/typedefs.h"

#if defined(_MSC_VER)
// Disable "new behavior: elements of array will be default initialized"
// warning. Affects OverUseDetectorOptions.
#pragma warning(disable:4351)
#endif

#ifdef WEBRTC_EXPORT
#define WEBRTC_DLLEXPORT _declspec(dllexport)
#elif WEBRTC_DLL
#define WEBRTC_DLLEXPORT _declspec(dllimport)
#else
#define WEBRTC_DLLEXPORT
#endif

#ifndef NULL
#define NULL 0
#endif

#define RTP_PAYLOAD_NAME_SIZE 32

#if defined(WEBRTC_WIN) || defined(WIN32)
// Compares two strings without regard to case.
#define STR_CASE_CMP(s1, s2) ::_stricmp(s1, s2)
// Compares characters of two strings without regard to case.
#define STR_NCASE_CMP(s1, s2, n) ::_strnicmp(s1, s2, n)
#else
#define STR_CASE_CMP(s1, s2) ::strcasecmp(s1, s2)
#define STR_NCASE_CMP(s1, s2, n) ::strncasecmp(s1, s2, n)
#endif

namespace webrtc {

class Config;

enum TraceModule
{
    kTraceUndefined              = 0,
    // not a module, triggered from the engine code
    kTraceVoice                  = 0x0001,
    // not a module, triggered from the engine code
    kTraceVideo                  = 0x0002,
    // not a module, triggered from the utility code
    kTraceUtility                = 0x0003,
    kTraceRtpRtcp                = 0x0004,
    kTraceTransport              = 0x0005,
    kTraceSrtp                   = 0x0006,
    kTraceAudioCoding            = 0x0007,
    kTraceAudioMixerServer       = 0x0008,
    kTraceAudioMixerClient       = 0x0009,
    kTraceFile                   = 0x000a,
    kTraceAudioProcessing        = 0x000b,
    kTraceVideoCoding            = 0x0010,
    kTraceVideoMixer             = 0x0011,
    kTraceAudioDevice            = 0x0012,
    kTraceVideoRenderer          = 0x0014,
    kTraceVideoCapture           = 0x0015,
    kTraceRemoteBitrateEstimator = 0x0017,
};

enum TraceLevel
{
    kTraceNone               = 0x0000,    // no trace
    kTraceStateInfo          = 0x0001,
    kTraceWarning            = 0x0002,
    kTraceError              = 0x0004,
    kTraceCritical           = 0x0008,
    kTraceApiCall            = 0x0010,
    kTraceDefault            = 0x00ff,

    kTraceModuleCall         = 0x0020,
    kTraceMemory             = 0x0100,   // memory info
    kTraceTimer              = 0x0200,   // timing info
    kTraceStream             = 0x0400,   // "continuous" stream of data

    // used for debug purposes
    kTraceDebug              = 0x0800,  // debug
    kTraceInfo               = 0x1000,  // debug info

    // Non-verbose level used by LS_INFO of logging.h. Do not use directly.
    kTraceTerseInfo          = 0x2000,
    kTraceReport             = 0x4000,  // Report

    kTraceAll                = 0xffff
};

// External Trace API
class TraceCallback {
 public:
  virtual void Print(TraceLevel level, const char* message, int length) = 0;

 protected:
  virtual ~TraceCallback() {}
  TraceCallback() {}
};

enum FileFormats
{
    kFileFormatWavFile        = 1,
    kFileFormatCompressedFile = 2,
	kFileFormatAviFile = 3,
    kFileFormatPreencodedFile = 4,
    kFileFormatPcm16kHzFile   = 7,
    kFileFormatPcm8kHzFile    = 8,
    kFileFormatPcm32kHzFile   = 9
};

enum ProcessingTypes
{
    kPlaybackPerChannel = 0,
    kPlaybackAllChannelsMixed,
    kRecordingPerChannel,
    kRecordingAllChannelsMixed,
    kRecordingPreprocessing
};

enum FrameType
{
    kFrameEmpty            = 0,
    kAudioFrameSpeech      = 1,
    kAudioFrameCN          = 2,
    kVideoFrameKey         = 3,    // independent frame
    kVideoFrameDelta       = 4,    // depends on the previus frame
	kNotifyFrameFec        = 5,    // add niyuanxun
};

enum PayloadFrequencies
{
    kFreq8000Hz = 8000,
    kFreq16000Hz = 16000,
    kFreq32000Hz = 32000
};

enum VadModes                 // degree of bandwidth reduction
{
    kVadConventional = 0,      // lowest reduction
    kVadAggressiveLow,
    kVadAggressiveMid,
    kVadAggressiveHigh         // highest reduction
};

enum NsModes    // type of Noise Suppression
{
    kNsUnchanged = 0,   // previously set mode
    kNsDefault,         // platform default
    kNsConference,      // conferencing default
    kNsLowSuppression,  // lowest suppression
    kNsModerateSuppression,
    kNsHighSuppression,
    kNsVeryHighSuppression,     // highest suppression
};

enum AgcModes                  // type of Automatic Gain Control
{
    kAgcUnchanged = 0,        // previously set mode
    kAgcDefault,              // platform default
    // adaptive mode for use when analog volume control exists (e.g. for
    // PC softphone)
    kAgcAdaptiveAnalog,
    // scaling takes place in the digital domain (e.g. for conference servers
    // and embedded devices)
    kAgcAdaptiveDigital,
    // can be used on embedded devices where the capture signal level
    // is predictable
    kAgcFixedDigital
};

// EC modes
enum EcModes                   // type of Echo Control
{
    kEcUnchanged = 0,          // previously set mode
    kEcDefault,                // platform default
    kEcConference,             // conferencing default (aggressive AEC)
    kEcAec,                    // Acoustic Echo Cancellation
    kEcAecm,                   // AEC mobile
};

// AECM modes
enum AecmModes                 // mode of AECM
{
    kAecmQuietEarpieceOrHeadset = 0,
                               // Quiet earpiece or headset use
    kAecmEarpiece,             // most earpiece use
    kAecmLoudEarpiece,         // Loud earpiece or quiet speakerphone use
    kAecmSpeakerphone,         // most speakerphone use (default)
    kAecmLoudSpeakerphone      // Loud speakerphone
};

// AGC configuration
typedef struct
{
    unsigned short targetLeveldBOv;
    unsigned short digitalCompressionGaindB;
    bool           limiterEnable;
} AgcConfig;                  // AGC configuration parameters

enum StereoChannel
{
    kStereoLeft = 0,
    kStereoRight,
    kStereoBoth
};

}  // namespace webrtc

#endif  // WEBRTC_COMMON_TYPES_H_
