/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/modules/audio_processing/aecm/aecm_core.h"

#include <assert.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>
//#include <string.h>

#include "webrtc/common_audio/ring_buffer.h"
#include "webrtc/common_audio/signal_processing/include/real_fft.h"
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/utility/delay_estimator_wrapper.h"
#include "webrtc/system_wrappers/interface/compile_assert_c.h"
#include "webrtc/system_wrappers/interface/cpu_features_wrapper.h"
#include "webrtc/typedefs.h"
#include <math.h>


#ifdef AEC_DEBUG
FILE *dfile;
FILE *testfile;
#endif


const int16_t WebRtcAecm_kCosTable[] = {
    8192,  8190,  8187,  8180,  8172,  8160,  8147,  8130,  8112,
    8091,  8067,  8041,  8012,  7982,  7948,  7912,  7874,  7834,
    7791,  7745,  7697,  7647,  7595,  7540,  7483,  7424,  7362,
    7299,  7233,  7164,  7094,  7021,  6947,  6870,  6791,  6710,
    6627,  6542,  6455,  6366,  6275,  6182,  6087,  5991,  5892,
    5792,  5690,  5586,  5481,  5374,  5265,  5155,  5043,  4930,
    4815,  4698,  4580,  4461,  4341,  4219,  4096,  3971,  3845,
    3719,  3591,  3462,  3331,  3200,  3068,  2935,  2801,  2667,
    2531,  2395,  2258,  2120,  1981,  1842,  1703,  1563,  1422,
    1281,  1140,   998,   856,   713,   571,   428,   285,   142,
       0,  -142,  -285,  -428,  -571,  -713,  -856,  -998, -1140,
   -1281, -1422, -1563, -1703, -1842, -1981, -2120, -2258, -2395,
   -2531, -2667, -2801, -2935, -3068, -3200, -3331, -3462, -3591,
   -3719, -3845, -3971, -4095, -4219, -4341, -4461, -4580, -4698,
   -4815, -4930, -5043, -5155, -5265, -5374, -5481, -5586, -5690,
   -5792, -5892, -5991, -6087, -6182, -6275, -6366, -6455, -6542,
   -6627, -6710, -6791, -6870, -6947, -7021, -7094, -7164, -7233,
   -7299, -7362, -7424, -7483, -7540, -7595, -7647, -7697, -7745,
   -7791, -7834, -7874, -7912, -7948, -7982, -8012, -8041, -8067,
   -8091, -8112, -8130, -8147, -8160, -8172, -8180, -8187, -8190,
   -8191, -8190, -8187, -8180, -8172, -8160, -8147, -8130, -8112,
   -8091, -8067, -8041, -8012, -7982, -7948, -7912, -7874, -7834,
   -7791, -7745, -7697, -7647, -7595, -7540, -7483, -7424, -7362,
   -7299, -7233, -7164, -7094, -7021, -6947, -6870, -6791, -6710,
   -6627, -6542, -6455, -6366, -6275, -6182, -6087, -5991, -5892,
   -5792, -5690, -5586, -5481, -5374, -5265, -5155, -5043, -4930,
   -4815, -4698, -4580, -4461, -4341, -4219, -4096, -3971, -3845,
   -3719, -3591, -3462, -3331, -3200, -3068, -2935, -2801, -2667,
   -2531, -2395, -2258, -2120, -1981, -1842, -1703, -1563, -1422,
   -1281, -1140,  -998,  -856,  -713,  -571,  -428,  -285,  -142,
       0,   142,   285,   428,   571,   713,   856,   998,  1140,
    1281,  1422,  1563,  1703,  1842,  1981,  2120,  2258,  2395,
    2531,  2667,  2801,  2935,  3068,  3200,  3331,  3462,  3591,
    3719,  3845,  3971,  4095,  4219,  4341,  4461,  4580,  4698,
    4815,  4930,  5043,  5155,  5265,  5374,  5481,  5586,  5690,
    5792,  5892,  5991,  6087,  6182,  6275,  6366,  6455,  6542,
    6627,  6710,  6791,  6870,  6947,  7021,  7094,  7164,  7233,
    7299,  7362,  7424,  7483,  7540,  7595,  7647,  7697,  7745,
    7791,  7834,  7874,  7912,  7948,  7982,  8012,  8041,  8067,
    8091,  8112,  8130,  8147,  8160,  8172,  8180,  8187,  8190
};

const int16_t WebRtcAecm_kSinTable[] = {
       0,    142,    285,    428,    571,    713,    856,    998,
    1140,   1281,   1422,   1563,   1703,   1842,   1981,   2120,
    2258,   2395,   2531,   2667,   2801,   2935,   3068,   3200,
    3331,   3462,   3591,   3719,   3845,   3971,   4095,   4219,
    4341,   4461,   4580,   4698,   4815,   4930,   5043,   5155,
    5265,   5374,   5481,   5586,   5690,   5792,   5892,   5991,
    6087,   6182,   6275,   6366,   6455,   6542,   6627,   6710,
    6791,   6870,   6947,   7021,   7094,   7164,   7233,   7299,
    7362,   7424,   7483,   7540,   7595,   7647,   7697,   7745,
    7791,   7834,   7874,   7912,   7948,   7982,   8012,   8041,
    8067,   8091,   8112,   8130,   8147,   8160,   8172,   8180,
    8187,   8190,   8191,   8190,   8187,   8180,   8172,   8160,
    8147,   8130,   8112,   8091,   8067,   8041,   8012,   7982,
    7948,   7912,   7874,   7834,   7791,   7745,   7697,   7647,
    7595,   7540,   7483,   7424,   7362,   7299,   7233,   7164,
    7094,   7021,   6947,   6870,   6791,   6710,   6627,   6542,
    6455,   6366,   6275,   6182,   6087,   5991,   5892,   5792,
    5690,   5586,   5481,   5374,   5265,   5155,   5043,   4930,
    4815,   4698,   4580,   4461,   4341,   4219,   4096,   3971,
    3845,   3719,   3591,   3462,   3331,   3200,   3068,   2935,
    2801,   2667,   2531,   2395,   2258,   2120,   1981,   1842,
    1703,   1563,   1422,   1281,   1140,    998,    856,    713,
     571,    428,    285,    142,      0,   -142,   -285,   -428,
    -571,   -713,   -856,   -998,  -1140,  -1281,  -1422,  -1563,
   -1703,  -1842,  -1981,  -2120,  -2258,  -2395,  -2531,  -2667,
   -2801,  -2935,  -3068,  -3200,  -3331,  -3462,  -3591,  -3719,
   -3845,  -3971,  -4095,  -4219,  -4341,  -4461,  -4580,  -4698,
   -4815,  -4930,  -5043,  -5155,  -5265,  -5374,  -5481,  -5586,
   -5690,  -5792,  -5892,  -5991,  -6087,  -6182,  -6275,  -6366,
   -6455,  -6542,  -6627,  -6710,  -6791,  -6870,  -6947,  -7021,
   -7094,  -7164,  -7233,  -7299,  -7362,  -7424,  -7483,  -7540,
   -7595,  -7647,  -7697,  -7745,  -7791,  -7834,  -7874,  -7912,
   -7948,  -7982,  -8012,  -8041,  -8067,  -8091,  -8112,  -8130,
   -8147,  -8160,  -8172,  -8180,  -8187,  -8190,  -8191,  -8190,
   -8187,  -8180,  -8172,  -8160,  -8147,  -8130,  -8112,  -8091,
   -8067,  -8041,  -8012,  -7982,  -7948,  -7912,  -7874,  -7834,
   -7791,  -7745,  -7697,  -7647,  -7595,  -7540,  -7483,  -7424,
   -7362,  -7299,  -7233,  -7164,  -7094,  -7021,  -6947,  -6870,
   -6791,  -6710,  -6627,  -6542,  -6455,  -6366,  -6275,  -6182,
   -6087,  -5991,  -5892,  -5792,  -5690,  -5586,  -5481,  -5374,
   -5265,  -5155,  -5043,  -4930,  -4815,  -4698,  -4580,  -4461,
   -4341,  -4219,  -4096,  -3971,  -3845,  -3719,  -3591,  -3462,
   -3331,  -3200,  -3068,  -2935,  -2801,  -2667,  -2531,  -2395,
   -2258,  -2120,  -1981,  -1842,  -1703,  -1563,  -1422,  -1281,
   -1140,   -998,   -856,   -713,   -571,   -428,   -285,   -142
};

// Initialization table for echo channel in 8 kHz
static const int16_t kChannelStored8kHz[PART_LEN1] = {
    2040,   1815,   1590,   1498,   1405,   1395,   1385,   1418,
    1451,   1506,   1562,   1644,   1726,   1804,   1882,   1918,
    1953,   1982,   2010,   2025,   2040,   2034,   2027,   2021,
    2014,   1997,   1980,   1925,   1869,   1800,   1732,   1683,
    1635,   1604,   1572,   1545,   1517,   1481,   1444,   1405,
    1367,   1331,   1294,   1270,   1245,   1239,   1233,   1247,
    1260,   1282,   1303,   1338,   1373,   1407,   1441,   1470,
    1499,   1524,   1549,   1565,   1582,   1601,   1621,   1649,
    1676
};

// Initialization table for echo channel in 16 kHz
static const int16_t kChannelStored16kHz[PART_LEN1] = {
    2040,   1590,   1405,   1385,   1451,   1562,   1726,   1882,
    1953,   2010,   2040,   2027,   2014,   1980,   1869,   1732,
    1635,   1572,   1517,   1444,   1367,   1294,   1245,   1233,
    1260,   1303,   1373,   1441,   1499,   1549,   1582,   1621,
    1676,   1741,   1802,   1861,   1921,   1983,   2040,   2102,
    2170,   2265,   2375,   2515,   2651,   2781,   2922,   3075,
    3253,   3471,   3738,   3976,   4151,   4258,   4308,   4288,
    4270,   4253,   4237,   4179,   4086,   3947,   3757,   3484,
    3153
};

// Moves the pointer to the next entry and inserts |far_spectrum| and
// corresponding Q-domain in its buffer.
//
// Inputs:
//      - self          : Pointer to the delay estimation instance
//      - far_spectrum  : Pointer to the far end spectrum
//      - far_q         : Q-domain of far end spectrum
//
void WebRtcAecm_UpdateFarHistory(AecmCore* self,
                                 uint16_t* far_spectrum,
                                 int far_q) {
  // Get new buffer position
  self->far_history_pos++;
  if (self->far_history_pos >= MAX_DELAY) {
    self->far_history_pos = 0;
  }
  // Update Q-domain buffer
  self->far_q_domains[self->far_history_pos] = far_q;
  // Update far end spectrum buffer
  memcpy(&(self->far_history[self->far_history_pos * PART_LEN1]),
         far_spectrum,
         sizeof(uint16_t) * PART_LEN1);
}

void WebRtcAecm_UpdateFarHistoryComplex(AecmCore* self,ComplexInt16* far_spectrum_complex) 
{  
  // Update far end spectrum buffer
  memcpy(&(self->far_history_complex[self->far_history_pos * PART_LEN1]),far_spectrum_complex,sizeof(ComplexInt16) * PART_LEN1);
}

#ifdef MUSIC_OR_SPEECH
void WebRtcAecm_UpdateFarHistoryTime(AecmCore* self,int16_t* far_data) 
{  
  // Update far end spectrum buffer
  memcpy(&(self->far_history_time[self->far_history_pos * PART_LEN]),far_data,sizeof(int16_t) * PART_LEN);
}
#endif


// Returns a pointer to the far end spectrum aligned to current near end
// spectrum. The function WebRtc_DelayEstimatorProcessFix(...) should have been
// called before AlignedFarend(...). Otherwise, you get the pointer to the
// previous frame. The memory is only valid until the next call of
// WebRtc_DelayEstimatorProcessFix(...).
//
// Inputs:
//      - self              : Pointer to the AECM instance.
//      - delay             : Current delay estimate.
//
// Output:
//      - far_q             : The Q-domain of the aligned far end spectrum
//
// Return value:
//      - far_spectrum      : Pointer to the aligned far end spectrum
//                            NULL - Error
//
const uint16_t* WebRtcAecm_AlignedFarend(AecmCore* self,
                                         int* far_q,
                                         int delay) {
  int buffer_position = 0;
  assert(self != NULL);
  buffer_position = self->far_history_pos - delay;

  // Check buffer position
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  // Get Q-domain
  *far_q = self->far_q_domains[buffer_position];
  // Return far end spectrum
  return &(self->far_history[buffer_position * PART_LEN1]);
}
#ifdef MUSIC_OR_SPEECH
const int16_t* WebRtcAecm_AlignedFarendTime(AecmCore* self,                                        
                                         int delay) {
  int buffer_position = 0;
  assert(self != NULL);
  buffer_position = self->far_history_pos - delay;

  // Check buffer position
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  
  return &(self->far_history_time[buffer_position * PART_LEN]);
}
#endif

const ComplexInt16* WebRtcAecm_AlignedFarend_complex(AecmCore* self,  int delay) 
{
  int buffer_position = 0;
  assert(self != NULL);
  buffer_position = self->far_history_pos - delay;

  // Check buffer position
  if (buffer_position < 0) {
    buffer_position += MAX_DELAY;
  }
  // Get Q-domain
  //*far_q = self->far_q_domains[buffer_position];
  // Return far end spectrum
  return &(self->far_history_complex[buffer_position * PART_LEN1]);
}



// Declare function pointers.
CalcLinearEnergies WebRtcAecm_CalcLinearEnergies;
StoreAdaptiveChannel WebRtcAecm_StoreAdaptiveChannel;
ResetAdaptiveChannel WebRtcAecm_ResetAdaptiveChannel;

int WebRtcAecm_CreateCore(AecmCore** aecmInst) {
    AecmCore* aecm = malloc(sizeof(AecmCore));
    *aecmInst = aecm;
    if (aecm == NULL)
    {
        return -1;
    }

    aecm->farFrameBuf = WebRtc_CreateBuffer(FRAME_LEN + PART_LEN,
                                            sizeof(int16_t));
    if (!aecm->farFrameBuf)
    {
        WebRtcAecm_FreeCore(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->nearNoisyFrameBuf = WebRtc_CreateBuffer(FRAME_LEN + PART_LEN,
                                                  sizeof(int16_t));
    if (!aecm->nearNoisyFrameBuf)
    {
        WebRtcAecm_FreeCore(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->nearCleanFrameBuf = WebRtc_CreateBuffer(FRAME_LEN + PART_LEN,
                                                  sizeof(int16_t));
    if (!aecm->nearCleanFrameBuf)
    {
        WebRtcAecm_FreeCore(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->outFrameBuf = WebRtc_CreateBuffer(FRAME_LEN + PART_LEN,
                                            sizeof(int16_t));
    if (!aecm->outFrameBuf)
    {
        WebRtcAecm_FreeCore(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->delay_estimator_farend = WebRtc_CreateDelayEstimatorFarend(PART_LEN1,
                                                                     MAX_DELAY);
    if (aecm->delay_estimator_farend == NULL) {
      WebRtcAecm_FreeCore(aecm);
      aecm = NULL;
      return -1;
    }
    aecm->delay_estimator =
        WebRtc_CreateDelayEstimator(aecm->delay_estimator_farend, 0);
    if (aecm->delay_estimator == NULL) {
      WebRtcAecm_FreeCore(aecm);
      aecm = NULL;
      return -1;
    }
    // TODO(bjornv): Explicitly disable robust delay validation until no
    // performance regression has been established.  Then remove the line.	
    WebRtc_enable_robust_validation(aecm->delay_estimator, 0);

    aecm->real_fft = WebRtcSpl_CreateRealFFT(PART_LEN_SHIFT);
    if (aecm->real_fft == NULL) {
      WebRtcAecm_FreeCore(aecm);
      aecm = NULL;
      return -1;
    }

    // Init some aecm pointers. 16 and 32 byte alignment is only necessary
    // for Neon code currently.
    aecm->xBuf = (int16_t*) (((uintptr_t)aecm->xBuf_buf + 31) & ~ 31);
    aecm->dBufClean = (int16_t*) (((uintptr_t)aecm->dBufClean_buf + 31) & ~ 31);
    aecm->dBufNoisy = (int16_t*) (((uintptr_t)aecm->dBufNoisy_buf + 31) & ~ 31);
    aecm->outBuf = (int16_t*) (((uintptr_t)aecm->outBuf_buf + 15) & ~ 15);
    aecm->channelStored = (int16_t*) (((uintptr_t)
                                             aecm->channelStored_buf + 15) & ~ 15);
    aecm->channelAdapt16 = (int16_t*) (((uintptr_t)
                                              aecm->channelAdapt16_buf + 15) & ~ 15);
    aecm->channelAdapt32 = (int32_t*) (((uintptr_t)
                                              aecm->channelAdapt32_buf + 31) & ~ 31);
    return 0;
}

void WebRtcAecm_InitEchoPathCore(AecmCore* aecm, const int16_t* echo_path) {
    int i = 0;

    // Reset the stored channel
    memcpy(aecm->channelStored, echo_path, sizeof(int16_t) * PART_LEN1);

		
	memcpy(aecm->channelStoredBackup, aecm->channelStored, sizeof(int16_t) * PART_LEN1);	
	aecm->backupFilterNeverUpdate=1;
	aecm->diffStoredBackupSmoothed=0;
	aecm->mseStoredSmoothed=32*MIN_MSE_COUNT;
	aecm->mseAdaptSmoothed=64*MIN_MSE_COUNT;
	aecm->mseStoredSmallerCnt=0;
	aecm->backupFilterUpdateTime=0;
	memcpy(aecm->storedFilterCandidate,aecm->channelStoredBackup,PART_LEN1*sizeof(int16_t));
	aecm->minMseStored = 64*MIN_MSE_COUNT;
	aecm->diffStoredCandidate=0;
	aecm->mseReach20Cnt=0;
    aecm->backupFilterUpdateCnt=0;
	aecm->mseChannelCntClearCnt=0;

    // Reset the adapted channels
    memcpy(aecm->channelAdapt16, echo_path, sizeof(int16_t) * PART_LEN1);
    for (i = 0; i < PART_LEN1; i++)
    {
        aecm->channelAdapt32[i] = (int32_t)aecm->channelAdapt16[i] << 16;
    }

    // Reset channel storing variables
    aecm->mseAdaptOld = 1000;
    aecm->mseStoredOld = 1000;
    aecm->mseThreshold = WEBRTC_SPL_WORD32_MAX;
    aecm->mseChannelCount = 0;
}

static void CalcLinearEnergiesC(AecmCore* aecm,
								const uint16_t* far_spectrum,
								int32_t* echo_est,
								uint32_t* far_energy,
								uint32_t* echo_energy_adapt,
								uint32_t* echo_energy_stored)

{
    int i;
    // Get energy for the delayed far end signal and estimated
    // echo using both stored and adapted channels.
    for (i = 0; i < PART_LEN1; i++)
    {
        echo_est[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i],
                                           far_spectrum[i]);
        (*far_energy) += (uint32_t)(far_spectrum[i]);
        *echo_energy_adapt += aecm->channelAdapt16[i] * far_spectrum[i];
        (*echo_energy_stored) += (uint32_t)echo_est[i];

    }
}

static void CalcEchoStoreBackupEn(AecmCore* aecm,
								const uint16_t* far_spectrum,								
								uint32_t* echo_energy_stored_backup)

{
    int i;

	int32_t echo_est_backup=0;

    // Get energy for the delayed far end signal and estimated
    // echo using both stored and adapted channels.
    for (i = 0; i < PART_LEN1; i++)
    {
		echo_est_backup +=WEBRTC_SPL_MUL_16_U16(aecm->channelStoredBackup[i],far_spectrum[i]);
		
    }
	*echo_energy_stored_backup=echo_est_backup;
}


static void StoreAdaptiveChannelC(AecmCore* aecm,
                                  const uint16_t* far_spectrum,
                                  int32_t* echo_est) {
    int i;
    // During startup we store the channel every block.
    memcpy(aecm->channelStored, aecm->channelAdapt16, sizeof(int16_t) * PART_LEN1);
    // Recalculate echo estimate
    for (i = 0; i < PART_LEN; i += 4)
    {
        echo_est[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i],
                                           far_spectrum[i]);
        echo_est[i + 1] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i + 1],
                                           far_spectrum[i + 1]);
        echo_est[i + 2] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i + 2],
                                           far_spectrum[i + 2]);
        echo_est[i + 3] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i + 3],
                                           far_spectrum[i + 3]);
    }
    echo_est[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i],
                                       far_spectrum[i]);
}




static void ResetAdaptiveChannelC(AecmCore* aecm) {
	int i;

    // The stored channel has a significantly lower MSE than the adaptive one for
    // two consecutive calculations. Reset the adaptive channel.
    memcpy(aecm->channelAdapt16, aecm->channelStored,
           sizeof(int16_t) * PART_LEN1);
    // Restore the W32 channel
    for (i = 0; i < PART_LEN; i += 4)
    {
        aecm->channelAdapt32[i] = (int32_t)aecm->channelStored[i] << 16;
        aecm->channelAdapt32[i + 1] = (int32_t)aecm->channelStored[i + 1] << 16;
        aecm->channelAdapt32[i + 2] = (int32_t)aecm->channelStored[i + 2] << 16;
        aecm->channelAdapt32[i + 3] = (int32_t)aecm->channelStored[i + 3] << 16;
    }
    aecm->channelAdapt32[i] = (int32_t)aecm->channelStored[i] << 16;
}


// Initialize function pointers for ARM Neon platform.
#if (defined WEBRTC_DETECT_ARM_NEON || defined WEBRTC_ARCH_ARM_NEON || \
     defined WEBRTC_ARCH_ARM64_NEON)
static void WebRtcAecm_InitNeon(void)
{
  WebRtcAecm_StoreAdaptiveChannel = WebRtcAecm_StoreAdaptiveChannelNeon;
  WebRtcAecm_ResetAdaptiveChannel = WebRtcAecm_ResetAdaptiveChannelNeon;
  WebRtcAecm_CalcLinearEnergies = WebRtcAecm_CalcLinearEnergiesNeon;
}
#endif

// Initialize function pointers for MIPS platform.
#if defined(MIPS32_LE)
static void WebRtcAecm_InitMips(void)
{
#if defined(MIPS_DSP_R1_LE)
  WebRtcAecm_StoreAdaptiveChannel = WebRtcAecm_StoreAdaptiveChannel_mips;
  WebRtcAecm_ResetAdaptiveChannel = WebRtcAecm_ResetAdaptiveChannel_mips;
#endif
  WebRtcAecm_CalcLinearEnergies = WebRtcAecm_CalcLinearEnergies_mips;
}
#endif

// WebRtcAecm_InitCore(...)
//
// This function initializes the AECM instant created with WebRtcAecm_CreateCore(...)
// Input:
//      - aecm            : Pointer to the Echo Suppression instance
//      - samplingFreq   : Sampling Frequency
//
// Output:
//      - aecm            : Initialized instance
//
// Return value         :  0 - Ok
//                        -1 - Error
//
int WebRtcAecm_InitCore(AecmCore* const aecm, int samplingFreq) {
    int i = 0;
    int32_t tmp32 = PART_LEN1 * PART_LEN1;
    int16_t tmp16 = PART_LEN1;

    if (samplingFreq != 8000 && samplingFreq != 16000)
    {
        samplingFreq = 8000;
        return -1;
    }
    // sanity check of sampling frequency
    aecm->mult = (int16_t)samplingFreq / 8000;

    aecm->farBufWritePos = 0;
    aecm->farBufReadPos = 0;
    aecm->knownDelay = 0;
    aecm->lastKnownDelay = 0;

    WebRtc_InitBuffer(aecm->farFrameBuf);
    WebRtc_InitBuffer(aecm->nearNoisyFrameBuf);
    WebRtc_InitBuffer(aecm->nearCleanFrameBuf);
    WebRtc_InitBuffer(aecm->outFrameBuf);

    memset(aecm->xBuf_buf, 0, sizeof(aecm->xBuf_buf));
    memset(aecm->dBufClean_buf, 0, sizeof(aecm->dBufClean_buf));
    memset(aecm->dBufNoisy_buf, 0, sizeof(aecm->dBufNoisy_buf));
    memset(aecm->outBuf_buf, 0, sizeof(aecm->outBuf_buf));

    aecm->seed = 666;
    aecm->totCount = 0;

    if (WebRtc_InitDelayEstimatorFarend(aecm->delay_estimator_farend) != 0) {
      return -1;
    }
    if (WebRtc_InitDelayEstimator(aecm->delay_estimator) != 0) {
      return -1;
    }
    // Set far end histories to zero
    memset(aecm->far_history, 0, sizeof(uint16_t) * PART_LEN1 * MAX_DELAY);
    memset(aecm->far_q_domains, 0, sizeof(int) * MAX_DELAY);
    aecm->far_history_pos = MAX_DELAY;

#ifdef USE_BINARY_SPECTRUM_BEFORE_IFFT_OUT
	aecm->corrGain=ONE_Q14;
#endif
	
	aecm->nearFarBitCountCorrHis = (uint32_t*)(((uintptr_t)aecm->nearFarBitCountCorrHisBuf+15)&~15);
	memset(aecm->nearFarBitCountCorrHis, 0, MAX_BUF_LEN*sizeof(uint32_t));	

#ifdef DELAY_ADJUSTING
	//aecm->nearEchoCandidateCorrHis = (int16_t*)(((uintptr_t)aecm->nearEchoCandidateCorr_buf+15)&~15);
	  memset(aecm->nearEchoCandidateCorrHis,0,PART_LEN*sizeof(int16_t));
#endif

   //aecm->nearFarEnRatioHis = (int16_t*)(((uintptr_t)aecm->nearFarEnRatio_buf+15)&~15);
   memset(&aecm->nearFarEnRatioHis[0],0,SPEC_BUF_LEN*sizeof(int16_t));
   memset(&aecm->farEnergyHis[0],0,MAX_BUF_LEN*sizeof(int32_t));   
   aecm->echoFarEnRatio=256;
   aecm->echoFarEnRatioValidFlag=0;
   aecm->echoFarEnRatioUpdateFlag=0;   
   aecm->nearFarCorr=0;     
   memset(&aecm->nearSpecHis[0],0,SPEC_BUF_LEN*PART_LEN1*sizeof(uint16_t));
   memset(&aecm->farSpecHis[0],0,SPEC_BUF_LEN*PART_LEN1*sizeof(uint16_t));  
   aecm->effectOutEnAvg=0;
   aecm->currentResidualEnAvg=0;
   aecm->nearEnavg=0;
   aecm->farEndScaleFactor=1024;
   //aecm->lastFarEndScaleFactor=1024;
   aecm->corrDtFlag = 0;

	memset(aecm->nearFarComplexCorr,0,PART_LEN1*sizeof(float));
	memset(aecm->errorFarCorr,0,PART_LEN1*sizeof(float));
	memset(aecm->nearPow,0,PART_LEN1*sizeof(float));
	memset(aecm->farPow,0,PART_LEN1*sizeof(float));
	memset(aecm->nearFarPow,0,PART_LEN1*sizeof(ComplexFloat));


	aecm->farBufferCnt=0;
	aecm->nearProcessCnt=0;

	
	aecm->cutHighFrequencyEnable=0;
    aecm->nlpFlag = 1;
    aecm->fixedDelay = -1;

    aecm->dfaCleanQDomain = 0;
    aecm->dfaCleanQDomainOld = 0;
    aecm->dfaNoisyQDomain = 0;
    aecm->dfaNoisyQDomainOld = 0;

    memset(aecm->nearLogEnergy, 0, sizeof(aecm->nearLogEnergy));    	
	aecm->farLogEnergy=0;
   // memset(aecm->farLogEnergyArray, 0, sizeof(aecm->farLogEnergyArray));
    memset(aecm->echoAdaptLogEnergy, 0, sizeof(aecm->echoAdaptLogEnergy));
    memset(aecm->echoStoredLogEnergy, 0, sizeof(aecm->echoStoredLogEnergy));

	
	memset(&aecm->echoStoredBackupLogEnergy[0], 0, sizeof(int16_t)*MAX_BUF_LEN);
	


    // Initialize the echo channels with a stored shape.
    if (samplingFreq == 8000)
    {
        WebRtcAecm_InitEchoPathCore(aecm, kChannelStored8kHz);
    }
    else
    {
        WebRtcAecm_InitEchoPathCore(aecm, kChannelStored16kHz);
    }
	aecm->farEndScaleFactorSpeakerOn=896;
	aecm->farEndScaleFactorSpeakerOff=896;
    memset(aecm->echoFilt, 0, sizeof(aecm->echoFilt));
    memset(aecm->nearFilt, 0, sizeof(aecm->nearFilt));
    aecm->noiseEstCtr = 0;

    aecm->cngMode = AecmTrue;

    memset(aecm->noiseEstTooLowCtr, 0, sizeof(aecm->noiseEstTooLowCtr));
    memset(aecm->noiseEstTooHighCtr, 0, sizeof(aecm->noiseEstTooHighCtr));
    // Shape the initial noise level to an approximate pink noise.
    for (i = 0; i < (PART_LEN1 >> 1) - 1; i++)
    {
        aecm->noiseEst[i] = (tmp32 << 8);
        tmp16--;
        tmp32 -= (int32_t)((tmp16 << 1) + 1);
    }
    for (; i < PART_LEN1; i++)
    {
        aecm->noiseEst[i] = (tmp32 << 8);
    }

    aecm->farEnergyMin = WEBRTC_SPL_WORD16_MAX;
    aecm->farEnergyMax = WEBRTC_SPL_WORD16_MIN;
    aecm->farEnergyMaxMin = 0;
    aecm->farEnergyVAD = FAR_ENERGY_MIN; // This prevents false speech detection at the
                                         // beginning.
    aecm->farEnergyMSE = 0;
    aecm->currentVADValue = 0;
    aecm->vadUpdateCount = 0;
    aecm->firstVAD = 1;

    aecm->startupState = 0;
    aecm->supGain = SUPGAIN_DEFAULT;
    aecm->supGainOld = SUPGAIN_DEFAULT;

    aecm->supGainErrParamA = SUPGAIN_ERROR_PARAM_A;
    aecm->supGainErrParamD = SUPGAIN_ERROR_PARAM_D;
    aecm->supGainErrParamDiffAB = SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B;
    aecm->supGainErrParamDiffBD = SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D;
	aecm->stream_has_echo = 0;//added by chgx
	aecm->AecmMode = 0;//add by wuzhaoyang
	aecm->nlpLevel=1;
	aecm->tailLenMs=128;

	aecm->afcEnable =1;
#ifdef AFC
	afcInit(&aecm->afcInstance);
#endif
	memset(&aecm->AecmModeArray[0],0,1024*sizeof(int16_t));
	aecm->fromReceiverToSpeaker=0;
	aecm->fromSpeakerToReceiver=0;
	aecm->speakerGain=1;


    // Assert a preprocessor definition at compile-time. It's an assumption
    // used in assembly code, so check the assembly files before any change.
    //COMPILE_ASSERT(PART_LEN % 16 == 0);

    // Initialize function pointers.
    WebRtcAecm_CalcLinearEnergies = CalcLinearEnergiesC;
    WebRtcAecm_StoreAdaptiveChannel = StoreAdaptiveChannelC;
    WebRtcAecm_ResetAdaptiveChannel = ResetAdaptiveChannelC;
	aecm->farFrameDropCnt =0;
#ifdef WEBRTC_DETECT_ARM_NEON
    uint64_t features = WebRtc_GetCPUFeaturesARM();
    if ((features & kCPUFeatureNEON) != 0)
    {
      WebRtcAecm_InitNeon();
    }
#elif defined(WEBRTC_ARCH_ARM_NEON) || defined(WEBRTC_ARCH_ARM64_NEON)
    WebRtcAecm_InitNeon();
#endif

#if defined(MIPS32_LE)
    WebRtcAecm_InitMips();
#endif
    return 0;
}

// TODO(bjornv): This function is currently not used. Add support for these
// parameters from a higher level
int WebRtcAecm_Control(AecmCore* aecm, int delay, int nlpFlag) {
    aecm->nlpFlag = nlpFlag;
    aecm->fixedDelay = delay;

    return 0;
}

int WebRtcAecm_FreeCore(AecmCore* aecm) {
    if (aecm == NULL)
    {
        return -1;
    }

    WebRtc_FreeBuffer(aecm->farFrameBuf);
    WebRtc_FreeBuffer(aecm->nearNoisyFrameBuf);
    WebRtc_FreeBuffer(aecm->nearCleanFrameBuf);
    WebRtc_FreeBuffer(aecm->outFrameBuf);

    WebRtc_FreeDelayEstimator(aecm->delay_estimator);
    WebRtc_FreeDelayEstimatorFarend(aecm->delay_estimator_farend);
    WebRtcSpl_FreeRealFFT(aecm->real_fft);

    free(aecm);

    return 0;
}

int WebRtcAecm_ProcessFrame(AecmCore* aecm,
                            const int16_t* farend,
                            const int16_t* nearendNoisy,
                            const int16_t* nearendClean,
                            int16_t* out) {
    int16_t outBlock_buf[PART_LEN + 8]; // Align buffer to 8-byte boundary.
    int16_t* outBlock = (int16_t*) (((uintptr_t) outBlock_buf + 15) & ~ 15);

    int16_t farFrame[FRAME_LEN];
    const int16_t* out_ptr = NULL;
    int size = 0;

    // Buffer the current frame.
    // Fetch an older one corresponding to the delay.
    WebRtcAecm_BufferFarFrame(aecm, farend, FRAME_LEN);
    WebRtcAecm_FetchFarFrame(aecm, farFrame, FRAME_LEN, aecm->knownDelay);

    // Buffer the synchronized far and near frames,
    // to pass the smaller blocks individually.
    WebRtc_WriteBuffer(aecm->farFrameBuf, farFrame, FRAME_LEN);
    WebRtc_WriteBuffer(aecm->nearNoisyFrameBuf, nearendNoisy, FRAME_LEN);
    if (nearendClean != NULL)
    {
        WebRtc_WriteBuffer(aecm->nearCleanFrameBuf, nearendClean, FRAME_LEN);
    }

    // Process as many blocks as possible.
    while (WebRtc_available_read(aecm->farFrameBuf) >= PART_LEN)
    {
        int16_t far_block[PART_LEN];
        const int16_t* far_block_ptr = NULL;
        int16_t near_noisy_block[PART_LEN];
        const int16_t* near_noisy_block_ptr = NULL;

        WebRtc_ReadBuffer(aecm->farFrameBuf, (void**) &far_block_ptr, far_block,
                          PART_LEN);
        WebRtc_ReadBuffer(aecm->nearNoisyFrameBuf,
                          (void**) &near_noisy_block_ptr,
                          near_noisy_block,
                          PART_LEN);
        if (nearendClean != NULL)
        {
            int16_t near_clean_block[PART_LEN];
            const int16_t* near_clean_block_ptr = NULL;

            WebRtc_ReadBuffer(aecm->nearCleanFrameBuf,
                              (void**) &near_clean_block_ptr,
                              near_clean_block,
                              PART_LEN);
            if (WebRtcAecm_ProcessBlock(aecm,
                                        far_block_ptr,
                                        near_noisy_block_ptr,
                                        near_clean_block_ptr,
                                        outBlock) == -1)
            {
                return -1;
            }
        } else
        {
            if (WebRtcAecm_ProcessBlock(aecm,
                                        far_block_ptr,
                                        near_noisy_block_ptr,
                                        NULL,
                                        outBlock) == -1)
            {
                return -1;
            }
        }

        WebRtc_WriteBuffer(aecm->outFrameBuf, outBlock, PART_LEN);
    }

    // Stuff the out buffer if we have less than a frame to output.
    // This should only happen for the first frame.
    size = (int) WebRtc_available_read(aecm->outFrameBuf);
    if (size < FRAME_LEN)
    {
        WebRtc_MoveReadPtr(aecm->outFrameBuf, size - FRAME_LEN);
    }

    // Obtain an output frame.
    WebRtc_ReadBuffer(aecm->outFrameBuf, (void**) &out_ptr, out, FRAME_LEN);
    if (out_ptr != out) {
      // ReadBuffer() hasn't copied to |out| in this case.
      memcpy(out, out_ptr, FRAME_LEN * sizeof(int16_t));
    }

    return 0;
}

// WebRtcAecm_AsymFilt(...)
//
// Performs asymmetric filtering.
//
// Inputs:
//      - filtOld       : Previous filtered value.
//      - inVal         : New input value.
//      - stepSizePos   : Step size when we have a positive contribution.
//      - stepSizeNeg   : Step size when we have a negative contribution.
//
// Output:
//
// Return: - Filtered value.
//
int16_t WebRtcAecm_AsymFilt(const int16_t filtOld, const int16_t inVal,
                            const int16_t stepSizePos,
                            const int16_t stepSizeNeg)
{
    int16_t retVal;

    if ((filtOld == WEBRTC_SPL_WORD16_MAX) | (filtOld == WEBRTC_SPL_WORD16_MIN))
    {
        return inVal;
    }
    retVal = filtOld;
    if (filtOld > inVal)
    {
        retVal -= (filtOld - inVal) >> stepSizeNeg;
    } else
    {
        retVal += (inVal - filtOld) >> stepSizePos;
    }

    return retVal;
}

// ExtractFractionPart(a, zeros)
//
// returns the fraction part of |a|, with |zeros| number of leading zeros, as an
// int16_t scaled to Q8. There is no sanity check of |a| in the sense that the
// number of zeros match.
static int16_t ExtractFractionPart(uint32_t a, int zeros) {
  return (int16_t)(((a << zeros) & 0x7FFFFFFF) >> 23);
}

// Calculates and returns the log of |energy| in Q8. The input |energy| is
// supposed to be in Q(|q_domain|).
int16_t LogOfEnergyInQ8(uint32_t energy, int q_domain) {
  static const int16_t kLogLowValue = PART_LEN_SHIFT << 7; 
  int16_t log_energy_q8 = kLogLowValue;
  if (energy > 0) {
    int zeros = WebRtcSpl_NormU32(energy);
    int16_t frac = ExtractFractionPart(energy, zeros);
    // log2 of |energy| in Q8.
    log_energy_q8 += ((31 - zeros) << 8) + frac - (q_domain << 8);
  }
  return log_energy_q8;
}

// WebRtcAecm_CalcEnergies(...)
//
// This function calculates the log of energies for nearend, farend and estimated
// echoes. There is also an update of energy decision levels, i.e. internal VAD.
//
//
// @param  aecm         [i/o]   Handle of the AECM instance.
// @param  far_spectrum [in]    Pointer to farend spectrum.
// @param  far_q        [in]    Q-domain of farend spectrum.
// @param  nearEner     [in]    Near end energy for current block in
//                              Q(aecm->dfaQDomain).
// @param  echoEst      [out]   Estimated echo in Q(xfa_q+RESOLUTION_CHANNEL16).
//

void WebRtcAecm_CalcEnergies(AecmCore* aecm,
                             const uint16_t* far_spectrum,
                             const int16_t far_q,
                             const uint32_t nearEner,
                             int32_t* echoEst) {
    // Local variables
    uint32_t tmpAdapt = 0;
    uint32_t tmpStored = 0;
    uint32_t tmpFar = 0;
	uint32_t tmpStoredBackup;
	int32_t nearFarEnRatio; 
    int i;
    int16_t tmp16;
    int16_t increase_max_shifts = 2;//0709  before chgx change ,the next four parameters is 4  11 11 3
    int16_t decrease_max_shifts = 11;
    int16_t increase_min_shifts = 11;
    int16_t decrease_min_shifts = 2;


    // Get log of near end energy and store in buffer

    // Shift buffer
    memmove(aecm->nearLogEnergy + 1, aecm->nearLogEnergy,
            sizeof(int16_t) * (MAX_BUF_LEN - 1));

    // Logarithm of integrated magnitude spectrum (nearEner)
    aecm->nearLogEnergy[0] = LogOfEnergyInQ8(nearEner, aecm->dfaNoisyQDomain);
	CalcEchoStoreBackupEn( aecm, far_spectrum,&tmpStoredBackup);
    WebRtcAecm_CalcLinearEnergies(aecm, far_spectrum, echoEst, &tmpFar, &tmpAdapt, &tmpStored);


    // Shift buffers
    memmove(aecm->echoAdaptLogEnergy + 1, aecm->echoAdaptLogEnergy,
            sizeof(int16_t) * (MAX_BUF_LEN - 1));
    memmove(aecm->echoStoredLogEnergy + 1, aecm->echoStoredLogEnergy,
            sizeof(int16_t) * (MAX_BUF_LEN - 1));
	
	memmove(aecm->echoStoredBackupLogEnergy+1,aecm->echoStoredBackupLogEnergy,sizeof(int16_t) * (MAX_BUF_LEN - 1));
	aecm->echoStoredBackupLogEnergy[0] = LogOfEnergyInQ8(tmpStoredBackup,RESOLUTION_CHANNEL16 + far_q);	


    // Logarithm of delayed far end energy	
    aecm->farLogEnergy = LogOfEnergyInQ8(tmpFar, far_q);
	//memmove(aecm->farLogEnergyArray+1,aecm->farLogEnergyArray,(MAX_BUF_LEN-1)*sizeof(int16_t));
	//aecm->farLogEnergyArray[0]=aecm->farLogEnergy;


    // Logarithm of estimated echo energy through adapted channel
    aecm->echoAdaptLogEnergy[0] = LogOfEnergyInQ8(tmpAdapt,RESOLUTION_CHANNEL16 + far_q);

    // Logarithm of estimated echo energy through stored channel
    aecm->echoStoredLogEnergy[0] =
        LogOfEnergyInQ8(tmpStored, RESOLUTION_CHANNEL16 + far_q);


  //calculate the energy ratio between near end and far end
	//save near end and far end energy
	memmove(aecm->nearEnergyHis + 1, aecm->nearEnergyHis,sizeof(int16_t) * (MAX_BUF_LEN - 1));
	memmove(aecm->farEnergyHis + 1, aecm->farEnergyHis,sizeof(int16_t) * (MAX_BUF_LEN - 1));
	aecm->nearEnergyHis[0] = nearEner>>( aecm->dfaCleanQDomain);
	aecm->farEnergyHis[0] = tmpFar>>far_q;		
	//calculate the energy ratio between near end and far end
	nearFarEnRatio = aecm->nearEnergyHis[0]*1024;	
	nearFarEnRatio = nearFarEnRatio/(aecm->farEnergyHis[0]+32);	
	if(nearFarEnRatio>4096)
	{
		nearFarEnRatio=4096;
	}
	//save the energy ratio between near end and far end
	memmove(aecm->nearFarEnRatioHis + 1, aecm->nearFarEnRatioHis,sizeof(int16_t) * (SPEC_BUF_LEN - 1));
	aecm->nearFarEnRatioHis[0] = nearFarEnRatio;	 




    // Update farend energy levels (min, max, vad, mse)
    if (aecm->farLogEnergy > FAR_ENERGY_MIN)
    {
        if (aecm->startupState == 0)
        {
            increase_max_shifts = 2;
            decrease_min_shifts = 2;
            increase_min_shifts = 8;
        }

        aecm->farEnergyMin = WebRtcAecm_AsymFilt(aecm->farEnergyMin, aecm->farLogEnergy,
                                                 increase_min_shifts, decrease_min_shifts);
        aecm->farEnergyMax = WebRtcAecm_AsymFilt(aecm->farEnergyMax, aecm->farLogEnergy,
                                                 increase_max_shifts, decrease_max_shifts);
        aecm->farEnergyMaxMin = (aecm->farEnergyMax - aecm->farEnergyMin);

        // Dynamic VAD region size
        tmp16 = 2560 - aecm->farEnergyMin;
        if (tmp16 > 0)
        {
          tmp16 = (int16_t)((tmp16 * FAR_ENERGY_VAD_REGION) >> 9);
        } else
        {
            tmp16 = 0;
        }
        tmp16 += FAR_ENERGY_VAD_REGION;


        if ((aecm->startupState == 0) | (aecm->vadUpdateCount > 128))//0708 chgx change 1024 t0 128           
        {
            // In startup phase or VAD update halted
            aecm->farEnergyVAD = aecm->farEnergyMin + tmp16;
        } else
        {
            if (aecm->farEnergyVAD > aecm->farLogEnergy)
            {
                aecm->farEnergyVAD +=
                    (aecm->farLogEnergy + tmp16 - aecm->farEnergyVAD) >> 6;
                aecm->vadUpdateCount = 0;
            } else
            {
                aecm->vadUpdateCount++;
            }
        }
        // Put MSE threshold higher than VAD
        aecm->farEnergyMSE = aecm->farEnergyVAD + (1 << 8);
    }

    // Update VAD variables
    if (aecm->farLogEnergy > aecm->farEnergyVAD)
    {
       

        if ((aecm->startupState == 0) | (aecm->farEnergyMaxMin > FAR_ENERGY_DIFF))
        {
            // We are in startup or have significant dynamics in input speech level
            aecm->currentVADValue = 1;
        }
    }
    else
    {
        aecm->currentVADValue = 0;
    }
    
    if ((aecm->currentVADValue) && (aecm->firstVAD))
    {
        aecm->firstVAD = 0;
        if (aecm->echoAdaptLogEnergy[0] > aecm->nearLogEnergy[0])
        {
            // The estimated echo has higher energy than the near end signal.
            // This means that the initialization was too aggressive. Scale
            // down by a factor 8
            for (i = 0; i < PART_LEN1; i++)
            {
                aecm->channelAdapt16[i] >>= 3;
            }
            // Compensate the adapted echo energy level accordingly.
            aecm->echoAdaptLogEnergy[0] -= (3 << 8);
            aecm->firstVAD = 1;
        }
    }
}
int16_t WebRtcAecm_calcCorrQ14(uint16_t* d0,uint16_t*d1,int16_t q0,int16_t q1,int16_t len) 
{	
	float auto0=0;
	float auto1=0;
	float corr=0;
	float tmp;
	int16_t i;	
	float value16_0,value16_1;	
	

	for(i=0;i<len;i++)
	{
		value16_0 = (d0[i]>>q0);
		tmp=1.0f*value16_0*value16_0;
		auto0 +=tmp;
		value16_1 = (d1[i]>>q1);
		tmp=1.0f*value16_1*value16_1;
		auto1+=tmp; 
		tmp=1.0f*value16_0*value16_1;
		corr +=tmp;
	}
	corr = corr*ONE_Q14+1e-10;
	corr = corr/sqrtf(1.0f*auto0*auto1+1e-10);
	corr = (int16_t)corr;
	return (int16_t)corr;
}

// WebRtcAecm_CalcStepSize(...)
//
// This function calculates the step size used in channel estimation
//
//
// @param  aecm  [in]    Handle of the AECM instance.
// @param  mu    [out]   (Return value) Stepsize in log2(), i.e. number of shifts.
//
//
int16_t WebRtcAecm_CalcStepSize(AecmCore* const aecm) {
    int32_t tmp32;
    int16_t tmp16;
    int16_t mu = MU_MAX;

    // Here we calculate the step size mu used in the
    // following NLMS based Channel estimation algorithm
    if (!aecm->currentVADValue)
    {
        // Far end energy level too low, no channel update
        mu = 0;
    } else if (aecm->startupState > 0)
    {
        if (aecm->farEnergyMin >= aecm->farEnergyMax)
        {
            mu = MU_MIN;
        } else
        {
            tmp16 = (aecm->farLogEnergy - aecm->farEnergyMin);
            tmp32 = tmp16 * MU_DIFF;
            tmp32 = WebRtcSpl_DivW32W16(tmp32, aecm->farEnergyMaxMin);
            mu = MU_MIN - 1 - (int16_t)(tmp32);
            // The -1 is an alternative to rounding. This way we get a larger
            // stepsize, so we in some sense compensate for truncation in NLMS
        }
        if (mu < MU_MAX)
        {
            mu = MU_MAX; // Equivalent with maximum step size of 2^-MU_MAX
        }
    }

    return mu;
}

// WebRtcAecm_UpdateChannel(...)
//
// This function performs channel estimation. NLMS and decision on channel storage.
//
//
// @param  aecm         [i/o]   Handle of the AECM instance.
// @param  far_spectrum [in]    Absolute value of the farend signal in Q(far_q)
// @param  far_q        [in]    Q-domain of the farend signal
// @param  dfa          [in]    Absolute value of the nearend signal (Q[aecm->dfaQDomain])
// @param  mu           [in]    NLMS step size.
// @param  echoEst      [i/o]   Estimated echo in Q(far_q+RESOLUTION_CHANNEL16).
//
int16_t WebRtcAecm_UpdateChannel(AecmCore* aecm,
                              const uint16_t* far_spectrum,
                              const int16_t far_q,
                              const uint16_t* const dfa,
                              int16_t mu,
                              int32_t* echoEst) {
    uint32_t tmpU32no1, tmpU32no2;
    int32_t tmp32no1, tmp32no2;
    int32_t mseStored;//MSE of stored Channel
    int32_t mseAdapt;//MSE of  adaptive channel

	int32_t mseStoredBackup=0;	// MSE of store backup channel
	int16_t mseStoreTooBig=0;// this flag tell us whether the mse of stored channel is too big 
	int32_t nearLogSum=0.0f;
	//int32_t farLogSum=0.0f;
	int16_t mseAdaptiveTooBig=0;// this flag tell us whether the mse of adaptive channel is too big 	
	int32_t mseAdaptPostHalf=0; // this value is  the sum of mse of adaptive channel in the post half blocks 
	int32_t diffStoredBackup=0;
	int32_t diffStored=0;
	int16_t updateFromStoreCandidate=0;
	int16_t echoBiggerThanNearCnt=0;
    int i;
    int16_t zerosFar, zerosNum, zerosCh, zerosDfa;
    int16_t shiftChFar, shiftNum, shift2ResChan;
    int16_t tmp16no1;
    int16_t xfaQ, dfaQ;
	int16_t channelUpdate=0;
    // This is the channel estimation algorithm. It is base on NLMS but has a variable step
    // length, which was calculated above.
	if (mu)    
    {
        for (i = 0; i < PART_LEN1; i++)
        {
            // Determine norm of channel and farend to make sure we don't get overflow in
            // multiplication
            zerosCh = WebRtcSpl_NormU32(aecm->channelAdapt32[i]);
            zerosFar = WebRtcSpl_NormU32((uint32_t)far_spectrum[i]);
            if (zerosCh + zerosFar > 31)
            {
                // Multiplication is safe
                tmpU32no1 = WEBRTC_SPL_UMUL_32_16(aecm->channelAdapt32[i],
                        far_spectrum[i]);
                shiftChFar = 0;
            } else
            {
                // We need to shift down before multiplication
                shiftChFar = 32 - zerosCh - zerosFar;
                tmpU32no1 = (aecm->channelAdapt32[i] >> shiftChFar) *
                    far_spectrum[i];
            }
            // Determine Q-domain of numerator
            zerosNum = WebRtcSpl_NormU32(tmpU32no1);
            if (dfa[i])
            {
                zerosDfa = WebRtcSpl_NormU32((uint32_t)dfa[i]);
            } else
            {
                zerosDfa = 32;
            }
            tmp16no1 = zerosDfa - 2 + aecm->dfaNoisyQDomain -
                RESOLUTION_CHANNEL32 - far_q + shiftChFar;
            if (zerosNum > tmp16no1 + 1)
            {
                xfaQ = tmp16no1;
                dfaQ = zerosDfa - 2;
            } else
            {
                xfaQ = zerosNum - 2;
                dfaQ = RESOLUTION_CHANNEL32 + far_q - aecm->dfaNoisyQDomain -
                    shiftChFar + xfaQ;
            }
            // Add in the same Q-domain
            tmpU32no1 = WEBRTC_SPL_SHIFT_W32(tmpU32no1, xfaQ);
            tmpU32no2 = WEBRTC_SPL_SHIFT_W32((uint32_t)dfa[i], dfaQ);
            tmp32no1 = (int32_t)tmpU32no2 - (int32_t)tmpU32no1;
            zerosNum = WebRtcSpl_NormW32(tmp32no1);
            if ((tmp32no1) && (far_spectrum[i] > (CHANNEL_VAD << far_q)))
            {
                //
                // Update is needed
                //
                // This is what we would like to compute
                //
                // tmp32no1 = dfa[i] - (aecm->channelAdapt[i] * far_spectrum[i])
                // tmp32norm = (i + 1)
                // aecm->channelAdapt[i] += (2^mu) * tmp32no1
                //                        / (tmp32norm * far_spectrum[i])
                //

                // Make sure we don't get overflow in multiplication.
                if (zerosNum + zerosFar > 31)
                {
                    if (tmp32no1 > 0)
                    {
                        tmp32no2 = (int32_t)WEBRTC_SPL_UMUL_32_16(tmp32no1,
                                                                        far_spectrum[i]);
                    } else
                    {
                        tmp32no2 = -(int32_t)WEBRTC_SPL_UMUL_32_16(-tmp32no1,
                                                                         far_spectrum[i]);
                    }
                    shiftNum = 0;
                } else
                {
                    shiftNum = 32 - (zerosNum + zerosFar);
                    if (tmp32no1 > 0)
                    {
                        tmp32no2 = (tmp32no1 >> shiftNum) * far_spectrum[i];
                    } else
                    {
                        tmp32no2 = -((-tmp32no1 >> shiftNum) * far_spectrum[i]);
                    }
                }
                // Normalize with respect to frequency bin
                tmp32no2 = WebRtcSpl_DivW32W16(tmp32no2, i + 1);

                // Make sure we are in the right Q-domain

                shift2ResChan = shiftNum + shiftChFar - xfaQ - mu - ((30 - zerosFar) << 1);

				
                if (WebRtcSpl_NormW32(tmp32no2) < shift2ResChan)
                {
                    tmp32no2 = WEBRTC_SPL_WORD32_MAX;
                } else
                {
                    tmp32no2 = WEBRTC_SPL_SHIFT_W32(tmp32no2, shift2ResChan);
                }
				/*if(tmp32no2>1.5e-2f*(1<<RESOLUTION_CHANNEL32)&&(aecm->speakerGain<1.05f))
				{
					tmp32no2=1.5e-2f*(1<<RESOLUTION_CHANNEL32);
				}*/
                aecm->channelAdapt32[i] =WebRtcSpl_AddSatW32(aecm->channelAdapt32[i], tmp32no2);
                if (aecm->channelAdapt32[i] < 0)
                {
                    // We can never have negative channel gain
                    aecm->channelAdapt32[i] = 0;
                }
                aecm->channelAdapt16[i] =(int16_t)(aecm->channelAdapt32[i] >> 16);

            }
        }
    }
    // END: Adaptive channel update


	aecm->mseStoredOld = ((int32_t)aecm->echoStoredLogEnergy[0] - (int32_t)aecm->nearLogEnergy[0]);
	{
		int16_t i;		
		int32_t tmpCurBand;
		for(i=0;i<PART_LEN1;i++)
		{
		    echoEst[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStored[i],far_spectrum[i]);
			tmpCurBand =LogOfEnergyInQ8(echoEst[i],RESOLUTION_CHANNEL16 + far_q)-LogOfEnergyInQ8(dfa[i],aecm->dfaNoisyQDomain);
			if(tmpCurBand<0)
			{
				tmpCurBand = -(-tmpCurBand)>>1;
			}			
			aecm->mseStoredBand[i]=tmpCurBand;//save the mse of stored channel in every band
		}		
	}

	
	if(aecm->mseStoredOld<0)
	{
		aecm->mseStoredOld = -((-aecm->mseStoredOld)>>1);
	}
    // Determine if we should store or restore the channel
    if ((aecm->startupState == 0) & (aecm->currentVADValue))
    {
        // During startup we store the channel every block,
        // and we recalculate echo estimate
        WebRtcAecm_StoreAdaptiveChannel(aecm, far_spectrum, echoEst);
		aecm->mseChannelCntClearCnt=0;
    } 
	else
    {
		aecm->channelAdaptCorr = WebRtcAecm_calcCorrQ14((uint16_t*)aecm->channelAdaptOld16,(uint16_t*)aecm->channelAdapt16,0,0,PART_LEN1);		
		aecm->farPreNowFrameCorr =  WebRtcAecm_calcCorrQ14((uint16_t*)(aecm->farSpecHis),(uint16_t*)(aecm->farSpecHis+PART_LEN1),0,0,PART_LEN1);
		memcpy(aecm->channelAdaptOld16,aecm->channelAdapt16,PART_LEN1*sizeof(int16_t));
		memcpy(aecm->channelAdaptOld32,aecm->channelAdapt32,PART_LEN1*sizeof(int32_t));
		if((aecm->farPreNowFrameCorr>aecm->channelAdaptCorr+400*aecm->speakerGain)&&(aecm->mseChannelCntClearCnt<=13*aecm->mult))
		{
			aecm->corrDtFlag = ONE_Q14;//double talk
			aecm->mseChannelCount = 0;//reset mseChannelCount
		}
		else if((ONE_Q14==aecm->corrDtFlag)&&(aecm->farPreNowFrameCorr>aecm->channelAdaptCorr+200*aecm->speakerGain)&&(aecm->mseChannelCntClearCnt<=13*aecm->mult))
		{//condition explain: double talk happens in last block ;  adaptive channel change faster than far end signal            mseChannelCntClearCnt is small
			aecm->corrDtFlag = ONE_Q14;
			aecm->mseChannelCount = 0;
		}
		else if(aecm->mseChannelCount>=(MIN_MSE_COUNT-1))
		{
			aecm->corrDtFlag = 0;
		}
        if (aecm->farLogEnergy < aecm->farEnergyMSE)// check if the far end signal is big enough
        {
            aecm->mseChannelCount = 0;// far end signal is too small, clear aecm->mseChannelCount
        } 		
		
		else
        {
            aecm->mseChannelCount++;
        }
        
        // Check if remote signal stable
		if(aecm->mseChannelCount>=2)
		{
			int16_t m;
			int32_t maxFarEn=aecm->farEnergyHis[0];
			int32_t minFarEn=aecm->farEnergyHis[0];
			int16_t unStableCnt=0;
			for(m=1;m<aecm->mseChannelCount;m++)
			{
				// calculate the max value and min value of far end signal in the last  aecm->mseChannelCount blocks
				if(maxFarEn<aecm->farEnergyHis[m])
				{
					maxFarEn=aecm->farEnergyHis[m];
				}
				else if(minFarEn>aecm->farEnergyHis[m])
				{
					minFarEn=aecm->farEnergyHis[m];
				}				
			}
			for(m=0;m<aecm->mseChannelCount;m++)
			{				
				if(maxFarEn>aecm->farEnergyHis[m]*4)
				{
					unStableCnt++;//increase the risk of  far end unstable
					if(unStableCnt*4>MIN_MSE_COUNT)
					{
						if(aecm->mseChannelCount*2>MIN_MSE_COUNT)
						{
							aecm->mseChannelCntClearCnt++;// increase the clear cnt
						}
						aecm->mseChannelCount = m;	//reset aecm->mseChannelCount to m					
						break;
					}
				}
			}
		}
		

        // If mseChannelCntClearCnt too larger, do not clear mseChannelCount by large nearFarEnRatioHis value
		if((aecm->mseChannelCount>=MIN_MSE_COUNT)&&(aecm->mseChannelCntClearCnt<=13*aecm->mult))
		{
			int16_t enRatioThreshold;
			int16_t m;
			int bigCnt=0;
			if(aecm->AecmMode<=3)
			{
				enRatioThreshold =1536+aecm->echoFarEnRatio;
			}
			else
			{
				enRatioThreshold =2048+aecm->echoFarEnRatio;
			}			
			for(m=0;m<aecm->mseChannelCount;m++)
			{
				if(aecm->nearFarEnRatioHis[m]>enRatioThreshold)//check if the energy ratio between near and far signal is too big
				{
					bigCnt++;//  near signal is much bigger than far signal
					if(bigCnt*4>MIN_MSE_COUNT)
					{
						aecm->mseChannelCount = 0;

		                aecm->mseChannelCntClearCnt++;

						break;
					}
				}
			}	
		}
		
		//if we reach here,that means that: in the past MIN_MSE_COUNT blocks,far signal is stable and the energy ratio between near and far is reasonable
		if(aecm->mseChannelCount>=MIN_MSE_COUNT)
		{
			aecm->mseChannelCntClearCnt=0;
		}

		//update the aecm->speakerGain
        if((aecm->fromReceiverToSpeaker==1)&&(aecm->mseChannelCount*2>=MIN_MSE_COUNT))
		{				
			if(aecm->speakerGain>1)
			{
				aecm->speakerGain = aecm->speakerGain*0.9f+0.1f;
			}
			if(aecm->speakerGain<1.0f)
			{
				aecm->speakerGain = 1.0f;
			}
		}
	
        // Enough data for validation. Store channel if we can.
		if (aecm->mseChannelCount>=MIN_MSE_COUNT)
        {
			int16_t curMseCount=aecm->mseChannelCount;			
			if(curMseCount>MIN_MSE_COUNT)
			{
				curMseCount=MIN_MSE_COUNT;
			}
			channelUpdate +=curMseCount*100;//this value is for debug,do not mind it.
            // We have enough data.
            // Calculate MSE of "Adapt" and "Stored" versions.
            // It is actually not MSE, but average absolute error.
			//channelUpdate= aecm->mseChannelCount*100;//chgx 
            mseStored = 0;
            mseAdapt = 0;
			mseStoredBackup =0;
			aecm->mseReach20Cnt++;

			for (i = 0; i < curMseCount; i++)
            {
				tmp32no1 = ((int32_t)aecm->echoAdaptLogEnergy[i] - (int32_t)aecm->nearLogEnergy[i]);
				if(tmp32no1<0)
				{
					tmp32no1 = -((-tmp32no1)>>1);
				}
                tmp32no2 = WEBRTC_SPL_ABS_W32(tmp32no1);
                mseAdapt += tmp32no2;

				if(i+i>curMseCount)
				{
					mseAdaptPostHalf+=tmp32no2;
				}
                tmp32no1 = ((int32_t)aecm->echoStoredLogEnergy[i] - (int32_t)aecm->nearLogEnergy[i]);
				if(tmp32no1<0)
				{
					tmp32no1 = -((-tmp32no1)>>1);
				}

				diffStored +=tmp32no1;
                tmp32no2 = WEBRTC_SPL_ABS_W32(tmp32no1);
                mseStored += tmp32no2;

				nearLogSum +=aecm->nearLogEnergy[i];				
				tmp32no1 = ((int32_t)aecm->echoStoredBackupLogEnergy[i] - (int32_t)aecm->nearLogEnergy[i]);
				if(tmp32no1<0)
				{
					tmp32no1 = -((-tmp32no1)>>1);
				}
				diffStoredBackup +=tmp32no1;
				if(tmp32no1>64)//check if the estimate echo spectrum is bigger than near spectrum.(if true,it means the the store backup channel is over estimated) 
				{
					echoBiggerThanNearCnt++;//calculate the number of bands where the estimate echo is bigger than near
				}
                tmp32no2 = WEBRTC_SPL_ABS_W32(tmp32no1);
                mseStoredBackup += tmp32no2;

            }
		
			//if(aecm->AecmMode==4)
			if(0)
			{
				if(mseStored>=curMseCount*32*aecm->speakerGain)//(mseStored>0.125f*nearLogSum*aecm->speakerGain)||
			    {
					mseStoreTooBig=ONE_Q14;					
			    }
				if(mseAdapt>=curMseCount*64*aecm->speakerGain)//(mseStored>0.125f*nearLogSum*aecm->speakerGain)||
			    {
					mseAdaptiveTooBig=ONE_Q14;					
			    }
			}
			else
			{	
				// check if the mse of stored channel is too big or not.
				if((mseStored>=(curMseCount*48+(aecm->mseStoredSmoothed>>2))*aecm->speakerGain))
			    {					
					mseStoreTooBig=ONE_Q14;					
			    }
				if(mseStored<96*curMseCount)
				{					
					float smoothFactorStore=1.0f;
					float newValue;
					smoothFactorStore = 1.0f*48*curMseCount/(mseStored+48*curMseCount);
					newValue=aecm->mseStoredSmoothed*(1.0f-smoothFactorStore)+mseStored*smoothFactorStore;
					if(newValue>32*curMseCount)
					{
						aecm->mseStoredSmoothed=newValue;
					}
				}

				// check if the mse of adaptive channel is too big or not.
				if((mseAdapt>=(curMseCount*64+(aecm->mseAdaptSmoothed>>2))*aecm->speakerGain))
			    {					
					mseAdaptiveTooBig=ONE_Q14;					
			    }
				if(mseAdapt<96*curMseCount)
				{
					float smoothFactorAdapt=1.0f;
					float newValue;
					smoothFactorAdapt = smoothFactorAdapt*48*curMseCount/(mseAdapt+48*curMseCount);
					newValue=aecm->mseAdaptSmoothed*(1.0f-smoothFactorAdapt)+mseAdapt*smoothFactorAdapt;	
					if(newValue>64*curMseCount)
					{
						aecm->mseAdaptSmoothed=newValue;
					}
				}
				if(mseAdaptPostHalf<=curMseCount*48*aecm->speakerGain)
				{
					mseAdaptiveTooBig=0;	
				}				
			}
		
			if(((mseStoredBackup>mseStored)&&(!mseStoreTooBig))//if true,it means: StoredBackup channel is worse than store channel, and the store channel is good
				||((mseStoredBackup>mseStored*2)&&(mseStored<=curMseCount*64*aecm->speakerGain))//if true,it means: StoredBackup channel is much worse than store channel, and the store channel is reasonable
				||((mseStoredBackup>mseStored)&&(aecm->mseStoredSmallerCnt>=4)&&(aecm->backupFilterUpdateTime+64*aecm->mult<aecm->totCount)))//if true,it means: StoredBackup channel is worse than store channel, and the store channel is reasonable
			{				
				if(((mseStoredBackup>mseStored)&&(!mseStoreTooBig))||((mseStoredBackup>mseStored*2)&&(mseStored<=curMseCount*64*aecm->speakerGain)))
				{
					// here we can comfirm that the performance of stored backup filter is worse than the stored filter
					int32_t tmpCurBand;
					// now we need to find in which band the performance of stored backup filter is worse than stored filter
					for(i=0;i<PART_LEN1;i++)
					{
						echoEst[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStoredBackup[i],far_spectrum[i]);
						tmpCurBand =LogOfEnergyInQ8(echoEst[i],RESOLUTION_CHANNEL16 + far_q)-LogOfEnergyInQ8(dfa[i],aecm->dfaNoisyQDomain);
						if(tmpCurBand<0)
						{
							tmpCurBand = -(-tmpCurBand)>>1;
						}	
						if(WEBRTC_SPL_ABS_W32(tmpCurBand)>WEBRTC_SPL_ABS_W32(aecm->mseStoredBand[i]))
						{
							if(aecm->nearFarComplexCorr[i]*aecm->speakerGain>0.3f)

							{
								aecm->channelStoredBackup[i]=aecm->channelStored[i];//update stored backup filter
							}
						}					
					}		
					
					aecm->backupFilterUpdateCnt++;
					//memcpy(aecm->channelStoredBackup,aecm->channelStored,PART_LEN1*sizeof(int16_t));// 
					memcpy(aecm->storedFilterCandidate,aecm->channelStoredBackup,PART_LEN1*sizeof(int16_t));// save it as a candidate 
					aecm->echoFarEnRatioNeed=1;
					aecm->echoFarEnRatioUpdateFlag=ONE_Q14;
					channelUpdate += ONE_Q14;
					diffStoredBackup = diffStored;
					aecm->mseStoredSmallerCnt=0;
					aecm->backupFilterUpdateTime=aecm->totCount;//save the update time
					memcpy(aecm->storedFilterCandidate,aecm->channelStoredBackup,PART_LEN1*sizeof(int16_t));
					aecm->minMseStored = mseStored;
				}
			    else
				{
					aecm->backupFilterUpdateCnt++;
					memcpy(aecm->channelStoredBackup,aecm->storedFilterCandidate,PART_LEN1*sizeof(int16_t));
					updateFromStoreCandidate=1;
					aecm->echoFarEnRatioNeed=1;
					aecm->echoFarEnRatioUpdateFlag=ONE_Q14;
					channelUpdate += ONE_Q14;
					diffStoredBackup = aecm->diffStoredCandidate;
					aecm->mseStoredSmallerCnt=0;
					aecm->backupFilterUpdateTime=aecm->totCount;
					//memcpy(aecm->storedFilterCandidate,aecm->channelStoredBackup,PART_LEN1*sizeof(int16_t));
					//aecm->minMseStored = mseStored;
				}				
			}
			else if(mseStoredBackup>mseStored)
			{
				aecm->mseStoredSmallerCnt++;
				if(mseStored<aecm->minMseStored)
				{
					aecm->minMseStored = mseStored;
					memcpy(aecm->storedFilterCandidate,aecm->channelStored,PART_LEN1*sizeof(int16_t));
				}			
			}
			else if((mseStored>=160*curMseCount)||(mseStoredBackup<mseStored-32*curMseCount))
			{
				aecm->mseStoredSmallerCnt=0;
			}
			else
			{
				aecm->mseStoredSmallerCnt++;
			}
			aecm->diffStoredBackupSmoothed += (diffStoredBackup-aecm->diffStoredBackupSmoothed)>>2;
			if(((aecm->diffStoredBackupSmoothed>curMseCount*32)||(echoBiggerThanNearCnt>=PART_LEN-16))&&(updateFromStoreCandidate!=1))
			{
				//estimate echo is bigger than near end signal,so we need to scale down the channelStoredBackup[i]
				int j;
				float downFactor = 1.0f/powf(2,1.0f*diffStoredBackup/curMseCount/256.0f);
				float farAvg=0.0f;
				float smoothFactor = (aecm->echoFarEnRatio-64)/256;
				if(smoothFactor<0.0f)
				{
					smoothFactor=0.0f;
				}
				else if(smoothFactor>1.0f)
				{
					smoothFactor=1.0f;
				}
				smoothFactor *=0.5f;				
				//downFactor = (1.0f+downFactor)*0.5f;//
				downFactor = 1.0f*(1.0f-smoothFactor)+downFactor*smoothFactor;
				for(j=0;j<PART_LEN1;j++)
				{
					farAvg+=far_spectrum[j];
				}
				farAvg = farAvg/PART_LEN1;
				for(j=0;j<PART_LEN1;j++)
				{
					if(far_spectrum[j]<4*farAvg)
					{
						aecm->channelStoredBackup[j]*=downFactor;
					}
				}
				channelUpdate*=-1;//debug value
			}
			aecm->mseStoredBackUpOld=mseStoredBackup;

			if((mseAdapt>mseStored)&&(!mseStoreTooBig))
			{
				ResetAdaptiveChannelC(aecm);
				aecm->mseChannelCount = 0;				
			}
			else if((mseAdapt<mseStored)&&(!mseAdaptiveTooBig))				
            {
                // The adaptive channel has a significantly lower MSE than the stored one.
                // The MSE for the adaptive channel has also been low for two consecutive
                // calculations. Store the adaptive channel.
				channelUpdate += ONE_Q14>>1;//debug value
				StoreAdaptiveChannelC(aecm,far_spectrum,echoEst);		

				if(1==aecm->backupFilterNeverUpdate)//
				{
					memcpy(aecm->channelStoredBackup,aecm->channelStored,PART_LEN1*sizeof(int16_t));//
				    channelUpdate =32767;
				    aecm->echoFarEnRatioNeed=1;
				    aecm->echoFarEnRatioUpdateFlag=ONE_Q14;				
					aecm->backupFilterNeverUpdate=0;
				}
                // Update threshold
                if (aecm->mseThreshold == WEBRTC_SPL_WORD32_MAX)
                {
                    aecm->mseThreshold = (mseAdapt + aecm->mseAdaptOld);
                } else
                {
                  int scaled_threshold = aecm->mseThreshold * 5 / 8;
                  aecm->mseThreshold += ((mseAdapt - scaled_threshold) * 205) >> 8;
                }
			}

            // Reset counter			
			aecm->mseChannelCount = 0;			
            // Store the MSE values.
            aecm->mseStoredOld = mseStored/MIN_MSE_COUNT;
			
            aecm->mseAdaptOld = mseAdapt/MIN_MSE_COUNT;
        }
    }
    // END: Determine if we should store or reset channel estimate.



	/* if the aecm->channelStoredBackup can not be updated for a long time, we try to check the nearFarComplexCorr.*/
	if((aecm->echoFarEnRatioNeed!=1)&&(aecm->mseChannelCntClearCnt>aecm->mult*6))
	{
		int16_t m;
		int16_t updateSomeBand=0;
		float corrAvg=0;
		for(m=12;m<44;m++)
		{
			corrAvg+=aecm->nearFarComplexCorr[m];
		}
		corrAvg/=32;
		for(m=0;m<PART_LEN1;m++)
		{
			if(((aecm->nearFarComplexCorr[m]>0.75f-0.005*m)&&(aecm->nearFarComplexCorr[m]>corrAvg+0.25f-0.005*m))||(aecm->nearFarComplexCorr[m]>0.9f))
			{
				aecm->channelStoredBackup[m]=aecm->channelAdapt16[m];
				aecm->channelStored[m]=aecm->channelAdapt16[m];
				updateSomeBand =1;
			}
		}
		if(updateSomeBand==1)
		{			
			aecm->backupFilterUpdateCnt++;					
			aecm->echoFarEnRatioNeed=1;
			aecm->echoFarEnRatioUpdateFlag=ONE_Q14;
			channelUpdate = -10000;					
			aecm->backupFilterUpdateTime=aecm->totCount;
		}
	}




	//use aecm->channelStoredBackup[i] to calculate echoEst[i]
	tmpU32no1 =0;
	for (i = 0; i < PART_LEN1; i ++)
	{
		echoEst[i] = WEBRTC_SPL_MUL_16_U16(aecm->channelStoredBackup[i],far_spectrum[i]);
		tmpU32no1+=echoEst[i];
	}
	aecm->echoStoredBackupLogEnergy[0] = LogOfEnergyInQ8(tmpU32no1,RESOLUTION_CHANNEL16 + far_q);	
	{
		float echoSum=0.0f;
		float farSum=0.0f;
		float echoScale = 1.0f/(1<<RESOLUTION_CHANNEL16);		
		for (i = 0; i*2 < PART_LEN1; i ++)
		{
			echoSum+=echoEst[i];
			farSum+=far_spectrum[i];		
		}
		echoSum = echoSum*echoScale;		
		aecm->echoFarEnRatioFromBackUp=1024*echoSum/(farSum+1e-10f);
	}
	


	return channelUpdate;
}

// CalcSuppressionGain(...)
//
// This function calculates the suppression gain that is used in the Wiener filter.
//
//
// @param  aecm     [i/n]   Handle of the AECM instance.
// @param  supGain  [out]   (Return value) Suppression gain with which to scale the noise
//                          level (Q14).
//
//
int16_t WebRtcAecm_CalcSuppressionGain(AecmCore* const aecm) {
    int32_t tmp32no1;

    int16_t supGain = SUPGAIN_DEFAULT;
    int16_t tmp16no1;
    int16_t dE = 0;

    // Determine suppression gain used in the Wiener filter. The gain is based on a mix of far
    // end energy and echo estimation error.
    // Adjust for the far end signal level. A low signal level indicates no far end signal,
    // hence we set the suppression gain to 0
    if (!aecm->currentVADValue)
    {
        supGain = 0;
    } 
	else if((aecm->corrDtFlag>0)&&(aecm->farPreNowFrameCorr>aecm->channelAdaptCorr+200+50*aecm->nlpLevel))
    {
		supGain = aecm->supGainErrParamD;//double talk,set supGain to ONE_Q8
    }
	else
    {
        // Adjust for possible double talk. If we have large variations in estimation error we
        // likely have double talk (or poor channel).

		tmp16no1 = (aecm->nearLogEnergy[0] - aecm->echoStoredBackupLogEnergy[0] - ENERGY_DEV_OFFSET);
        dE = WEBRTC_SPL_ABS_W16(tmp16no1);

        if (dE < ENERGY_DEV_TOL)
        {
            // Likely no double talk. The better estimation, the more we can suppress signal.
            // Update counters
            if (dE < SUPGAIN_EPC_DT)
            {
                tmp32no1 = aecm->supGainErrParamDiffAB * dE;
                tmp32no1 += (SUPGAIN_EPC_DT >> 1);
                tmp16no1 = (int16_t)WebRtcSpl_DivW32W16(tmp32no1, SUPGAIN_EPC_DT);
                supGain = aecm->supGainErrParamA - tmp16no1;
            } else
            {
                tmp32no1 = aecm->supGainErrParamDiffBD * (ENERGY_DEV_TOL - dE);
                tmp32no1 += ((ENERGY_DEV_TOL - SUPGAIN_EPC_DT) >> 1);
                tmp16no1 = (int16_t)WebRtcSpl_DivW32W16(tmp32no1, (ENERGY_DEV_TOL
                        - SUPGAIN_EPC_DT));
                supGain = aecm->supGainErrParamD + tmp16no1;
            }
        } else
        {
			// Likely in double talk. Use default value
            supGain = aecm->supGainErrParamD;
        }
    }


    if (supGain > aecm->supGainOld)
    {
        tmp16no1 = supGain;
    } else
    {
        tmp16no1 = aecm->supGainOld;
    }
    aecm->supGainOld = supGain;
    if (tmp16no1 < aecm->supGain)
    {
		aecm->supGain += (int16_t)((tmp16no1 - aecm->supGain) >> 4);
    } 
	else
    {
       aecm->supGain += (int16_t)((tmp16no1 - aecm->supGain) >> 4);
    }

    // END: Update suppression gain

    return aecm->supGain;
}

void WebRtcAecm_BufferFarFrame(AecmCore* const aecm,
                               const int16_t* farend,
                               const int farLen) {
    int writeLen = farLen, writePos = 0;


    // Check if the write position must be wrapped
    while (aecm->farBufWritePos + writeLen > FAR_BUF_LEN)
    {
        // Write to remaining buffer space before wrapping
        writeLen = FAR_BUF_LEN - aecm->farBufWritePos;
        memcpy(aecm->farBuf + aecm->farBufWritePos, farend + writePos,
               sizeof(int16_t) * writeLen);
        aecm->farBufWritePos = 0;
        writePos = writeLen;
        writeLen = farLen - writeLen;
    }

    memcpy(aecm->farBuf + aecm->farBufWritePos, farend + writePos,
           sizeof(int16_t) * writeLen);
    aecm->farBufWritePos += writeLen;
}

void WebRtcAecm_FetchFarFrame(AecmCore* const aecm,
                              int16_t* const farend,
                              const int farLen,
                              const int knownDelay) {
    int readLen = farLen;
    int readPos = 0;
    int delayChange = knownDelay - aecm->lastKnownDelay;

    aecm->farBufReadPos -= delayChange;

    // Check if delay forces a read position wrap
    while (aecm->farBufReadPos < 0)
    {
        aecm->farBufReadPos += FAR_BUF_LEN;
    }
    while (aecm->farBufReadPos > FAR_BUF_LEN - 1)
    {
        aecm->farBufReadPos -= FAR_BUF_LEN;
    }

    aecm->lastKnownDelay = knownDelay;

    // Check if read position must be wrapped
    while (aecm->farBufReadPos + readLen > FAR_BUF_LEN)
    {

        // Read from remaining buffer space before wrapping
        readLen = FAR_BUF_LEN - aecm->farBufReadPos;
        memcpy(farend + readPos, aecm->farBuf + aecm->farBufReadPos,
               sizeof(int16_t) * readLen);
        aecm->farBufReadPos = 0;
        readPos = readLen;
        readLen = farLen - readLen;
    }
    memcpy(farend + readPos, aecm->farBuf + aecm->farBufReadPos,
           sizeof(int16_t) * readLen);
    aecm->farBufReadPos += readLen;
}
