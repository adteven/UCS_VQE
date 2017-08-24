/*
 *  Copyright (c) 2013 The WebRTC project authors. All Rights Reserved.
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

#include "webrtc/common_audio/ring_buffer.h"
#include "webrtc/common_audio/signal_processing/include/real_fft.h"
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/utility/delay_estimator_wrapper.h"
#include "webrtc/system_wrappers/interface/compile_assert_c.h"
#include "webrtc/system_wrappers/interface/cpu_features_wrapper.h"
#include "webrtc/typedefs.h"
#include "webrtc/modules/audio_processing/utility/delay_estimator_internal.h"//chgx:add this header file to calc nearFarBitCountCorr;
#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"

#include"stdio.h"
#include<math.h>

// Square root of Hanning window in Q14.
#if defined(WEBRTC_DETECT_ARM_NEON) || defined(WEBRTC_ARCH_ARM_NEON)
// Table is defined in an ARM assembly file.
extern const ALIGN8_BEG int16_t WebRtcAecm_kSqrtHanning[] ALIGN8_END;
#else
static const ALIGN8_BEG int16_t WebRtcAecm_kSqrtHanning[] ALIGN8_END = {
  0, 399, 798, 1196, 1594, 1990, 2386, 2780, 3172,
  3562, 3951, 4337, 4720, 5101, 5478, 5853, 6224,
  6591, 6954, 7313, 7668, 8019, 8364, 8705, 9040,
  9370, 9695, 10013, 10326, 10633, 10933, 11227, 11514,
  11795, 12068, 12335, 12594, 12845, 13089, 13325, 13553,
  13773, 13985, 14189, 14384, 14571, 14749, 14918, 15079,
  15231, 15373, 15506, 15631, 15746, 15851, 15947, 16034,
  16111, 16179, 16237, 16286, 16325, 16354, 16373, 16384
};
#endif

#ifdef AECM_WITH_ABS_APPROX
//Q15 alpha = 0.99439986968132  const Factor for magnitude approximation
static const uint16_t kAlpha1 = 32584;
//Q15 beta = 0.12967166976970   const Factor for magnitude approximation
static const uint16_t kBeta1 = 4249;
//Q15 alpha = 0.94234827210087  const Factor for magnitude approximation
static const uint16_t kAlpha2 = 30879;
//Q15 beta = 0.33787806009150   const Factor for magnitude approximation
static const uint16_t kBeta2 = 11072;
//Q15 alpha = 0.82247698684306  const Factor for magnitude approximation
static const uint16_t kAlpha3 = 26951;
//Q15 beta = 0.57762063060713   const Factor for magnitude approximation
static const uint16_t kBeta3 = 18927;
#endif

static const int16_t kNoiseEstQDomain = 15;
static const int16_t kNoiseEstIncCount = 5;


static void ComfortNoise(AecmCore* aecm,
                         const uint16_t* dfa,
                         ComplexInt16* out,
                         const int16_t* lambda);

static void WindowAndFFT(AecmCore* aecm,
                         int16_t* fft,
                         const int16_t* time_signal,
                         ComplexInt16* freq_signal,
                         int time_signal_scaling) {
  int i = 0;

  // FFT of signal
  for (i = 0; i < PART_LEN; i++) {
    // Window time domain signal and insert into real part of
    // transformation array |fft|
    int16_t scaled_time_signal = time_signal[i] << time_signal_scaling;
    fft[i] = (int16_t)((scaled_time_signal * WebRtcAecm_kSqrtHanning[i]) >> 14);
    scaled_time_signal = time_signal[i + PART_LEN] << time_signal_scaling;
    fft[PART_LEN + i] = (int16_t)((
        scaled_time_signal * WebRtcAecm_kSqrtHanning[PART_LEN - i]) >> 14);
  }

  // Do forward FFT, then take only the first PART_LEN complex samples,
  // and change signs of the imaginary parts.
  WebRtcSpl_RealForwardFFT(aecm->real_fft, fft, (int16_t*)freq_signal);
  for (i = 0; i < PART_LEN; i++) {
    freq_signal[i].imag = -freq_signal[i].imag;
  }
}

static void InverseFFTAndWindow(AecmCore* aecm,
                                int16_t* fft,
                                ComplexInt16* efw,
                                int16_t* output,
                                const int16_t* nearendClean) {
  int i, j, outCFFT;
  int32_t tmp32no1;
  // Reuse |efw| for the inverse FFT output after transferring
  // the contents to |fft|.
  int16_t* ifft_out = (int16_t*)efw;

  // Synthesis
 for (i = 1, j = 2; i < PART_LEN; i += 1, j += 2) {
    fft[j] = efw[i].real;
    fft[j + 1] = -efw[i].imag;
  }
  fft[0] = efw[0].real;
  fft[1] = -efw[0].imag;

  fft[PART_LEN2] = efw[PART_LEN].real;
  fft[PART_LEN2 + 1] = -efw[PART_LEN].imag;

  // Inverse FFT. Keep outCFFT to scale the samples in the next block.
  outCFFT = WebRtcSpl_RealInverseFFT(aecm->real_fft, fft, ifft_out);
  for (i = 0; i < PART_LEN; i++) {
    ifft_out[i] = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
                    ifft_out[i], WebRtcAecm_kSqrtHanning[i], 14);
    tmp32no1 = WEBRTC_SPL_SHIFT_W32((int32_t)ifft_out[i],
                                     outCFFT - aecm->dfaCleanQDomain);
    output[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,
                                        tmp32no1 + aecm->outBuf[i],
                                        WEBRTC_SPL_WORD16_MIN);

    tmp32no1 = (ifft_out[PART_LEN + i] *
        WebRtcAecm_kSqrtHanning[PART_LEN - i]) >> 14;
    tmp32no1 = WEBRTC_SPL_SHIFT_W32(tmp32no1,
                                    outCFFT - aecm->dfaCleanQDomain);
    aecm->outBuf[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,
                                                tmp32no1,
                                                WEBRTC_SPL_WORD16_MIN);
  }

  // Copy the current block to the old position
  // (aecm->outBuf is shifted elsewhere)
  memcpy(aecm->xBuf, aecm->xBuf + PART_LEN, sizeof(int16_t) * PART_LEN);
  memcpy(aecm->dBufNoisy,
         aecm->dBufNoisy + PART_LEN,
         sizeof(int16_t) * PART_LEN);
  if (nearendClean != NULL)
  {
    memcpy(aecm->dBufClean,
           aecm->dBufClean + PART_LEN,
           sizeof(int16_t) * PART_LEN);
  }
}

// Transforms a time domain signal into the frequency domain, outputting the
// complex valued signal, absolute value and sum of absolute values.
//
// time_signal          [in]    Pointer to time domain signal
// freq_signal_real     [out]   Pointer to real part of frequency domain array
// freq_signal_imag     [out]   Pointer to imaginary part of frequency domain
//                              array
// freq_signal_abs      [out]   Pointer to absolute value of frequency domain
//                              array
// freq_signal_sum_abs  [out]   Pointer to the sum of all absolute values in
//                              the frequency domain array
// return value                 The Q-domain of current frequency values
//
static int TimeToFrequencyDomain(AecmCore* aecm,
                                 const int16_t* time_signal,
                                 ComplexInt16* freq_signal,
                                 uint16_t* freq_signal_abs,
                                 uint32_t* freq_signal_sum_abs) {
  int i = 0;
  int time_signal_scaling = 0;

  int32_t tmp32no1 = 0;
  int32_t tmp32no2 = 0;

  // In fft_buf, +16 for 32-byte alignment.
  int16_t fft_buf[PART_LEN4 + 16];
  int16_t *fft = (int16_t *) (((uintptr_t) fft_buf + 31) & ~31);

  int16_t tmp16no1;
#ifndef WEBRTC_ARCH_ARM_V7
  int16_t tmp16no2;
#endif
#ifdef AECM_WITH_ABS_APPROX
  int16_t max_value = 0;
  int16_t min_value = 0;
  uint16_t alpha = 0;
  uint16_t beta = 0;
#endif

#ifdef AECM_DYNAMIC_Q
  tmp16no1 = WebRtcSpl_MaxAbsValueW16(time_signal, PART_LEN2);
  time_signal_scaling = WebRtcSpl_NormW16(tmp16no1);
#endif

  WindowAndFFT(aecm, fft, time_signal, freq_signal, time_signal_scaling);

  // Extract imaginary and real part, calculate the magnitude for
  // all frequency bins
  freq_signal[0].imag = 0;
  freq_signal[PART_LEN].imag = 0;
  freq_signal_abs[0] = (uint16_t)WEBRTC_SPL_ABS_W16(freq_signal[0].real);
  freq_signal_abs[PART_LEN] = (uint16_t)WEBRTC_SPL_ABS_W16(
                                freq_signal[PART_LEN].real);
  (*freq_signal_sum_abs) = (uint32_t)(freq_signal_abs[0]) +
                           (uint32_t)(freq_signal_abs[PART_LEN]);

  for (i = 1; i < PART_LEN; i++)
  {
    if (freq_signal[i].real == 0)
    {
      freq_signal_abs[i] = (uint16_t)WEBRTC_SPL_ABS_W16(freq_signal[i].imag);
    }
    else if (freq_signal[i].imag == 0)
    {
      freq_signal_abs[i] = (uint16_t)WEBRTC_SPL_ABS_W16(freq_signal[i].real);
    }
    else
    {
      // Approximation for magnitude of complex fft output
      // magn = sqrt(real^2 + imag^2)
      // magn ~= alpha * max(|imag|,|real|) + beta * min(|imag|,|real|)
      //
      // The parameters alpha and beta are stored in Q15

#ifdef AECM_WITH_ABS_APPROX
      tmp16no1 = WEBRTC_SPL_ABS_W16(freq_signal[i].real);
      tmp16no2 = WEBRTC_SPL_ABS_W16(freq_signal[i].imag);

      if(tmp16no1 > tmp16no2)
      {
        max_value = tmp16no1;
        min_value = tmp16no2;
      } else
      {
        max_value = tmp16no2;
        min_value = tmp16no1;
      }

      // Magnitude in Q(-6)
      if ((max_value >> 2) > min_value)
      {
        alpha = kAlpha1;
        beta = kBeta1;
      } else if ((max_value >> 1) > min_value)
      {
        alpha = kAlpha2;
        beta = kBeta2;
      } else
      {
        alpha = kAlpha3;
        beta = kBeta3;
      }
      tmp16no1 = (int16_t)((max_value * alpha) >> 15);
      tmp16no2 = (int16_t)((min_value * beta) >> 15);
      freq_signal_abs[i] = (uint16_t)tmp16no1 + (uint16_t)tmp16no2;
#else
#ifdef WEBRTC_ARCH_ARM_V7
      __asm __volatile(
        "smulbb %[tmp32no1], %[real], %[real]\n\t"
        "smlabb %[tmp32no2], %[imag], %[imag], %[tmp32no1]\n\t"
        :[tmp32no1]"+&r"(tmp32no1),
         [tmp32no2]"=r"(tmp32no2)
        :[real]"r"(freq_signal[i].real),
         [imag]"r"(freq_signal[i].imag)
      );
#else
      tmp16no1 = WEBRTC_SPL_ABS_W16(freq_signal[i].real);
      tmp16no2 = WEBRTC_SPL_ABS_W16(freq_signal[i].imag);
      tmp32no1 = tmp16no1 * tmp16no1;
      tmp32no2 = tmp16no2 * tmp16no2;
      tmp32no2 = WebRtcSpl_AddSatW32(tmp32no1, tmp32no2);
#endif // WEBRTC_ARCH_ARM_V7
      tmp32no1 = WebRtcSpl_SqrtFloor(tmp32no2);

      freq_signal_abs[i] = (uint16_t)tmp32no1;
#endif // AECM_WITH_ABS_APPROX
    }
    (*freq_signal_sum_abs) += (uint32_t)freq_signal_abs[i];
  }

  return time_signal_scaling;
}

  void WebRtcAecm_echoFarEnRatio(AecmCore* aecm,uint16_t n)//estimate the energy ratio between echo and far signal
  {  
	 if((aecm->currentVADValue==1)||(n==1))
	{
	  uint16_t i;
	  uint16_t maxTmp=aecm->nearFarBitCountCorrHis[0];
	  uint16_t minTmp=maxTmp;
	  uint16_t checkLen=MAX_BUF_LEN;
	  uint16_t RatioMaxTmp=aecm->nearFarEnRatioHis[0];
	  uint16_t RatioMinTmp=maxTmp;
	  float enSumTmpAll=0.0f;
	  float enFarAvg=0.0f;
	  float minFarEnSum=20;
	  float ratioEffectiveSum;
	  float enEffectiveSum;
	  float ratioTmp;
	  float ratioCandidate;
	  int16_t effectiveFrameNum=0;
	  //int16_t effectiveFlag=0;
	 // int16_t lastEchoFarRatio=aecm->echoFarEnRatio;
	  float ratioAllSum=0.0f;
	  float ratioAllAvg=0.0f;
	  enSumTmpAll +=aecm->farEnergyHis[0];	
	  ratioAllSum += aecm->farEnergyHis[0]*aecm->nearFarEnRatioHis[0];
	  if(aecm->echoFarEnRatioNeed==1)//aecm->echoFarEnRatioNeed==1 means the history filter is updated,we only check MIN_MSE_COUNT blocks
	  {
		  checkLen = MIN_MSE_COUNT;
	  }
	  for(i=1;i<checkLen;i++)
	  {
		  enSumTmpAll +=aecm->farEnergyHis[i];	// sum of weight coef
		  ratioAllSum += aecm->farEnergyHis[i]*aecm->nearFarEnRatioHis[i];// use aecm->farEnergyHis[i] as the weighte coef, we assume that aecm->nearFarEnRatio is more reliable when the far energy is bigger
		  //calculate the max value and min value of nearFarBitCountCorr;  calculate the max value and min value of nearFarEnRatio;
		  if(maxTmp<aecm->nearFarBitCountCorrHis[i])
		  {
			  maxTmp=aecm->nearFarBitCountCorrHis[i];
		  }
		  else if(minTmp>aecm->nearFarBitCountCorrHis[i])
		  {
			  minTmp=aecm->nearFarBitCountCorrHis[i];
		  }
		  if(RatioMaxTmp<aecm->nearFarEnRatioHis[i])
		  {
			  RatioMaxTmp=aecm->nearFarEnRatioHis[i];
		  }
		  else if(RatioMinTmp>aecm->nearFarEnRatioHis[i])
		  {
			  RatioMinTmp=aecm->nearFarEnRatioHis[i];
		  }
	  }	  
	  enFarAvg = enSumTmpAll/checkLen;// the average energy of far signal
	  ratioAllAvg = ratioAllSum/enSumTmpAll;//the weighted average of ratio
	  enEffectiveSum =0.0f;
	  ratioEffectiveSum =0.0f;
	  effectiveFrameNum =0;
	  for(i=0;i<checkLen;i++)
	  {
		  if((enFarAvg>512)&&(enFarAvg<aecm->farEnergyHis[i])&&(aecm->nearFarEnRatioHis[i]<2048)&&(aecm->nearFarEnRatioHis[i]<1024+ratioAllAvg))//远端能量小于平均值的帧，不参与计算
		  {
			 ratioTmp =aecm->nearFarEnRatioHis[i];
		     ratioEffectiveSum +=ratioTmp*aecm->farEnergyHis[i];
			 enEffectiveSum +=aecm->farEnergyHis[i];
			 effectiveFrameNum++;
		  }
	  }
	  if((effectiveFrameNum>0)&&(enEffectiveSum>0))
	  {
		ratioCandidate = ratioEffectiveSum/enEffectiveSum;	// the candidate value of energy between echo signal and far signal	
	  }
	  else
	  {
		  ratioCandidate = aecm->echoFarEnRatio;		  
	  }
	  if(ratioCandidate>4096)//if the candidate is bigger than 4 in Q10,we force it equal to the last echoFarEnRatio
	  {
		  ratioCandidate = aecm->echoFarEnRatio;		  
	  }
	  else if((ratioCandidate>2048)&&(aecm->echoFarEnRatio*3<ratioCandidate))//if the candidate is much bigger than the last echoFarEnRatio,we force it equal to the last echoFarEnRatio
	  {
		  ratioCandidate = aecm->echoFarEnRatio;	//ratioCandidate is invalid ,use the last echoFarEnRatio 
	  }
	 
	 if(aecm->echoFarEnRatioNeed==1) //check if the history filter has beed updated. if ture, the ratioCandidate is more reliable
	  {	
		  if((minTmp>ONE_Q14-4096)&&(ratioCandidate<2048))// the min value of nearFarBitCountCorr is big and ratioCandidate reasonable low
		  {
			  if(aecm->speakerGain>1)//check if the speaker is just turned on 
			  {
				  aecm->echoFarEnRatio= ratioCandidate;//update echoFarEnRatio fastly,this will help us to eliminate the echo fast.
			  }
			  else
			  {
				  float smoothFactor = 0.5f*minTmp/ONE_Q14;
				  aecm->echoFarEnRatio = aecm->echoFarEnRatio*(1.0f-smoothFactor)+ratioCandidate*smoothFactor;//update echoFarEnRatio slowly
			  }

		  }
		  else
		   {
			   float smoothFactor = 0.5f*minTmp/ONE_Q14;
			   if(ratioCandidate<256)
			   {
				   aecm->echoFarEnRatio= ratioCandidate;//update echoFarEnRatio fastly since the ratioCandidate is very low
			   }
			   else 
			  {
				  if(smoothFactor<0.2f)
				  {
					smoothFactor=0.2f;
				  }
				  else if (smoothFactor >= 0.9f)
				  {
					  smoothFactor = 0.9f;
				  }

				  if(aecm->speakerGain>1)
				  {
					aecm->echoFarEnRatio= ratioCandidate;
				  }
				  else
				 {
					smoothFactor *= 0.25f;
					aecm->echoFarEnRatio = aecm->echoFarEnRatio*(1.0f-smoothFactor)+ratioCandidate*smoothFactor;
				  }
			   }
		   }
	  }	 
	 //condition:(1) ratio is stable         (2)far end is not too small   (3)current ratio candidate is less than last echoFarEnRatio
	 else if((RatioMaxTmp<RatioMinTmp+1024)&&(enSumTmpAll>minFarEnSum)&&(ratioCandidate<aecm->echoFarEnRatio))
	  {		 
		 // float  factor = 1.0f*(ONE_Q14-minTmp)/ONE_Q14;
		  float factor =0.95f;//fast decrease
		  aecm->echoFarEnRatio= aecm->echoFarEnRatio*factor + ratioCandidate*(1.0f-factor);
		  
		  if(aecm->echoFarEnRatio>2048)
		   {
			   aecm->echoFarEnRatio =2048;//Q10
		   }		 
	  }
//condition:(1)the min value of  nearFarbitcountCorr is big enough           (2)far end is not too small      (3)current ratio candidate is less than last echoFarEnRatio
	  else if((minTmp>ONE_Q14-4096)&&(enSumTmpAll>minFarEnSum)&&(ratioCandidate<aecm->echoFarEnRatio))
	  {		  
		  float factor =0.95f;//fast decrease
		  aecm->echoFarEnRatio = aecm->echoFarEnRatio*factor + ratioCandidate*(1.0f - factor);
		  if(aecm->echoFarEnRatio>2048)
		   {
			  aecm->echoFarEnRatio =2048;//Q10
		   }
	  }	
  }
  }   


  static int CmpUint16_t(const void *a, const void *b)
{
    const uint16_t *da = (const uint16_t *)a;
    const uint16_t *db = (const uint16_t *)b;

    return (*da < *db) - (*da > *db);
}
#ifdef MUSIC_OR_SPEECH
static void getTimedomainSignalAfterLms(AecmCore* aecm,	int16_t* fft,ComplexInt16* nearComplexSpec,	int16_t* output,uint16_t* nearSpecAbs,	const int32_t* echoEst,	int16_t far_q) 
{
		int i, j, outCFFT;
		int32_t tmp32no1;
		ComplexInt16 efw[PART_LEN1];
		// Reuse |efw| for the inverse FFT output after transferring
		// the contents to |fft|.
		int16_t* ifft_out = (int16_t*)efw;

		for (i = 0; i < PART_LEN1; i++) 
		{
			float hnlTmp = 1.0f*echoEst[i];
			hnlTmp =hnlTmp/(nearSpecAbs[i]+1);
			hnlTmp = 1.0f-(hnlTmp/(1<<(far_q+12-aecm->dfaCleanQDomain)));
			if(hnlTmp<0.05f)
			{
			   hnlTmp=0.05f;
			}
			else if(hnlTmp>1.0f)
			{
				hnlTmp=1.0f;
			}
					
			 efw[i].real = nearComplexSpec[i].real*hnlTmp;
			 efw[i].imag = nearComplexSpec[i].imag*hnlTmp;
		}

		// Synthesis
		for (i = 1, j = 2; i < PART_LEN; i += 1, j += 2) {
			fft[j] = efw[i].real;
			fft[j + 1] = -efw[i].imag;
		}
		fft[0] = efw[0].real;
		fft[1] = -efw[0].imag;

		fft[PART_LEN2] = efw[PART_LEN].real;
		fft[PART_LEN2 + 1] = -efw[PART_LEN].imag;

		// Inverse FFT. Keep outCFFT to scale the samples in the next block.
		outCFFT = WebRtcSpl_RealInverseFFT(aecm->real_fft, fft, ifft_out);
		for (i = 0; i < PART_LEN; i++) {
			ifft_out[i] = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(
				ifft_out[i], WebRtcAecm_kSqrtHanning[i], 14);
			tmp32no1 = WEBRTC_SPL_SHIFT_W32((int32_t)ifft_out[i],
				outCFFT - aecm->dfaCleanQDomain);
			output[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,
				tmp32no1 + aecm->residualBuf[i],
				WEBRTC_SPL_WORD16_MIN);

			tmp32no1 = (ifft_out[PART_LEN + i] *
				WebRtcAecm_kSqrtHanning[PART_LEN - i]) >> 14;
			tmp32no1 = WEBRTC_SPL_SHIFT_W32(tmp32no1,
				outCFFT - aecm->dfaCleanQDomain);
			aecm->residualBuf[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,
				tmp32no1,
				WEBRTC_SPL_WORD16_MIN);
		}

		
}

void WebRtcAecm_updateSpeechMusicMode(AecmCore* aecm,const int16_t* farend,	const int16_t* nearend,int16_t sampleNum)
{
	int16_t lenToReadIn;
	float alpha=0.9f;
	int16_t len10ms=aecm->mult*80;
	if(aecm->smdBufLeft+sampleNum>SMD_BUF_MAX_SIZE)
	{
		lenToReadIn =SMD_BUF_MAX_SIZE-aecm->smdBufLeft;		
	}
	else
	{
		lenToReadIn =sampleNum;
	}
	memcpy(aecm->nearSmdAudioBuf+aecm->smdBufLeft,nearend,lenToReadIn*sizeof(int16_t));
	memcpy(aecm->farSmdAudioBuf+aecm->smdBufLeft,farend,lenToReadIn*sizeof(int16_t));
	aecm->smdBufLeft += lenToReadIn;
	while (aecm->smdBufLeft>=len10ms)
	{
		float farMusicProb = smd_handle_recv(aecm->farSmdInst,aecm->farSmdAudioBuf, len10ms,len10ms*100);
		float nearMusicProb = smd_handle_recv(aecm->nearSmdInst,aecm->nearSmdAudioBuf, len10ms,len10ms*100);		
		aecm->smdBufLeft-=len10ms;
		memmove(aecm->nearSmdAudioBuf,aecm->nearSmdAudioBuf+len10ms,aecm->smdBufLeft*sizeof(int16_t));
		memmove(aecm->farSmdAudioBuf,aecm->farSmdAudioBuf+len10ms,aecm->smdBufLeft*sizeof(int16_t));

		aecm->farMusicProb[0] = aecm->farMusicProb[0]*alpha+farMusicProb*(1.0f-alpha);
		aecm->nearMusicProb[0] = aecm->nearMusicProb[0]*alpha+nearMusicProb*(1.0f-alpha);
		//aecm->nearBandTonality=smd_get_band_tonality(aecm->nearSmdInst);
		//aecm->farBandTonality=smd_get_band_tonality(aecm->farSmdInst);
	}
	aecm->nearBandTonality=smd_get_band_tonality(aecm->nearSmdInst);
	aecm->farBandTonality=smd_get_band_tonality(aecm->farSmdInst);
	if(aecm->nearMusicProb[0]>0.5f)
	{
		aecm->nearIsSpeech=0;
	}
	else
	{
		aecm->nearIsSpeech=1;
	}
	//aecm->nearIsSpeech=1;
}
#endif
#ifdef AFC
int16_t afcInit(AudioFeedbackCancellation*afcHandle)
{
	int16_t i;
	if(!afcHandle)
	{
		return -1;
	}
	afcHandle->veryLongTermPos=VERY_LONG_TERM_BLOCK_NUM-1;
	afcHandle->howlingPos=-1;
	afcHandle->candidatePos=-1;	
	afcHandle->howlingPossibility=0.0f; 
	afcHandle->afcInOutEnRatioSmoothed=1.0f;	
	memset(afcHandle->veryLongTermSpecHis,0,sizeof(uint16_t)*VERY_LONG_TERM_BLOCK_NUM*PART_LEN1);
	memset(afcHandle->veryLongTermPeakPosHis,0,sizeof(uint16_t)*VERY_LONG_TERM_BLOCK_NUM*PEAK_NUM);
	for (i = 0; i < PART_LEN1 ; i++)
	{
		afcHandle->howlingScaleFactorSmoothed[i]=1.0f;
	}
	return 0;
}
/*
para explain：
afcHandle ： afc handle
nearSpecAbs:abs of near end spectrum
sampleRate:only 8000 or 16000
frameCnt：
efw：complex spectrum of near end
*/
int16_t howlingCancellation(AudioFeedbackCancellation*afcHandle,uint16_t* nearSpecAbs,int32_t sampleRate,uint32_t frameCnt,ComplexInt16 *efw)
{
	int16_t freqMult=sampleRate/8000;
	uint16_t peakPos[PEAK_NUM];
	int16_t i,j;
	int16_t shortTermFrameNum=1;
	uint16_t nearSpecSorted[PART_LEN1];//sorted spectrum ,help to find spectrum peaks
	float longTermPowerAvg[PART_LEN1];//long term avgerage power in one band
	float longTermPowerNum[PART_LEN1];
	float longTermPowerAvgInLowBandGroup=1.0f;//the avg of longTermPowerAvg[i] while i<lowBandGroupWidth
	float longTermPowerMaxInLowBandGroup=0.0f;//the max of longTermPowerAvg[i] while i<lowBandGroupWidth
	float longTermPowerMinInLowBandGroup=1e10f;//the min of longTermPowerAvg[i] while i<lowBandGroupWidth
	int16_t lowBandGroupWidth = 500/(4000*freqMult/PART_LEN1);	//
	float howlingScaleFactor[PART_LEN1];	//instant suppression factor 
	int16_t peakPosMin = PART_LEN1;//the min value of peak position
	int16_t peakPosMax = 0;//the max value of peak position
	int16_t minHowlingPos = 500*PART_LEN1/(4000*freqMult);//the min value of howling position
	uint16_t validPeakCnt=0;//the number of valid peaks
	float validPeakHistogram[PART_LEN1];//histogram of valid peaks
	int16_t firstHowlingChoice=1;//this flag tell us whether the howling position is the firsh choice 		
	int16_t ret;
	float candidateLtPower=0.0f;//long term avgerage power in candidate band
	float howlingLtPower=0.0f;//long term avgerage power in howling band
	float candidateStPower=0.0f;//short term avgerage power in candidate band
	float howlingStPower=0.0f;//long term avgerage power in howling band
	
	float afcInEn=1.0f;//input power sum
    float afcOutEn=1.0f;////output power sum
	float peakThreshold=0.0f;//threshold for valid peaks
	float avgPower=1.0f;
	int16_t avgPowerCnt=1;
	afcHandle->longTermFrameNum= 2000*freqMult/8;	//it tells us how much blocks it contains for long term
	
	for(i=0;i<PART_LEN1;i++)
	{
		howlingScaleFactor[i]=1.0f;	 //intialize howlingScaleFactor	
	}
	if(frameCnt<10)
	{
		for(i=0;i<PART_LEN1;i++)
		{
			afcHandle->howlingScaleFactorSmoothed[i]=1.0f;		 //intialize howlingScaleFactorSmoothed	
		}
	}
	
	//step1 :sort near spectrum
	memcpy(nearSpecSorted,&nearSpecAbs[0],PART_LEN1*sizeof(uint16_t));
	qsort(nearSpecSorted,  PART_LEN1, sizeof(uint16_t), CmpUint16_t);

	
	
    //step 2: find peak from near end spectrum
	for(j=0;j<PEAK_NUM;j++)
	{
		for(i=0;i<PART_LEN1;i++)
		{
			if(nearSpecAbs[i]==nearSpecSorted[j])
			{
				peakPos[j]=i;
				break;
			}
		}
		if(peakPos[j]<peakPosMin)
		{
			peakPosMin = peakPos[j];
		}
		if(peakPos[j]>peakPosMax)
		{
			peakPosMax = peakPos[j];
		}
	}
    //step 3: calculate peak distribution
	if(1)
	{			
		uint16_t peakCnt=0;			
		afcHandle->veryLongTermPos++;
		if(afcHandle->veryLongTermPos>=VERY_LONG_TERM_BLOCK_NUM)
		{
			afcHandle->veryLongTermPos=0;
		}		
		memmove(&afcHandle->veryLongTermSpecHis[1][0],&afcHandle->veryLongTermSpecHis[0][0],PART_LEN1*sizeof(uint16_t)*(VERY_LONG_TERM_BLOCK_NUM-1));		
		memmove(&afcHandle->veryLongTermPeakPosHis[1][0],&afcHandle->veryLongTermPeakPosHis[0][0],PEAK_NUM*sizeof(uint16_t)*(VERY_LONG_TERM_BLOCK_NUM-1));
		afcHandle->veryLongTermPos=0;
		memcpy(&afcHandle->veryLongTermSpecHis[afcHandle->veryLongTermPos][0],&nearSpecAbs[0],PART_LEN1*sizeof(uint16_t));		
		memcpy(&afcHandle->veryLongTermPeakPosHis[afcHandle->veryLongTermPos][0],&peakPos[0],PEAK_NUM*sizeof(uint16_t));
		if(frameCnt<=VERY_LONG_TERM_BLOCK_NUM)
		{
			//return 0;
		}
		for(j=0;j<VERY_LONG_TERM_BLOCK_NUM;j++)
		{
			for(i=0;i<PEAK_NUM;i++)
			{
				peakThreshold += afcHandle->veryLongTermSpecHis[j][afcHandle->veryLongTermPeakPosHis[j][i]];
				peakCnt++;
			}
		}
		peakThreshold /=peakCnt;
		//peakThreshold /=2;
		for(i=0;i<PART_LEN1;i++)
		{
			validPeakHistogram[i]=0.0f;
		}
		for(j=0;j<VERY_LONG_TERM_BLOCK_NUM;j++)
		{
			for(i=0;i<PEAK_NUM;i++)
			{
				if(afcHandle->veryLongTermSpecHis[j][afcHandle->veryLongTermPeakPosHis[j][i]]>peakThreshold && afcHandle->veryLongTermPeakPosHis[j][i]<PART_LEN1)
				{
					validPeakHistogram[afcHandle->veryLongTermPeakPosHis[j][i]]+=1.0f;					
					validPeakCnt++;
				}
			}
		}		
		for(i=0;i<PART_LEN1;i++)
		{
			validPeakHistogram[i]/=validPeakCnt;						
		}
	}
	
	//step 4: update candidate howling poing
	if(validPeakCnt>VERY_LONG_TERM_BLOCK_NUM*PEAK_NUM*0.1f)
	{
		if(afcHandle->candidatePos<0 && peakPos[0]>minHowlingPos)
		{
			afcHandle->candidatePos = peakPos[0];
		}
		else if(nearSpecAbs[afcHandle->candidatePos]<nearSpecAbs[peakPos[0]])
		{
			int16_t candidateIsInPeakGroup=0;
			for(j=0;j<PEAK_NUM;j++)
			{
				if(nearSpecAbs[afcHandle->candidatePos]>=nearSpecAbs[peakPos[j]])
				{
					candidateIsInPeakGroup=1;
					break;
				}
			}
			if(candidateIsInPeakGroup==0 && peakPos[0]>minHowlingPos)
			{
				afcHandle->candidatePos = peakPos[0];
			}
			else if(nearSpecAbs[afcHandle->candidatePos]<nearSpecAbs[peakPos[0]]*0.25f && peakPos[0]>minHowlingPos)
			{
				afcHandle->candidatePos = peakPos[0];
			}			
		}
		
	}
	// discard some invalid candidate howling point
	if(validPeakHistogram[afcHandle->candidatePos]*i<0.1f*minHowlingPos)
	{
		afcHandle->candidatePos = -1;
	}	
	if(afcHandle->candidatePos<0)
	{			
		afcHandle->howlingPos =-1;
		afcHandle->candidatePos =-1;
	}	
	
//step 5: update howling position	
	if(afcHandle->candidatePos>=0)
	{		
		for(i=0;i<PART_LEN1;i++)
		{
			longTermPowerAvg[i] = 0.0f;
			longTermPowerNum[i] =1;
		}		
		if(afcHandle->howlingPos>0)
		{			
			for(j=afcHandle->longTermFrameNum-1;j>=0;j--)
			{				
				float candidateBinPower,howlingBinPower;
				for(i=0;i<PART_LEN1;i++)
				{					
					if(afcHandle->veryLongTermSpecHis[j][afcHandle->veryLongTermPeakPosHis[j][0]]>peakThreshold)//if(afcHandle->veryLongTermSpecHis[j][i]*4>peakThreshold)
					{
						avgPower += afcHandle->veryLongTermSpecHis[j][i];
						avgPowerCnt++;
						longTermPowerAvg[i] += afcHandle->veryLongTermSpecHis[j][i];
						longTermPowerNum[i]++;
					}
				}			
				
				candidateBinPower= 1.0f*afcHandle->veryLongTermSpecHis[j][afcHandle->candidatePos];
				if(j<shortTermFrameNum)
				{
					candidateStPower +=candidateBinPower;					
				}				
				candidateLtPower += candidateBinPower;
				howlingBinPower=afcHandle->veryLongTermSpecHis[j][afcHandle->howlingPos];
				if(j<shortTermFrameNum)
				{
					howlingStPower +=howlingBinPower;
				}
				howlingLtPower += howlingBinPower;				
			}
			avgPower /=avgPowerCnt;
			candidateStPower /=shortTermFrameNum;
			candidateLtPower /=afcHandle->longTermFrameNum;
			howlingStPower /=shortTermFrameNum;
			howlingLtPower /=afcHandle->longTermFrameNum;
			for(i=0;i<PART_LEN1;i++)
			{
				longTermPowerAvg[i] /=longTermPowerNum[i];
				if(i<lowBandGroupWidth)
				{
					longTermPowerAvgInLowBandGroup += longTermPowerAvg[i];
				}
			}
			longTermPowerAvgInLowBandGroup/=lowBandGroupWidth;
			if(candidateLtPower>howlingLtPower && candidateStPower>howlingStPower)
			{
				afcHandle->howlingPos = afcHandle->candidatePos;
				howlingLtPower = candidateLtPower;
				howlingStPower = candidateStPower;
			}			
		}
		else 
		{
			
			for(j=afcHandle->longTermFrameNum-1;j>=0;j--)
			{				
				float candidateBinPower;
				for(i=0;i<PART_LEN1;i++)
				{					
					if(afcHandle->veryLongTermSpecHis[j][afcHandle->veryLongTermPeakPosHis[j][0]]>peakThreshold)//if(afcHandle->veryLongTermSpecHis[j][i]*4>peakThreshold)
					{
						avgPower += afcHandle->veryLongTermSpecHis[j][i];
						avgPowerCnt++;
						longTermPowerAvg[i] += afcHandle->veryLongTermSpecHis[j][i];
						longTermPowerNum[i]++;
					}
				}				
				candidateBinPower = 1.0f*afcHandle->veryLongTermSpecHis[j][afcHandle->candidatePos];
				if(j<shortTermFrameNum)
				{
					candidateStPower +=candidateBinPower;
				}	
				candidateLtPower += candidateBinPower;								
				
			}
			avgPower /=avgPowerCnt;
			candidateStPower /=shortTermFrameNum;
			candidateLtPower /=afcHandle->longTermFrameNum;
			for(i=0;i<PART_LEN1;i++)
			{
				longTermPowerAvg[i] /=longTermPowerNum[i];
				if(i<lowBandGroupWidth)
				{
					longTermPowerAvgInLowBandGroup += longTermPowerAvg[i];
				}
			}
			longTermPowerAvgInLowBandGroup/=lowBandGroupWidth;
			afcHandle->howlingPos = afcHandle->candidatePos;
			howlingLtPower = candidateLtPower;
			howlingStPower = candidateStPower;
		}
	}
	else
	{
		afcHandle->howlingPos =-1;
		afcHandle->candidatePos =-1;
	}
	longTermPowerAvgInLowBandGroup =0.0f;
	for(i=0;i<lowBandGroupWidth;i++)
	{
		longTermPowerAvgInLowBandGroup+=longTermPowerAvg[i];
		if(longTermPowerMaxInLowBandGroup<longTermPowerAvg[i])
		{
			longTermPowerMaxInLowBandGroup=longTermPowerAvg[i];
		}
		if(longTermPowerMinInLowBandGroup>longTermPowerAvg[i])
		{
			longTermPowerMinInLowBandGroup=longTermPowerAvg[i];
		}
	}
	longTermPowerAvg[i]/=lowBandGroupWidth;

	//discard invalid candidate  and howling position
	if(afcHandle->candidatePos>0)
	{
		if(longTermPowerAvg[afcHandle->candidatePos]<longTermPowerAvgInLowBandGroup*6|| candidateStPower<2*longTermPowerAvgInLowBandGroup || longTermPowerAvg[afcHandle->candidatePos]< longTermPowerAvgInLowBandGroup )
		{
			afcHandle->candidatePos=-1;
		}
	}
	if(afcHandle->howlingPos>0)
	{
		if(longTermPowerAvg[afcHandle->howlingPos]<longTermPowerAvgInLowBandGroup*6 || howlingStPower<2*longTermPowerAvgInLowBandGroup || longTermPowerAvg[afcHandle->howlingPos]< longTermPowerAvgInLowBandGroup)
		{
			afcHandle->howlingPos=-1;
		}
		else if(afcHandle->howlingPos<2*minHowlingPos)
		{
			int16_t j;
			int16_t prePeakSearchBeginPos=200/(8/freqMult);//200ms
			int16_t prePeakSearchStopPos=2000/(8/freqMult);//2s
			int16_t prePeakCnt=0;
			for(j=200/(8/freqMult);j<prePeakSearchStopPos;j++)
			{
				if(afcHandle->veryLongTermSpecHis[j][afcHandle->howlingPos]>longTermPowerMaxInLowBandGroup && afcHandle->veryLongTermSpecHis[j][afcHandle->howlingPos]*2>afcHandle->veryLongTermSpecHis[0][afcHandle->howlingPos])
				{
					prePeakCnt++;
				}
			}
			if(prePeakCnt*10<prePeakSearchBeginPos)
			{
				afcHandle->howlingPos=-1;
			}
			else
			{
				afcHandle->howlingPos=afcHandle->howlingPos;
			}
		}
	}
	//step 6:calculate howling prosibility	
	if(1)
	{
		afcHandle->howlingPossibility -=0.01f;
		if(peakPosMax>peakPosMin+PEAK_NUM*1.5f)
		{
			if(peakPosMin>minHowlingPos && afcHandle->veryLongTermSpecHis[0][peakPos[0]>peakThreshold])//pure hwoling signal			
			{
				afcHandle->howlingPossibility+=0.02f;				
			}			
		}		
		if(afcHandle->howlingPossibility>1.0f)
		{
			afcHandle->howlingPossibility=1.0f;
		}
		else if(afcHandle->howlingPossibility<0.0f)
		{
			afcHandle->howlingPossibility=0.0f;
		}
		// repair candidate pos and howling pos
		if(afcHandle->howlingPossibility>0.8f )		
		{
			if(afcHandle->candidatePos<minHowlingPos)//high howling possibility ,but candidate is too low, so recalculate candidate
			{
				afcHandle->candidatePos=(peakPosMax +peakPosMin)/2;
				for(i=0;i<PEAK_NUM;i++)
				{
					if(peakPos[i]>minHowlingPos)
					{
						afcHandle->candidatePos = peakPos[i];//find a second choice candidate point
						break;
					}
				}
			}
			if(afcHandle->howlingPos<minHowlingPos)//high howling possibility ,but candidate is too low, so recalculate howling point
			{
				afcHandle->howlingPos = afcHandle->candidatePos;
			}
			if(afcHandle->howlingPos<minHowlingPos)
			{
				afcHandle->howlingPos=(peakPosMax +peakPosMin)/2;
				for(i=0;i<PEAK_NUM;i++)
				{
					if(peakPos[i]>minHowlingPos)
					{
						afcHandle->howlingPos = peakPos[i];//find a second choice howling point
						break;
					}
				}
			}
		}
		if(afcHandle->howlingPos<minHowlingPos)
		{
			float maxWeightedLongTermPower=0.0f;
			float weightedLongTermPowerTmp;
			int16_t abnormalBinCnt=0;
			for(i=0;i<PART_LEN1;i++)
			{
				weightedLongTermPowerTmp=longTermPowerAvg[i]*sqrtf(1.0*i);
				if(weightedLongTermPowerTmp>6*maxWeightedLongTermPower)
				{
					maxWeightedLongTermPower = weightedLongTermPowerTmp;
					afcHandle->howlingPos = i;
					if(afcHandle->howlingPos>=minHowlingPos)
					{
						firstHowlingChoice=0;
						abnormalBinCnt++;
					}
				}
			}
			if(abnormalBinCnt<PEAK_NUM)
			{
				afcHandle->howlingPos=-1;
				firstHowlingChoice=1;
			}
		}
		//finish repair candidate pos and howling pos
	}
	//discard invalid howling point
	if(afcHandle->howlingPos>0)
	{
		if(afcHandle->howlingPos<2*minHowlingPos || firstHowlingChoice==0 )
		{
			int16_t j;
			int16_t prePeakSearchBeginPos=200/(8/freqMult);//200ms
			int16_t prePeakSearchStopPos=2000/(8/freqMult);//2s
			int16_t prePeakCnt=0;
			for(j=200/(8/freqMult);j<prePeakSearchStopPos;j++)
			{
				if(afcHandle->veryLongTermSpecHis[j][afcHandle->howlingPos]>longTermPowerMaxInLowBandGroup && afcHandle->veryLongTermSpecHis[j][afcHandle->howlingPos]*2>afcHandle->veryLongTermSpecHis[0][afcHandle->howlingPos])
				{
					prePeakCnt++;
				}
			}
			if(prePeakCnt*10<prePeakSearchBeginPos)
			{
				afcHandle->howlingPos=-1;
			}
			else
			{
				afcHandle->howlingPos=afcHandle->howlingPos;
			}
		}
	}

	
	//howling point detection is finished,next we do howling suppression
	//step 7:update howling scale factor
		
	if(afcHandle->howlingPos>minHowlingPos)
	{
		float factorTmp;		
		
		for(i=0;i<PART_LEN1;i++)
		{		
			factorTmp=1.0f;
			if(firstHowlingChoice || i>=afcHandle->howlingPos)//if current howling position is not the first choice,then we only scale down frequency components which is equal or higher than howling point
			{
				factorTmp = longTermPowerAvg[i]/(longTermPowerAvg[afcHandle->howlingPos]+0.01f);
			}
			
			if(factorTmp>1.0f)
			{
				factorTmp=1.0f;
			}
			howlingScaleFactor[i]*=1.0f-factorTmp;// longTermPowerAvg[i] is bigger,the howlingScaleFactor[i] is smaller
			howlingScaleFactor[i] = powf(howlingScaleFactor[i],i/minHowlingPos);
			if(validPeakHistogram[i]*i*PART_LEN1>1.0f*minHowlingPos&& i>=afcHandle->howlingPos)
			{
				howlingScaleFactor[i] *= powf(0.9f*minHowlingPos/i,validPeakHistogram[i]*i*PART_LEN1/minHowlingPos);//  validPeakHistogram[i] is bigger,the howlingScaleFactor[i] is smaller			
			}
			if(longTermPowerAvg[i]>longTermPowerMaxInLowBandGroup)//longTermPowerAvg[i] is bigger than longTermPowerMaxInLowBandGroup,we should scale down more
			{
				howlingScaleFactor[i] *=powf(1.0f*longTermPowerMaxInLowBandGroup/longTermPowerAvg[i],i/minHowlingPos);
			}
		}
	}
	//printf("\n frameCnt=%d, canPos=%d, howlingPos=%d  ",frameCnt,afcHandle->candidatePos,afcHandle->howlingPos);
		
	for(i=0;i<PART_LEN1;i++)
	{	
		if(afcHandle->howlingPos>minHowlingPos && (i>minHowlingPos+longTermPowerAvgInLowBandGroup*longTermPowerMaxInLowBandGroup/avgPower/longTermPowerAvg[afcHandle->howlingPos] ) && i>minHowlingPos) //||  i>=afcHandle->howlingPos
		{
			afcHandle->howlingScaleFactorSmoothed[i] += (howlingScaleFactor[i]-afcHandle->howlingScaleFactorSmoothed[i])*0.25f;
		}
		else 
		{
			//try to recover afcHandle->howlingScaleFactorSmoothed[i] from low to 1.0
			float upStepSize=8.0f*PART_LEN1/(8000.0f*freqMult*(i+1));//lowest step size of recover
			int16_t j;
			if(afcHandle->veryLongTermSpecHis[0][i]>peakThreshold+1.0f)// try to acclerate the recover process
			{
				upStepSize *= afcHandle->veryLongTermSpecHis[0][i]*minHowlingPos/(peakThreshold+1.0f)/(i+1);//low frequency recover faster; high spectrum ampitude bin recover faster
				if(upStepSize>0.1f)
				{
					upStepSize=0.1f;
				}
			}			
			for(j=0;j<PEAK_NUM;j++)// try to acclerate the recover process
			{
				if((afcHandle->veryLongTermSpecHis[0][peakPos[j]]>peakThreshold+1.0f)&&(peakPos[j]<minHowlingPos))
				{
					upStepSize *= afcHandle->veryLongTermSpecHis[0][peakPos[j]]/(peakThreshold+1.0f);
					if(upStepSize>0.25f)
					{
						upStepSize=0.25f;
					}
				}		
			}
			afcHandle->howlingScaleFactorSmoothed[i] +=(1.0f-afcHandle->howlingScaleFactorSmoothed[i])*upStepSize;			
		}
		
		if(afcHandle->howlingScaleFactorSmoothed[i]<0.0f)
		{
			afcHandle->howlingScaleFactorSmoothed[i]=0.0f;
		}
		else if(afcHandle->howlingScaleFactorSmoothed[i]>1.0f)
		{
			afcHandle->howlingScaleFactorSmoothed[i]=1.0f;
		}
		//wiener filter
		if(i>minHowlingPos)		
		{
		  efw[i].imag*=afcHandle->howlingScaleFactorSmoothed[i];
		  efw[i].real*=afcHandle->howlingScaleFactorSmoothed[i];		 
		}
		if(i!=afcHandle->howlingPos)
		{
			//efw[i].imag=0;
			//efw[i].real=0;
		}
		else if(i==afcHandle->howlingPos)
		{
			float lowBandOutPowerAvg=0.0f;
			float powerInHowlingPos=afcHandle->veryLongTermSpecHis[0][i]*afcHandle->howlingScaleFactorSmoothed[i];
			int16_t j;
			for(j=0;j<minHowlingPos;j++)
			{
				lowBandOutPowerAvg +=afcHandle->veryLongTermSpecHis[0][j]*afcHandle->howlingScaleFactorSmoothed[j];
			}
			if(powerInHowlingPos>lowBandOutPowerAvg)
			{
				float rescale = lowBandOutPowerAvg/powerInHowlingPos;
				efw[i].imag*=rescale;
				efw[i].real*=rescale;
				afcHandle->howlingScaleFactorSmoothed[i]*=rescale;
			}
			//printf("howlingPointScale=%f",afcHandle->howlingScaleFactorSmoothed[i]);
		}
	}
	//step 9 :calulate outEn/inEn and smooth it
	for(i=0;i<PART_LEN1;i++)
	{
		afcInEn+=nearSpecAbs[i];		
	}
	// calculate power ratio between input and outPut
	if(afcHandle->howlingPos>minHowlingPos)
	{		
		float deepScaleFactor;
		for(i=0;i<PART_LEN1;i++)
		{
			afcOutEn+=nearSpecAbs[i]*afcHandle->howlingScaleFactorSmoothed[i];		
		}
		if(afcOutEn>afcInEn)
		{
			afcOutEn=afcInEn;
		}
		deepScaleFactor = afcOutEn/afcInEn;
		afcHandle->afcInOutEnRatioSmoothed = afcHandle->afcInOutEnRatioSmoothed + ((afcOutEn/afcInEn)-afcHandle->afcInOutEnRatioSmoothed)*0.1f;//smoothe the afcHandle->afcInOutEnRatio
		//if(afcHandle->afcInOutEnRatio<deepScaleFactor)
		{
			deepScaleFactor = afcHandle->afcInOutEnRatioSmoothed;
		}
		if(afcHandle->howlingPos>minHowlingPos )
		{
			int16_t deepScaleBeginPos=afcHandle->howlingPos*afcHandle->afcInOutEnRatioSmoothed;
			if(deepScaleBeginPos<minHowlingPos)
			{
				deepScaleBeginPos=minHowlingPos;
			}
			for(i=deepScaleBeginPos;i<PART_LEN1;i++)
			{
				efw[i].imag*=powf(deepScaleFactor,i/minHowlingPos);//scale output again
				efw[i].real*=powf(deepScaleFactor,i/minHowlingPos);	//scale output again
			}
		}
	}
	else
	{
		afcHandle->afcInOutEnRatioSmoothed +=0.01f;
	}
	if(afcHandle->afcInOutEnRatioSmoothed<0.0f)
	{
		afcHandle->afcInOutEnRatioSmoothed=0.0f;
	}
	else if(afcHandle->afcInOutEnRatioSmoothed>1.0f)
	{
		afcHandle->afcInOutEnRatioSmoothed=1.0f;
	}
	ret =0;
	if(afcHandle->candidatePos>minHowlingPos)
	{
		ret += 8192;
	}
	if(afcHandle->howlingPos>minHowlingPos)
	{
		ret += 8192;
	}	
	return ret;
}
#endif
int WebRtcAecm_ProcessBlock(AecmCore* aecm,
                            const int16_t* farend,
                            const int16_t* nearendNoisy,
                            const int16_t* nearendClean,
                            int16_t* output) {
  int i;

  uint32_t xfaSum;
  uint32_t dfaNoisySum;
  uint32_t dfaCleanSum;
  uint32_t echoEst32Gained;
  uint32_t tmpU32;

  int32_t tmp32no1;

  uint16_t xfa[PART_LEN1];
  uint16_t dfaNoisy[PART_LEN1];
  uint16_t dfaClean[PART_LEN1];
  uint16_t* ptrDfaClean = dfaClean;
  const uint16_t* far_spectrum_ptr = NULL;

  const ComplexInt16* far_spectrum_ptr_complex = NULL;

#ifdef MUSIC_OR_SPEECH
   const int16_t* far_time_ptr = NULL;
#endif
  // 32 byte aligned buffers (with +8 or +16).
  // TODO(kma): define fft with ComplexInt16.
  int16_t fft_buf[PART_LEN4 + 2 + 16]; // +2 to make a loop safe.
  int32_t echoEst32_buf[PART_LEN1 + 8];
  int32_t dfw_buf[PART_LEN2 + 8];
  int32_t efw_buf[PART_LEN2 + 8];

  int16_t* fft = (int16_t*) (((uintptr_t) fft_buf + 31) & ~ 31);
  int32_t* echoEst32 = (int32_t*) (((uintptr_t) echoEst32_buf + 31) & ~ 31);
  ComplexInt16* dfw = (ComplexInt16*)(((uintptr_t)dfw_buf + 31) & ~31);
  ComplexInt16* efw = (ComplexInt16*)(((uintptr_t)efw_buf + 31) & ~31);
#ifdef AFC
  int16_t howlingPosibility=-ONE_Q14;
#endif


  int16_t hnl[PART_LEN1];
  int16_t numPosCoef = 0;
  int16_t nlpGain = ONE_Q14;
  int delay;
  int16_t tmp16no1;
  int16_t tmp16no2;
  int16_t mu;
  int16_t supGain;
  int16_t zeros32, zeros16;
  int16_t zerosDBufNoisy, zerosDBufClean, zerosXBuf;
 
  int far_q;
  int16_t resolutionDiff, qDomainDiff, dfa_clean_q_domain_diff;

  const int kMinPrefBand = 4;
#ifdef K_MAX_PREF_BAND_MODIFICATION
  const int kMaxPrefBand = 16;
#else
  const int kMaxPrefBand = 24;
#endif
  
  int32_t avgHnl32 = 0;

 
 // int16_t outputValue=0;  
  //printf("aecm->totCount=%d",aecm->totCount);
  nearendClean =NULL;  
#ifdef AEC_DEBUG
  
  if ((nearendNoisy != NULL)&&(aecm->nearFile!=NULL))
  {
	  fwrite(nearendNoisy, sizeof(int16_t), PART_LEN, aecm->nearFile);
  }
  if ((nearendClean != NULL)&&(aecm->nearFile_1!=NULL))
  {
	  fwrite(nearendClean, sizeof(int16_t), PART_LEN, aecm->nearFile_1);
  }

  if ((aecm->farFile_block != NULL)&&(aecm->farFile_block!=NULL))
   {
	   fwrite(farend, sizeof(int16_t), PART_LEN, aecm->farFile_block);
	} 
#endif

  // Determine startup state. There are three states:
  // (0) the first CONV_LEN blocks
  // (1) another CONV_LEN blocks
  // (2) the rest

  if (aecm->startupState < 2)
  {
    aecm->startupState = (aecm->totCount >= CONV_LEN) +
                         (aecm->totCount >= CONV_LEN2);
  }
  // END: Determine startup state

  // Buffer near and far end signals
  memcpy(aecm->xBuf + PART_LEN, farend, sizeof(int16_t) * PART_LEN);
  memcpy(aecm->dBufNoisy + PART_LEN, nearendNoisy, sizeof(int16_t) * PART_LEN);
  if (nearendClean != NULL)
  {
    memcpy(aecm->dBufClean + PART_LEN,
           nearendClean,
           sizeof(int16_t) * PART_LEN);
  }

  // Transform far end signal from time domain to frequency domain.
  far_q = TimeToFrequencyDomain(aecm,
                                aecm->xBuf,
                                dfw,
                                xfa,
                                &xfaSum); 
   // Save far-end history 
  WebRtcAecm_UpdateFarHistory(aecm, xfa, far_q);
 #ifdef MUSIC_OR_SPEECH
  WebRtcAecm_UpdateFarHistoryTime(aecm,(int16_t*)farend); //save the far end signal in time domain
 #endif

  WebRtcAecm_UpdateFarHistoryComplex(aecm, dfw);//save the complex spectrum of far end signal


  if (WebRtc_AddFarSpectrumFix(aecm->delay_estimator_farend, //save the spectrum ampitude of far end
                               xfa,
                               PART_LEN1,
                               far_q) == -1) {
    return -1;
  }


  // Transform noisy near end signal from time domain to frequency domain.
  zerosDBufNoisy = TimeToFrequencyDomain(aecm,
                                         aecm->dBufNoisy,
                                         dfw,
                                         dfaNoisy,
                                         &dfaNoisySum);
  aecm->dfaNoisyQDomainOld = aecm->dfaNoisyQDomain;
  aecm->dfaNoisyQDomain = (int16_t)zerosDBufNoisy; 
  if (nearendClean == NULL)
  {
	memcpy(dfaClean,dfaNoisy,PART_LEN1*sizeof(uint16_t));
    //ptrDfaClean = dfaNoisy;
    aecm->dfaCleanQDomainOld = aecm->dfaNoisyQDomainOld;
    aecm->dfaCleanQDomain = aecm->dfaNoisyQDomain;
    dfaCleanSum = dfaNoisySum;
  } else
  {
    // Transform clean near end signal from time domain to frequency domain.
    zerosDBufClean = TimeToFrequencyDomain(aecm,
                                           aecm->dBufClean,
                                           dfw,
                                           dfaClean,
                                           &dfaCleanSum);
	dfaNoisySum = dfaCleanSum;	
    aecm->dfaCleanQDomainOld = aecm->dfaCleanQDomain;
    aecm->dfaCleanQDomain = (int16_t)zerosDBufClean;
  }

  // Get the delay 
  delay = WebRtc_DelayEstimatorProcessFix(aecm->delay_estimator,
                                          dfaNoisy,
                                          PART_LEN1,
                                          zerosDBufNoisy);

  if (delay == -1)
  {
    return -1;
  }
  else if (delay == -2)
  {
    // If the delay is unknown, we assume zero.
    // NOTE: this will have to be adjusted if we ever add lookahead.
    delay = 0;
  }

  if (aecm->fixedDelay >= 0)
  {
    // Use fixed delay
    delay = aecm->fixedDelay;
  }

  // Get aligned far end spectrum
  far_spectrum_ptr = WebRtcAecm_AlignedFarend(aecm, &far_q, delay); 
#ifdef MUSIC_OR_SPEECH
  far_time_ptr = WebRtcAecm_AlignedFarendTime(aecm, delay);//get aligned far end signal in time domain
#endif

  // calculate the near far complex corr 
  {	   
    const float gCoh[2][2] = {{0.9f, 0.1f}, {0.93f, 0.07f}};   
    const float *ptrGCoh = gCoh[aecm->mult - 1];	
	float nearScale = 1.0f/(1<<aecm->dfaNoisyQDomain);
	float farScale = 1.0f/(1<<far_q);	
	ComplexFloat nearValue,farValue;
	far_spectrum_ptr_complex = WebRtcAecm_AlignedFarend_complex(aecm, delay);
	for(i=0;i<PART_LEN1;i++)
	{
		nearValue.real=dfw[i].real*nearScale;
		nearValue.imag=dfw[i].imag*nearScale;
		farValue.real=far_spectrum_ptr_complex[i].real*farScale;
		farValue.imag=far_spectrum_ptr_complex[i].imag*farScale;
		aecm->nearPow[i] = ptrGCoh[0] * aecm->nearPow[i] + ptrGCoh[1] *    //near signal power
			(nearValue.real * nearValue.real + nearValue.imag * nearValue.imag);
		aecm->farPow[i] = ptrGCoh[0] * aecm->farPow[i] + ptrGCoh[1] *    //far signal power
			(farValue.real * farValue.real + farValue.imag * farValue.imag);
		if(aecm->farPow[i]<15)
		{
			aecm->farPow[i]=15;
		}
		aecm->nearFarPow[i].real = ptrGCoh[0] * aecm->nearFarPow[i].real + ptrGCoh[1] *    
			(nearValue.real * farValue.real +nearValue.imag * farValue.imag);//the real part of near*far
		aecm->nearFarPow[i].imag = ptrGCoh[0] * aecm->nearFarPow[i].imag + ptrGCoh[1] *    
			(nearValue.real * farValue.imag - nearValue.imag * farValue.real);//the image part of near*far
		aecm->nearFarComplexCorr[i]= (aecm->nearFarPow[i].real * aecm->nearFarPow[i].real + aecm->nearFarPow[i].imag * aecm->nearFarPow[i].imag) /
                   (aecm->nearPow[i] * aecm->farPow[i] + 1e-10f);
		if(aecm->nearFarComplexCorr[i]<0)
		{
		  aecm->nearFarComplexCorr[i]=0;
		 }
		else if(aecm->nearFarComplexCorr[i]>1)
		{
		  aecm->nearFarComplexCorr[i]=1;
		}			
	}		
  }
   // calculate the near far complex corr  finish


  zerosXBuf = (int16_t) far_q;
  if (far_spectrum_ptr == NULL)
  {
    return -1;
  }

  // Calculate log(energy) and update energy threshold levels

  aecm->nearSpecPtr = dfaClean;
  WebRtcAecm_CalcEnergies(aecm,
                          far_spectrum_ptr,
                          zerosXBuf,
                          dfaNoisySum,
                          echoEst32);

 {
	 int16_t tmp16_0=0;
	 int16_t tmp16_1=0;	 
	 memmove(&aecm->nearSpecHis[PART_LEN1],&aecm->nearSpecHis[0],(SPEC_BUF_LEN-1)*PART_LEN1*sizeof(uint16_t));//update near spectrum history
     memmove(&aecm->farSpecHis[PART_LEN1],&aecm->farSpecHis[0],(SPEC_BUF_LEN-1)*PART_LEN1*sizeof(uint16_t));//update far spectrum history 
    if(far_q>0)
	{
		tmp16_0 = 1<<(far_q-1);
	}
    if(aecm->dfaCleanQDomain>0)
	{
		tmp16_1 = 1<<(aecm->dfaCleanQDomain-1);
	} 
    for(i=0;i<PART_LEN1;i++)
    {
	  aecm->farSpecHis[i]=(far_spectrum_ptr[i]+tmp16_0)>>far_q;		//update far spectrum history
	  aecm->nearSpecHis[i]=(dfaClean[i]+tmp16_1)>>aecm->dfaCleanQDomain;	 //update near spectrum history
    }
 }

#ifdef NEAR_ERROR_CORR_WRITE
   if(aecm->nearErrorCorrFile!=NULL)
   {
	   int16_t value=0;	  
	   for(i=0;i<PART_LEN;i++)
	   {
		   fwrite(&value,2,1,aecm->nearErrorCorrFile);
	   }
   }
#endif
#ifdef NEAR_FAR_CORR_WRITE
   aecm->nearFarCorr = 0;
   if(aecm->nearFarCorrFile!=NULL)
   {
	   int16_t wrtieValue;
	   aecm->nearFarCorr= WebRtcAecm_calcCorrQ14(dfaClean+12,(uint16_t*)far_spectrum_ptr+12,aecm->dfaCleanQDomain,far_q,32);		 
		wrtieValue= aecm->nearFarCorr;
		
	   for(i=0;i<PART_LEN;i++)
	   {
		   fwrite(&wrtieValue,2,1,aecm->nearFarCorrFile);
	   }
   }
#endif

  //calculate the nearFarBitCountCorr and smooth it
  {	 
	  float smoothFactor = 0.95f;//15565 0.95 q14
	  uint32_t nearFarBitCountCorr = (32 * 512 - 512 * ((DelayEstimator*)aecm->delay_estimator)->binary_handle->bit_counts[delay]);//32*512-  mean_bit_counts . Q value of mean_bit_counts is 9
	  nearFarBitCountCorr = aecm->nearFarBitCountCorrHis[0]*smoothFactor +nearFarBitCountCorr*(1.0f- smoothFactor);	 
	  if(nearFarBitCountCorr>ONE_Q14)
	  {
		  nearFarBitCountCorr=ONE_Q14;
	  }	  
	  memmove(&aecm->nearFarBitCountCorrHis[1], &aecm->nearFarBitCountCorrHis[0], (MAX_BUF_LEN - 1)*sizeof(uint32_t));
	  aecm->nearFarBitCountCorrHis[0] = nearFarBitCountCorr;
  }


  //estimate the energy ratio between echo and far signal 
  aecm->echoFarEnRatioNeed=0;  
  if(aecm->mseChannelCount>=(MIN_MSE_COUNT+ 10)/2)
 {
	WebRtcAecm_echoFarEnRatio(aecm,1);  
  }


  //update the history of the flag whether the  speaker is on or off
  memmove(&aecm->AecmModeArray[1],&aecm->AecmModeArray[0],1023*sizeof(int16_t));
  aecm->AecmModeArray[0]=aecm->AecmMode;
  if((aecm->AecmModeArray[0]>DEFAULT_ECHO_MODE)&&(aecm->AecmModeArray[1]<=DEFAULT_ECHO_MODE))
  {
	 aecm->fromReceiverToSpeaker=1;//switch from receiver to speaker
	 aecm->speakerGain=4;
	 aecm->fromSpeakerToReceiver=0;
  }
  if((aecm->AecmModeArray[0]<=DEFAULT_ECHO_MODE)&&(aecm->AecmModeArray[1]>DEFAULT_ECHO_MODE))
  {
	  aecm->speakerGain=1;
	 aecm->fromReceiverToSpeaker=0;
	 aecm->fromSpeakerToReceiver=1;//switch from speaker  to receiver
  }
  

  //find out the main energy bands of far signal. we can scale down the wiener filer coefs in main bands 
  //if(aecm->currentVADValue)
  {
	 int16_t m;
	 float farSum=0.0f;		
	 int16_t lowIndex=PART_LEN1-1;
	 int16_t mostIndex=0;
	 float mostLimitedValue=0.0f;	 
	 int16_t sortLen=PART_LEN1>>(aecm->mult-1);
	 float lamda=0.5f;
	 if(aecm->AecmMode>DEFAULT_ECHO_MODE)
	 {
		 lamda=0.9f;
	 }
	 memset(aecm->farMainBandFlag,0,PART_LEN1*sizeof(uint16_t));
	 if(aecm->totCount<2)
	 {
		 memset(aecm->farSmoothedValue,0,PART_LEN1*sizeof(uint16_t));
	 }
	 for(m=0;m<sortLen;m++)
	 {   // smooth the far spectrum  : fast increase and slow decrease
		 if(aecm->farSpecHis[m]<aecm->farSmoothedValue[m])
		 {
			float lamdaTmp =lamda*m*aecm->mult*4/PART_LEN1;
			if(lamdaTmp>lamda)
			{
				lamdaTmp=lamda;
			}
			aecm->farSmoothedValue[m]=aecm->farSmoothedValue[m]*lamdaTmp*+aecm->farSpecHis[m]*(1.0f-lamdaTmp);
		 }
		 else
		 {
			 aecm->farSmoothedValue[m]=aecm->farSmoothedValue[m]*0.1f+aecm->farSpecHis[m]*0.9f;
		 }
	 }
	 if((aecm->AecmMode>DEFAULT_ECHO_MODE)&&(aecm->echoFarEnRatio>256))
	 {
		memcpy(aecm->farSortedValue, aecm->farSmoothedValue, sizeof(uint16_t) * sortLen);
		qsort(aecm->farSortedValue,  sortLen, sizeof(uint16_t), CmpUint16_t);
		for(m=0;m<sortLen;m++)
		{
			farSum+=aecm->farSortedValue[m];
		}
		 mostLimitedValue =farSum*0.8f;
		 for(m=0;m<sortLen;m++)
		{
			if(aecm->farSortedValue[m]<=0)
			{
				break;
			}
			mostLimitedValue-=aecm->farSortedValue[m];
			if(mostLimitedValue>0)
			{
				 mostIndex=m;
			}
			if(aecm->farSortedValue[m]*PART_LEN1>4*farSum)
			{
				lowIndex = m;				 
			 }
		 }
		if(lowIndex>mostIndex)
		{
			lowIndex = mostIndex;
		}
		if(lowIndex*aecm->mult*2>sortLen)
		{
			lowIndex = sortLen/(aecm->mult*2);
		}
	
		for(m=0;m<PART_LEN1;m++)
		{
			if(aecm->farSmoothedValue[m]>aecm->farSortedValue[lowIndex])
			{
				aecm->farMainBandFlag[m]=1;// the mth band is one of the main energy bands. 
			}
		}
	 }//end if(aecm->AecmMode>DEFAULT_ECHO_MODE)
  }
  //finish find out the main energy bands of far signal.


  // Calculate stepsize
  mu = WebRtcAecm_CalcStepSize(aecm); 
  // Update counters
  aecm->totCount++;
  // This is the channel estimation algorithm.
  // It is base on NLMS but has a variable step length,which was calculated above.
  WebRtcAecm_UpdateChannel(aecm,
                           far_spectrum_ptr,
                           zerosXBuf,
                           ptrDfaClean,//dfaNoisy,
                           mu,
                           echoEst32); 
#ifdef MUSIC_OR_SPEECH
  memset(output,0,PART_LEN*sizeof(int16_t));
  getTimedomainSignalAfterLms(aecm,	 fft, dfw,output,ptrDfaClean,echoEst32,far_q);
  WebRtcAecm_updateSpeechMusicMode(aecm,far_time_ptr,output,PART_LEN);
#endif

  if(aecm->echoFarEnRatioNeed==1)
  {
	 WebRtcAecm_echoFarEnRatio(aecm,2);  
  }
 supGain = WebRtcAecm_CalcSuppressionGain(aecm);
 
 if( aecm->speakerGain>1)
 {
	supGain *= aecm->speakerGain;// if we just switch from receiver to sperker, we should increase the supGain
 }



	
  // Calculate Wiener filter coef:hnl[]
  for (i = 0; i < PART_LEN1; i++)
  {
	   int16_t supGainTmp=supGain;//chgx:supGain is calculated from function WebRtcAecm_CalcSuppressionGain(), and here it is used as a base value 
	   supGainTmp=(supGain-256)*aecm->nearFarComplexCorr[i]+256;//chgx: adjusts supGainTmp with nearFarComplexCorr[i]
	   supGainTmp = supGainTmp + (supGain-supGainTmp)*aecm->nlpLevel*0.25f;//chgx: adjusts supGainTmp with nlpLevel
	  if(supGainTmp>supGain)
	  {
		  supGainTmp=supGain;
	  }	 
    // Far end signal through channel estimate in Q8
    // How much can we shift right to preserve resolution 
	   tmp32no1 = echoEst32[i] - aecm->echoFilt[i];//echo of current block -eho of pre block
	  // chgx modification begin:
	   if(tmp32no1<0)
	   {
		   // calculate decrease rate
		   float disScale=(0.75f+0.2f*i/PART_LEN1);
		   if((aecm->AecmMode<=DEFAULT_ECHO_MODE)&&(aecm->echoFarEnRatio>256))
		   {
			   disScale = disScale*0.5f;

			   if(aecm->farMainBandFlag[i]<=1)
				{
			       disScale = disScale*0.5f;
				}

			    aecm->echoFilt[i] = aecm->echoFilt[i]*disScale+echoEst32[i]*(1-disScale);
		   }
		   else 
		   {
			   aecm->echoFilt[i] += (tmp32no1 * 50) >> 8;
		   }		  
	   }
	   else
	   {
		   aecm->echoFilt[i] += (tmp32no1 * 50) >> 8;//aecm->echoFilt[i] = echoEst32[i];
	   }
       //aecm->echoFilt[i] += (tmp32no1 * 50) >> 8;
#ifdef MUSIC_OR_SPEECH
	   if((aecm->nearIsSpeech!=1)&&(aecm->nearBandTonality[i])>aecm->farBandTonality[i])
	   {
			aecm->echoFilt[i] = echoEst32[i];//music
			supGainTmp =256;//music
	   }
#endif
	  

    zeros32 = WebRtcSpl_NormW32(aecm->echoFilt[i]) + 1;
	zeros16 = WebRtcSpl_NormW16(supGainTmp) + 1;
    if (zeros32 + zeros16 > 16)
    {
      // Multiplication is safe
      // Result in
      // Q(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN+
      //   aecm->xfaQDomainBuf[diff])
      echoEst32Gained = WEBRTC_SPL_UMUL_32_16((uint32_t)aecm->echoFilt[i],
		  (uint16_t)supGainTmp);
      resolutionDiff = 14 - RESOLUTION_CHANNEL16 - RESOLUTION_SUPGAIN;
      resolutionDiff += (aecm->dfaCleanQDomain - zerosXBuf);
    } else
    {
      tmp16no1 = 17 - zeros32 - zeros16;
      resolutionDiff = 14 + tmp16no1 - RESOLUTION_CHANNEL16 -
                       RESOLUTION_SUPGAIN;
      resolutionDiff += (aecm->dfaCleanQDomain - zerosXBuf);
      if (zeros32 > tmp16no1)
      {
        echoEst32Gained = WEBRTC_SPL_UMUL_32_16((uint32_t)aecm->echoFilt[i],
			supGainTmp >> tmp16no1);
      } else
      {
        // Result in Q-(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN-16)
		  echoEst32Gained = (aecm->echoFilt[i] >> tmp16no1) * supGainTmp;
      }
    }

    zeros16 = WebRtcSpl_NormW16(aecm->nearFilt[i]);
    assert(zeros16 >= 0);  // |zeros16| is a norm, hence non-negative.
    dfa_clean_q_domain_diff = aecm->dfaCleanQDomain - aecm->dfaCleanQDomainOld;
    if (zeros16 < dfa_clean_q_domain_diff && aecm->nearFilt[i]) {
      tmp16no1 = aecm->nearFilt[i] << zeros16;
      qDomainDiff = zeros16 - dfa_clean_q_domain_diff;
      tmp16no2 = ptrDfaClean[i] >> -qDomainDiff;
    } else {
      tmp16no1 = dfa_clean_q_domain_diff < 0
          ? aecm->nearFilt[i] >> -dfa_clean_q_domain_diff
          : aecm->nearFilt[i] << dfa_clean_q_domain_diff;
      qDomainDiff = 0;
      tmp16no2 = ptrDfaClean[i];
    }

    tmp32no1 = (int32_t)(tmp16no2 - tmp16no1);
    tmp16no2 = (int16_t)(tmp32no1 >> 4);
	// tmp16no2 = (int16_t)(tmp32no1);
    tmp16no2 += tmp16no1;

    zeros16 = WebRtcSpl_NormW16(tmp16no2);
    if ((tmp16no2) & (-qDomainDiff > zeros16)) {
      aecm->nearFilt[i] = WEBRTC_SPL_WORD16_MAX;
    } else {
      aecm->nearFilt[i] = qDomainDiff < 0 ? tmp16no2 << -qDomainDiff
                                          : tmp16no2 >> qDomainDiff;
    }

    // Wiener filter coefficients, resulting hnl in Q14
    if (echoEst32Gained == 0)
    {
      hnl[i] = ONE_Q14;
    } else if (aecm->nearFilt[i] == 0)
    {
      hnl[i] = 0;
    } else
    {
      // Multiply the suppression gain
      // Rounding
      echoEst32Gained += (uint32_t)(aecm->nearFilt[i] >> 1);
      tmpU32 = WebRtcSpl_DivU32U16(echoEst32Gained,
                                   (uint16_t)aecm->nearFilt[i]);

      // Current resolution is
      // Q-(RESOLUTION_CHANNEL+RESOLUTION_SUPGAIN- max(0,17-zeros16- zeros32))
      // Make sure we are in Q14
      tmp32no1 = (int32_t)WEBRTC_SPL_SHIFT_W32(tmpU32, resolutionDiff);
      if (tmp32no1 > ONE_Q14)
      {
        hnl[i] = 0;
      } else if (tmp32no1 < 0)
      {
        hnl[i] = ONE_Q14;
      } else
      {
        // 1-echoEst/dfa
        hnl[i] = ONE_Q14 - (int16_t)tmp32no1;
        if (hnl[i] < 0)
        {
		  hnl[i] = 0;
        }
      }
    }	
	if (hnl[i])
	{
		numPosCoef++;
	}
  }
  // finish calculate Wiener filter coefs: hnl[]

   

  // nonlinear processing  stage 1: if near signal is music, skip this nlp step
#ifdef MUSIC_OR_SPEECH
  if((aecm->speakerGain<2) &&(1==aecm->nearIsSpeech)) 
#else
 if(aecm->speakerGain<2) 
#endif 
  {
	  int32_t enTmp;
	  int32_t effectEn=0;
	  int32_t nearEn=0;
	  int32_t farEn=0;	 
	  for (i = 0; i < PART_LEN1/aecm->mult; i++)
      {
		  enTmp = aecm->nearSpecHis[i]*hnl[i];
		  effectEn +=enTmp>>14;		// the sum of residual signal energy 
		  nearEn +=aecm->nearSpecHis[i];

		  if((aecm->AecmMode>DEFAULT_ECHO_MODE)&&(aecm->farMainBandFlag[i]==1))
		  {
			  farEn +=aecm->farSmoothedValue[i];			
		  }
		  else
		  {
			  farEn +=aecm->farSpecHis[i];
		  }		  
	  }
	 aecm->currentResidualEnAvg = 0.90f*aecm->currentResidualEnAvg+0.10f*effectEn;//smooth it
	 aecm->nearEnavg = 0.90f*aecm->nearEnavg+0.10f*nearEn;//smooth it
	 if(aecm->currentResidualEnAvg*10>aecm->nearEnavg)// if this condition is true,it means that double talk may be happen
	 {
		float echoNearRatio=1.0f;
		int16_t estimateHighHNL,estimateLowHNL;
		echoNearRatio =1.0*aecm->echoFarEnRatio*farEn/(nearEn+1)/1024.0f;	//calculate the energy between echo and near signal: echo/near = (echo/far)*far/near
		if(echoNearRatio>=0.9f)//clamp the value of echoNearRatio
		{
			echoNearRatio=0.9f;				
		}
		estimateHighHNL = ONE_Q14*(1.0-echoNearRatio)*1.25f;//estimate the possibile max value of hnl[i]
		estimateLowHNL = ONE_Q14*(1.0-echoNearRatio)*0.55f;	//estimate the possibile value value of hnl[i]	
		// make sure estimateHighHNL and  estimateLowHNL are reasonable
		if(estimateHighHNL>ONE_Q14)
		{
			estimateHighHNL=ONE_Q14;
		}	
		else if(estimateHighHNL<0)
		{
			estimateHighHNL=0;
		}
		if(estimateLowHNL>estimateHighHNL)
		{
			estimateLowHNL=estimateHighHNL;
		}	
		if(estimateLowHNL<0)
		{
			estimateLowHNL=0;
		}

		// repair every hnl[i] according estimateHighHNL、 estimateLowHNL、double talk flag、nearFarComplexCorr、farMainBandFlag
		for (i = 0; i < PART_LEN1; i++)
		{
			if((hnl[i]>=estimateHighHNL)&&(aecm->corrDtFlag==0))// if no double talk and hnl[i] is bigger than estimateHighHNL
			{
				hnl[i]=estimateHighHNL;//force  the hnl[i]  not bigger than estimateHighHNL
			}
			else if(hnl[i]<=estimateLowHNL)
			{
				if( aecm->farMainBandFlag[i]==0)
				{
				   if((aecm->nearFarComplexCorr[i]*PART_LEN1<0.75f-0.25f*aecm->mult*i)&&(aecm->farMainBandFlag[i]==0))
				   {
						hnl[i]=estimateLowHNL;//force  the hnl[i]  not small than estimateLowHNL
				   }
				}
			}			
		 }	//end for		
	  }	
	 //recalculate numPosCoef
	 {
	   numPosCoef =0;
	   for (i = 0; i < PART_LEN1; i++)
	   {
		   if(hnl[i]>0)
		   {
			  numPosCoef++;
		   }
	   }
	 }
  }
#ifdef MUSIC_OR_SPEECH
   else if((1==aecm->nearIsSpeech)&&(aecm->corrDtFlag==0)&&(aecm->currentVADValue==1))//music
#else
  else if((aecm->corrDtFlag==0)&&(aecm->currentVADValue==1))//if no double talk and far end is talking, set all hnl[i] to zero 
#endif
 
  {
	  for (i = 0; i < PART_LEN1; i++)
	   {
		   hnl[i] =0;		  
	   }
  }
  // nonlinear processing  stage 1  finish


  // check if near has echo or not. this falg can used in agc module
  aecm->stream_has_echo = 0; 
  if (aecm->currentResidualEnAvg <aecm->nearEnavg*0.75f)//  if echoFarEnRatio is less than 0.5 and the residual energy is less than 0.25*nearEn, we guess all the residual is echo.
  {		
	  aecm->stream_has_echo=1;
  }


  //nonlinear processing  stage 2: Only in wideband when near signal is speech. Prevent the gain in upper band from being larger than  in lower band.
  #ifdef MUSIC_OR_SPEECH
  if ((aecm->mult == 2)&&(1==aecm->nearIsSpeech))
#else
  if (aecm->mult == 2)
#endif
  {    
	int16_t kMaxPrefBandAdjusted = kMaxPrefBand+1024/((aecm->echoFarEnRatio>>2)+32);
	kMaxPrefBandAdjusted = kMaxPrefBandAdjusted+(16-kMaxPrefBandAdjusted)*0.25f*aecm->nlpLevel;
	if(kMaxPrefBandAdjusted>48)
	{
		kMaxPrefBandAdjusted=48;
	}
   /*for (i = 0; i < PART_LEN1; i++)
    {
      hnl[i] = (int16_t)((hnl[i] * hnl[i]) >> 14);
    }*/

    for (i = kMinPrefBand; i <= kMaxPrefBandAdjusted; i++)
    {
      avgHnl32 += (int32_t)hnl[i];
    }
    assert(kMaxPrefBandAdjusted - kMinPrefBand + 1 > 0);
    avgHnl32 /= (kMaxPrefBandAdjusted - kMinPrefBand + 1);

    for (i = kMaxPrefBandAdjusted; i < PART_LEN1; i++)
    {
		if(hnl[i]<0)
		  {
			  hnl[i]=0;
		  }
      if (hnl[i] > (int16_t)avgHnl32)
      {
        hnl[i] = (int16_t)avgHnl32;//make the wiener coef of the high frequency bin is not bigger than avgHnl32
		
      }
    }
  }
  //nonlinear processing  stage 2 finish

  //nonlinear processing  stage 3: Only when the speaker is on or echoFarEnRatio is bigger than 0.25f. smooth spectrum peaks and hnl peaks.  
#ifdef MUSIC_OR_SPEECH
  if((aecm->mult>0)&&((aecm->AecmMode>DEFAULT_ECHO_MODE)||(aecm->echoFarEnRatio>256))&&(1==aecm->nearIsSpeech)) //music
#else
  if((aecm->mult>0)&&((aecm->AecmMode>DEFAULT_ECHO_MODE)||(aecm->echoFarEnRatio>256)))
#endif 
  {
	  int32_t nearSum=0;
	  int32_t outSum=0;	 
	  int32_t tmp;
	  int16_t beginBan=2*aecm->mult;
	  int16_t stopBan=PART_LEN1/(2*aecm->mult);
	  int16_t residualTmp;
	  int16_t avgOut;	 
	  for (i =beginBan; i<stopBan; i++)
	  {
		  nearSum+=aecm->nearSpecHis[i];		  
		  if(aecm->nearSpecHis[i]<32767)
		  {
			tmp = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i],hnl[i], 14));
		  }
		  else
		  {
			  tmp = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i]*0.5f,hnl[i], 13));
		  }
		  outSum+=tmp;
	  }
	  avgOut = outSum/(stopBan-beginBan);
	 
	  if((outSum<nearSum)&&(outSum>0.01*nearSum))
	  {
		  float scale=1.0f*outSum/nearSum;
		   for (i = stopBan; i < PART_LEN1; i++)
           {
			    if(aecm->nearSpecHis[i]<32767)
				{
					residualTmp = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i],hnl[i], 14));
				}
				else
				{
					residualTmp = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i]*0.5f,hnl[i], 13));				}
			  
			   if((residualTmp>avgOut))
			   {					
				   float hnlFloat = 1.0f*avgOut/aecm->nearSpecHis[i];
				   hnl[i] = hnlFloat*ONE_Q14;
				   if(hnl[i]<0)
					{
						 hnl[i]=0;
					}
			   }
			   else if(hnl[i]>scale*ONE_Q14)
			   {
				   hnl[i]=scale*ONE_Q14;
				   if(hnl[i]<0)
					{
						 hnl[i]=0;
					}
			   }
		   }
	  }
  }  
  //nonlinear processing  stage 3 finish

  //nonlinear processing  stage 4: Only when the speaker is on and echoFarEnRatio is bigger than 0.25f.  eliminate small residual signal 
#ifdef MUSIC_OR_SPEECH	  
   if((aecm->AecmMode>DEFAULT_ECHO_MODE)&&(aecm->echoFarEnRatio>256)&&(1==aecm->nearIsSpeech))
#else
   if((aecm->AecmMode>DEFAULT_ECHO_MODE)&&(aecm->echoFarEnRatio>256))//music
#endif
  {	 
	  int32_t effectEn=0;
	  int32_t nearEn=0;	 
	  for (i = 0; i < PART_LEN1/aecm->mult; i++)
      {
		  if(aecm->nearSpecHis[i]<16384)
		  {
			effectEn += WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i],hnl[i], 14);	
		  }
		  else
		  {
			  effectEn += WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i]>>1,hnl[i], 13);
		  }
		  nearEn +=aecm->nearSpecHis[i];		  
	  }
	  if(nearEn>(10-aecm->nlpLevel)*effectEn)
	  {
		  memset(hnl,0,PART_LEN1*sizeof(int16_t));//residual is too small,set all hnl[i] to zero
	  }	  
  }
   //nonlinear processing  stage 4 finish


    //tailLen control  
  if(aecm->AecmMode>DEFAULT_ECHO_MODE && aecm->echoFarEnRatio>256)// if the speaker is on and echo is big,we should control the tail len   
   {
	   uint16_t errorSpec[PART_LEN1];
	   float errorEn=0;
	   float nearEn=0;	  
	   int16_t maxCorr=0;
	    for (i = 0; i < PART_LEN1; i++)
		{
			errorSpec[i]=(int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(aecm->nearSpecHis[i],hnl[i], 14));
			errorEn +=errorSpec[i];
			nearEn +=aecm->nearSpecHis[i];			
		}
		if(errorEn>nearEn*0.75f)
		{
			int16_t tailLenMs = aecm->tailLenMs;
			int16_t searchNum=tailLenMs*aecm->mult/8;
			
			int16_t corrTmp;
			for(i=0;i<searchNum;i++)
			{
				
				corrTmp =  WebRtcAecm_calcCorrQ14(errorSpec,&aecm->farSpecHis[i*PART_LEN1],0,0,PART_LEN1);				
				if(corrTmp>maxCorr )
				{
					maxCorr = corrTmp;
				}
			}
			if(maxCorr>0)
			{
				float scale=1.0f*(ONE_Q14-maxCorr)/ONE_Q14;
				 for (i = 0; i < PART_LEN1; i++)
				{
					hnl[i]*=scale;					
				 }
			}
		}
#ifdef MUSIC_OR_SPEECH
		if ((aecm->farFile_aligned != NULL)&&(far_time_ptr!=NULL))
		{
			for (i = 0; i < PART_LEN; i++)
			{
				fwrite(&maxCorr, sizeof(int16_t), 1, aecm->farFile_aligned);
			}
		}
#endif
		
	}
   //tailLen control end


  // Calculate NLP gain, result is in Q14 
#ifdef MUSIC_OR_SPEECH	
     if (aecm->nlpFlag && (1==aecm->nearIsSpeech))//music
#else
  if (aecm->nlpFlag)//music
#endif
  {
    for (i = 0; i < PART_LEN1; i++)
    {
      // Truncate values close to zero and one.
      if (hnl[i] > NLP_COMP_HIGH)
      {
        hnl[i] = ONE_Q14;
	  } else if (hnl[i] < NLP_COMP_LOW*(1.0f+aecm->nlpLevel*0.05f))
      {
        hnl[i] = 0;		
      }
      // Remove outliers
      if (numPosCoef < 3)
      {
        nlpGain = 0;
      } else
      {
        nlpGain = ONE_Q14;
      }
	   
      // NLP
      if ((hnl[i] == ONE_Q14) && (nlpGain == ONE_Q14))
      {
        hnl[i] = ONE_Q14;
      } else
      {
        hnl[i] = (int16_t)((hnl[i] * nlpGain) >> 14);
      }	  
      // multiply with Wiener coefficients	  
      efw[i].real = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(dfw[i].real,
                                                                   hnl[i], 14));
      efw[i].imag = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(dfw[i].imag,
                                                            hnl[i], 14));
    }
  }
  else
  {
    // multiply with Wiener coefficients
    for (i = 0; i < PART_LEN1; i++)
    {
		
      efw[i].real = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(dfw[i].real,
                                                                   hnl[i], 14));
      efw[i].imag = (int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(dfw[i].imag,
                                                                   hnl[i], 14));	  
    }
  }

  //howling cancellation
#ifdef AFC  
  if(aecm->AecmMode>DEFAULT_ECHO_MODE && aecm->afcEnable>0)
  {
	 howlingPosibility=howlingCancellation(&aecm->afcInstance,aecm->nearSpecHis,aecm->mult*8000,aecm->totCount,efw);	
	  aecm->farEndScaleFactorSpeakerOn = 896*aecm->afcInstance.afcInOutEnRatioSmoothed;//scale down the far signal
	 // printf(" howPos=%d \n",howPos);	  
   }
  else 
  {
	  aecm->farEndScaleFactorSpeakerOn = 896;
  }
#endif
  //howling cancellation finish
  
if ((aecm->mult == 2) && (aecm->AecmMode == 4)&&(1==aecm->cutHighFrequencyEnable))
{
 	for (i = (PART_LEN1>>1); i < PART_LEN1; i++)
 	{		
 		efw[i].real = 0;
 		efw[i].imag = 0;
		hnl[i]=0;
 	}
}

#ifdef SUPPORT_32K  
  if(aecm->processBlockCnt>=3)
  {
	  aecm->processBlockCnt=0;
  }
  else if(aecm->processBlockCnt<0)
  {
	  aecm->processBlockCnt=0;
  }
  for (i = 0; i <PART_LEN1; i++)
  {
	  aecm->HnlInLowBand16k[aecm->processBlockCnt][i]=hnl[i];
  }
  aecm->avgHnlInLowBand16k[aecm->processBlockCnt]=0;
  for (i = (PART_LEN1>>1); i < PART_LEN1; i++)
  {
	  aecm->avgHnlInLowBand16k[aecm->processBlockCnt]+=hnl[i];
  }
  aecm->avgHnlInLowBand16k[aecm->processBlockCnt]/=((PART_LEN1+1)>>1)+2;
  aecm->processBlockCnt++;
#endif

 if (aecm->cngMode == AecmTrue)
  {
    ComfortNoise(aecm, ptrDfaClean, efw, hnl);
  }

  InverseFFTAndWindow(aecm, fft, efw, output, nearendClean);
#ifdef AEC_DEBUG  
  if(aecm->outFile)
  {
	  fwrite(output, sizeof(int16_t), PART_LEN, aecm->outFile);
  } 
  if(aecm->speakerModeFile)
  {
	  short outputValue =aecm->AecmMode*4096;// efw[i].real	
  #ifdef AFC
	  outputValue = howlingPosibility;//aecm->farEndScaleFactorSpeakerOn*10;//howlingPosibility;
  #endif
	  for(i=0;i<PART_LEN;i++)
	  {
		  fwrite(&outputValue, sizeof(int16_t), 1, aecm->speakerModeFile);//write debug file
	  }
  }
#endif
  return 0;
}

static void ComfortNoise(AecmCore* aecm,
                         const uint16_t* dfa,
                         ComplexInt16* out,
                         const int16_t* lambda) {
  int16_t i;
  int16_t tmp16;
  int32_t tmp32;

  int16_t randW16[PART_LEN];
  int16_t uReal[PART_LEN1];
  int16_t uImag[PART_LEN1];
  int32_t outLShift32;
  int16_t noiseRShift16[PART_LEN1];

  int16_t shiftFromNearToNoise = kNoiseEstQDomain - aecm->dfaCleanQDomain;
  int16_t minTrackShift;

  assert(shiftFromNearToNoise >= 0);
  assert(shiftFromNearToNoise < 16);

  if (aecm->noiseEstCtr < 100)
  {
    // Track the minimum more quickly initially.
    aecm->noiseEstCtr++;
    minTrackShift = 6;
  } else
  {
    minTrackShift = 9;
  }

  // Estimate noise power.
  for (i = 0; i < PART_LEN1; i++)
  {
    // Shift to the noise domain.
    tmp32 = (int32_t)dfa[i];
    outLShift32 = tmp32 << shiftFromNearToNoise;

    if (outLShift32 < aecm->noiseEst[i])
    {
      // Reset "too low" counter
      aecm->noiseEstTooLowCtr[i] = 0;
      // Track the minimum.
      if (aecm->noiseEst[i] < (1 << minTrackShift))
      {
        // For small values, decrease noiseEst[i] every
        // |kNoiseEstIncCount| block. The regular approach below can not
        // go further down due to truncation.
        aecm->noiseEstTooHighCtr[i]++;
        if (aecm->noiseEstTooHighCtr[i] >= kNoiseEstIncCount)
        {
          aecm->noiseEst[i]--;
          aecm->noiseEstTooHighCtr[i] = 0; // Reset the counter
        }
      }
      else
      {
        aecm->noiseEst[i] -= ((aecm->noiseEst[i] - outLShift32)
                              >> minTrackShift);
      }
    } else
    {
      // Reset "too high" counter
      aecm->noiseEstTooHighCtr[i] = 0;
      // Ramp slowly upwards until we hit the minimum again.
      if ((aecm->noiseEst[i] >> 19) > 0)
      {
        // Avoid overflow.
        // Multiplication with 2049 will cause wrap around. Scale
        // down first and then multiply
        aecm->noiseEst[i] >>= 11;
        aecm->noiseEst[i] *= 2049;
      }
      else if ((aecm->noiseEst[i] >> 11) > 0)
      {
        // Large enough for relative increase
        aecm->noiseEst[i] *= 2049;
        aecm->noiseEst[i] >>= 11;
      }
      else
      {
        // Make incremental increases based on size every
        // |kNoiseEstIncCount| block
        aecm->noiseEstTooLowCtr[i]++;
        if (aecm->noiseEstTooLowCtr[i] >= kNoiseEstIncCount)
        {
          aecm->noiseEst[i] += (aecm->noiseEst[i] >> 9) + 1;
          aecm->noiseEstTooLowCtr[i] = 0; // Reset counter
        }
      }
    }
  }

  for (i = 0; i < PART_LEN1; i++)
  {
    tmp32 = aecm->noiseEst[i] >> shiftFromNearToNoise;
    if (tmp32 > 32767)
    {
      tmp32 = 32767;
      aecm->noiseEst[i] = tmp32 << shiftFromNearToNoise;
    }
    noiseRShift16[i] = (int16_t)tmp32;

    tmp16 = ONE_Q14 - lambda[i];
    noiseRShift16[i] = (int16_t)((tmp16 * noiseRShift16[i]) >> 14);
  }

  // Generate a uniform random array on [0 2^15-1].
  WebRtcSpl_RandUArray(randW16, PART_LEN, &aecm->seed);

  // Generate noise according to estimated energy.
  uReal[0] = 0; // Reject LF noise.
  uImag[0] = 0;
  for (i = 1; i < PART_LEN1; i++)
  {
    // Get a random index for the cos and sin tables over [0 359].
    tmp16 = (int16_t)((359 * randW16[i - 1]) >> 15);

    // Tables are in Q13.
    uReal[i] = (int16_t)((noiseRShift16[i] * WebRtcAecm_kCosTable[tmp16]) >>
        13);
    uImag[i] = (int16_t)((-noiseRShift16[i] * WebRtcAecm_kSinTable[tmp16]) >>
        13);
  }
  uImag[PART_LEN] = 0;

  for (i = 0; i < PART_LEN1; i++)
  {
    out[i].real = WebRtcSpl_AddSatW16(out[i].real, uReal[i]);
    out[i].imag = WebRtcSpl_AddSatW16(out[i].imag, uImag[i]);
  }
}

