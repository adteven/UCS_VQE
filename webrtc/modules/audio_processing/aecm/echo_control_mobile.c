/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"

#ifdef AEC_DEBUG
#include <stdio.h>
#endif
#include <stdlib.h>

#include "webrtc/common_audio/ring_buffer.h"
#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"
#include "webrtc/modules/audio_processing/aecm/aecm_core.h"

#define BUF_SIZE_FRAMES 50 // buffer size (frames)
// Maximum length of resampled signal. Must be an integer multiple of frames
// (ceil(1/(1 + MIN_SKEW)*2) + 1)*FRAME_LEN
// The factor of 2 handles wb, and the + 1 is as a safety margin
#define MAX_RESAMP_LEN (5 * FRAME_LEN)

static const size_t kBufSizeSamp = BUF_SIZE_FRAMES * FRAME_LEN; // buffer size (samples)
static const int kSampMsNb = 8; // samples per ms in nb
// Target suppression levels for nlp modes
// log{0.001, 0.00001, 0.00000001}
static const int kInitCheck = 42;

typedef struct
{
    int sampFreq;
    int scSampFreq;
    short bufSizeStart;
    int knownDelay;

    // Stores the last frame added to the farend buffer
    short farendOld[2][FRAME_LEN];
    short initFlag; // indicates if AEC has been initialized

    // Variables used for averaging far end buffer size
    short counter;
    short sum;
    short firstVal;
    short checkBufSizeCtr;

    // Variables used for delay shifts
    short msInSndCardBuf;
    short filtDelay;
    int timeForDelayChange;
    int ECstartup;
    int checkBuffSize;
    int delayChange;
    short lastDelayDiff;

    //int16_t echoMode;


    // Structures
    RingBuffer *farendBuf;

    int lastError;

    AecmCore* aecmCore;
#ifdef SUPPORT_32K
#ifdef AEC_DEBUG
	FILE* farFile32;
	FILE* nearFile32;
#endif
	int16_t inOutSampleRate;	
	int32_t resampleState[32];
	int32_t resampleOut[320];
	int16_t resampleOutShort[320];
	int32_t resampleStateNear[32];
	int32_t resampleOutNear[320];
	int16_t resampleOutShortNear[320];
	int16_t resampleOutShortNearClean[320];
	int32_t resampleStateOutUpBy2[32];
	int32_t resampleStateOutUpBy2forResidual[32];
	struct RealFFT* real_fft_for32k;
	RingBuffer* nearNoisyFrameBufFor32k;	
	RingBuffer* outFrameBufFor32k;
	int16_t nearBufFor32k[256];
	int16_t outBufFor32k[256];
#endif

} AecMobile;

// Estimates delay to set the position of the farend buffer read pointer
// (controlled by knownDelay)
#ifndef CLOSE_EST_BUF_DELAY
static int WebRtcAecm_EstBufDelay(AecMobile* aecmInst, short msInSndCardBuf);
#endif

// Stuffs the farend buffer if the estimated delay is too large
#ifndef CLOSE_DELAY_COMPARE
static int WebRtcAecm_DelayComp(AecMobile* aecmInst);
#endif

#ifdef RECORD_FILE_WITH_TIME
#ifndef PC_DEMO_VERSION
/*    time_t to string*/
int TimeToString(string &strDateStr,const time_t &timeData)
{
    char chTmp[15];
    bzero(chTmp,sizeof(chTmp));
 
    struct tm *p;
    p = localtime(&timeData);
 
    p->tm_year = p->tm_year + 1900;
 
    p->tm_mon = p->tm_mon + 1;
 
 
    snprintf(chTmp,sizeof(chTmp),"%04d-%02d-%02d",
        p->tm_year, p->tm_mon, p->tm_mday);
 
    strDateStr = chTmp;
    return 0;
}
#endif
#endif

int32_t WebRtcAecm_Create(void **aecmInst)
{
  AecMobile* aecm;
    if (aecmInst == NULL)
    {
        return -1;
    }

    aecm = malloc(sizeof(AecMobile));
    *aecmInst = aecm;
    if (aecm == NULL)
    {
        return -1;
    }

    WebRtcSpl_Init();

    if (WebRtcAecm_CreateCore(&aecm->aecmCore) == -1)
    {
        WebRtcAecm_Free(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->farendBuf = WebRtc_CreateBuffer(kBufSizeSamp,
                                          sizeof(int16_t));
    if (!aecm->farendBuf)
    {
        WebRtcAecm_Free(aecm);
        aecm = NULL;
        return -1;
    }

    aecm->initFlag = 0;
    aecm->lastError = 0;









#ifdef AEC_DEBUG

	aecm->aecmCore->farFile = NULL;
    aecm->aecmCore->farFile = fopen(FAR_FILE,"wb");
	aecm->aecmCore->farFile_block = fopen(FAR_BLOCK_FILE,"wb");
    aecm->aecmCore->nearFile = fopen(NEAR_FILE,"wb");
	aecm->aecmCore->nearFile_1 = fopen(NEAR_CLEAN_FILE, "wb");
    aecm->aecmCore->outFile = fopen(OUT_FILE,"wb");
	aecm->aecmCore->speakerModeFile = fopen(MODE_FILE,"wb");		

	 aecm->aecmCore->farFile_aligned = fopen(FAR_ALIGNED_FILE,"wb");
#endif // AEC_DEBUG

#ifdef MUSIC_OR_SPEECH
	aecm->aecmCore->nearIsSpeech=1;//DEFAUL_NEAR_SIGNAL_TYPE;
	aecm->aecmCore->farSmdInst=NULL;
	aecm->aecmCore->farSmdInst=smd_handle_create();
	aecm->aecmCore->nearSmdInst=NULL;
	aecm->aecmCore->nearSmdInst=smd_handle_create();
	aecm->aecmCore->smdBufLeft=0;
    memset(aecm->aecmCore->nearMusicProb,0,500*sizeof(float));
	memset(aecm->aecmCore->farMusicProb,0,500*sizeof(float));
	memset(aecm->aecmCore->residualBuf,0,PART_LEN*sizeof(int16_t));
#endif
    return 0;
}

int32_t WebRtcAecm_Free(void *aecmInst)
{
  AecMobile* aecm = aecmInst;

    if (aecm == NULL)
    {
        return -1;
    }
#ifdef SUPPORT_32K
	 WebRtcSpl_FreeRealFFT(aecm->real_fft_for32k);
	 WebRtc_FreeBuffer(aecm->nearNoisyFrameBufFor32k);
	 WebRtc_FreeBuffer(aecm->outFrameBufFor32k);
#endif
#ifdef AEC_DEBUG
#ifdef SUPPORT_32K
	if(aecm->farFile32)
	{
		fclose(aecm->farFile32);
		aecm->farFile32=NULL;
	}
	if(aecm->nearFile32)
	{
		fclose(aecm->nearFile32);
		aecm->nearFile32=NULL;
	}
#endif
	if(aecm->aecmCore->farFile)
	{
		fclose(aecm->aecmCore->farFile);   
	}
	if(aecm->aecmCore->farFile_block)
	{
		fclose(aecm->aecmCore->farFile_block);   
	}
	if(aecm->aecmCore->nearFile)
	{
		fclose(aecm->aecmCore->nearFile);
	}
	if(aecm->aecmCore->nearFile_1)
	{
		fclose(aecm->aecmCore->nearFile_1);
	}
	if(aecm->aecmCore->outFile)
	{
		 fclose(aecm->aecmCore->outFile); 
	}
	if(aecm->aecmCore->speakerModeFile)
	{
		fclose(aecm->aecmCore->speakerModeFile);
		aecm->aecmCore->speakerModeFile=NULL;
	}
	if(aecm->aecmCore->farFile_aligned)
	{
		fclose(aecm->aecmCore->farFile_aligned);   
	}
#endif // AEC_DEBUG

#ifdef MUSIC_OR_SPEECH
	aecm->aecmCore->nearIsSpeech=1;//DEFAUL_NEAR_SIGNAL_TYPE;	
	if(aecm->aecmCore->farSmdInst)
	{
		smd_handle_destory(aecm->aecmCore->farSmdInst);
		aecm->aecmCore->farSmdInst=NULL;
	}
	if(aecm->aecmCore->nearSmdInst)
	{
		smd_handle_destory(aecm->aecmCore->nearSmdInst);
		aecm->aecmCore->nearSmdInst=NULL;
	}	
#endif
    WebRtcAecm_FreeCore(aecm->aecmCore);
    WebRtc_FreeBuffer(aecm->farendBuf);

    free(aecm);

    return 0;
}

int32_t WebRtcAecm_Init(void *aecmInst, int32_t sampFreq)
{
  AecMobile* aecm = aecmInst;
    AecmConfig aecConfig;

    if (aecm == NULL)
    {
        return -1;
    }
#ifdef SUPPORT_32K
	aecm->inOutSampleRate=16000;
	if(32000==sampFreq)
	{
	    aecm->inOutSampleRate=32000;
		sampFreq =16000;
	}
	else
	{
		aecm->inOutSampleRate = sampFreq;
	}
	memset(aecm->resampleState,0,32*sizeof(int32_t));
	memset(aecm->resampleStateNear,0,32*sizeof(int32_t));
	memset(aecm->resampleStateOutUpBy2,0,32*sizeof(int32_t));
	memset(aecm->resampleStateOutUpBy2forResidual,0,32*sizeof(int32_t));
	memset(aecm->nearBufFor32k,0,256*sizeof(int16_t));
	memset(aecm->outBufFor32k,0,256*sizeof(int16_t));
	aecm->real_fft_for32k = WebRtcSpl_CreateRealFFT(PART_LEN_SHIFT+1);
	if (aecm->real_fft_for32k == NULL) {		
		return -1;
	}
	aecm->nearNoisyFrameBufFor32k = WebRtc_CreateBuffer(640,sizeof(int16_t));
	aecm->outFrameBufFor32k = WebRtc_CreateBuffer(640,sizeof(int16_t));
	WebRtc_InitBuffer(aecm->nearNoisyFrameBufFor32k);
	WebRtc_InitBuffer(aecm->outFrameBufFor32k);
	
#ifdef AEC_DEBUG
	aecm->farFile32 = NULL;
	aecm->farFile32=fopen(FAR_FILE_NAME_32,"wb");
	aecm->nearFile32 = NULL;
	aecm->nearFile32=fopen(NEAR_FILE_NAME_32,"wb");
#endif
#endif
    if (sampFreq != 8000 && sampFreq != 16000)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
    aecm->sampFreq = sampFreq;

    // Initialize AECM core
    if (WebRtcAecm_InitCore(aecm->aecmCore, aecm->sampFreq) == -1)
    {
        aecm->lastError = AECM_UNSPECIFIED_ERROR;
        return -1;
    }

    // Initialize farend buffer
    WebRtc_InitBuffer(aecm->farendBuf);

    aecm->initFlag = kInitCheck; // indicates that initialization has been done

    aecm->delayChange = 1;

    aecm->sum = 0;
    aecm->counter = 0;
    aecm->checkBuffSize = 1;
    aecm->firstVal = 0;

    aecm->ECstartup = 1;
    aecm->bufSizeStart = 0;
    aecm->checkBufSizeCtr = 0;
    aecm->filtDelay = 0;
    aecm->timeForDelayChange = 0;
    aecm->knownDelay = 0;
    aecm->lastDelayDiff = 0;

    memset(&aecm->farendOld[0][0], 0, 160);

    // Default settings.
    aecConfig.cngMode = AecmTrue;

    aecConfig.echoMode =  DEFAULT_ECHO_MODE;
	aecConfig.nlpLevel = 1;

	aecConfig.tailLenMs = 128;


    if (WebRtcAecm_set_config(aecm, aecConfig) == -1)
    {
        aecm->lastError = AECM_UNSPECIFIED_ERROR;
        return -1;
    }

    return 0;
}
int32_t WebRtcAecm_getEchoFarEnRatio(void *aecmInst)
{
	AecMobile* aecm = aecmInst;
	int32_t retVal = -ONE_Q14;	
	if (aecm == NULL)
	{
		return retVal;
	}	
	//return aecm->aecmCore->mseReach20Cnt;
	return aecm->aecmCore->echoFarEnRatio;
}
int32_t WebRtcAecm_getFarFrameDropCnt(void *aecmInst)
{
	AecMobile* aecm = aecmInst;
	int32_t retVal = -ONE_Q14;	
	if (aecm == NULL)
	{
		return retVal;
	}	
	retVal =  (int32_t) aecm->aecmCore->farFrameDropCnt;
	//retVal = WebRtc_available_read(aecm->farendBuf)/FRAME_LEN;
	//retVal = aecm->aecmCore->totCount;
	//retVal = aecm->aecmCore->mseStoredOld/20;
	/*if(aecm->aecmCore->backupFilterUpdateCnt>=30000)
	{
		aecm->aecmCore->backupFilterUpdateCnt=0;
	}
	retVal = (aecm->aecmCore->backupFilterUpdateCnt-(aecm->aecmCore->backupFilterUpdateCnt/100)*100)*100+(aecm->aecmCore->mseReach20Cnt-(aecm->aecmCore->mseReach20Cnt/100)*100);*/
	return retVal;
}
int32_t WebRtcAecm_getScaleFactor(void *aecmInst)
{
	AecMobile* aecm = aecmInst;
	int32_t retVal = 1024;
	const float maxScaleFactor = 896.0f;
	const float minScaleFactor=512.0f;
	const int16_t echoFarEnRatioLowThreshold = 512;
	if (aecm == NULL)
	{
		return retVal;
	}
	if(aecm->aecmCore->echoFarEnRatioUpdateFlag==ONE_Q14)
	{
		if ((aecm->aecmCore->echoFarEnRatio>echoFarEnRatioLowThreshold) && (aecm->aecmCore->farEndScaleFactor>minScaleFactor))
		{
			aecm->aecmCore->farEndScaleFactor = (aecm->aecmCore->farEndScaleFactor + minScaleFactor)/2;
			//aecm->aecmCore->farEndScaleFactor =aecm->aecmCore->farEndScaleFactor*0.95f;
		}
		else if ((aecm->aecmCore->echoFarEnRatio<echoFarEnRatioLowThreshold) && (aecm->aecmCore->farEndScaleFactor<maxScaleFactor))
		{
			aecm->aecmCore->farEndScaleFactor = aecm->aecmCore->farEndScaleFactor*1.1f;
		}
		aecm->aecmCore->echoFarEnRatioUpdateFlag=0;
	}
	if (aecm->aecmCore->farEndScaleFactor>maxScaleFactor)
	{
		aecm->aecmCore->farEndScaleFactor = maxScaleFactor;
	}
	else if (aecm->aecmCore->farEndScaleFactor<minScaleFactor)
	{
		aecm->aecmCore->farEndScaleFactor = minScaleFactor;
	}	
	if(aecm->aecmCore->AecmMode>DEFAULT_ECHO_MODE)
	{
		aecm->aecmCore->farEndScaleFactor=aecm->aecmCore->farEndScaleFactorSpeakerOn;
	}
	else
	{
		aecm->aecmCore->farEndScaleFactor=aecm->aecmCore->farEndScaleFactorSpeakerOff;
	}
	retVal = aecm->aecmCore->farEndScaleFactor;	
    return retVal;
}
int32_t WebRtcAecm_setPhoneModelName(void *aecmInst, char* name)
{
    AecMobile* aecm = aecmInst;		
	if ((aecm == NULL)||(name==NULL))
    {
        return -1;
    }
	
	if((!strcmp(name,"GN8002"))||(!strcmp(name,"GN8003")))
	{
		aecm->aecmCore->cutHighFrequencyEnable=1;
	}
	else
	{
		aecm->aecmCore->cutHighFrequencyEnable=0;
	}
	if(!strcmp(name,"R7Plus"))
	{
		aecm->aecmCore->farEndScaleFactorSpeakerOn=512;
	}
	else
	{
		aecm->aecmCore->farEndScaleFactorSpeakerOn=896;
	}
	return 0;
}
int32_t WebRtcAecm_BufferFarend(void *aecmInst, const int16_t *farend,
                                int16_t nrOfSamples)
{
  AecMobile* aecm = aecmInst;
    int32_t retVal = 0;	
    if (aecm == NULL)
    {
        return -1;
    }
#ifdef SUPPORT_32K
	if(32000==aecm->inOutSampleRate && 320==nrOfSamples)
	{		
		int32_t i,samVal;
#ifdef AEC_DEBUG
		if(aecm->farFile32 != NULL)
		{
			fwrite(farend,sizeof(int16_t),320,aecm->farFile32);
		}	
#endif
		WebRtcSpl_DownBy2ShortToInt(farend,320,aecm->resampleOut,aecm->resampleState);
		for(i=0;i<160;i++)
		{
			samVal = aecm->resampleOut[i]>>15;
			if(samVal>32767)
			{
				samVal=32767;
			}
			else if(samVal<-32768)
			{
				samVal=-32768;
			}
			aecm->resampleOutShort[i]=(int16_t)samVal;
		}
		retVal=WebRtcAecm_BufferFarend(aecmInst, (const int16_t*)aecm->resampleOutShort,160);      
		return retVal;
	}
#endif

    if (farend == NULL)
    {
        aecm->lastError = AECM_NULL_POINTER_ERROR;
        return -1;
    }

    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }

    if (nrOfSamples != 80 && nrOfSamples != 160)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
	aecm->aecmCore->farBufferCnt++;
    // TODO: Is this really a good idea?
#ifndef CLOSE_DELAY_COMPARE
    if (!aecm->ECstartup)
    {
       WebRtcAecm_DelayComp(aecm);
    }
#endif

	if (WebRtc_available_read(aecm->farendBuf) >= (kBufSizeSamp - nrOfSamples))//chgx added 20160415:if the far_end is almost full��
	{
		WebRtc_MoveReadPtr(aecm->farendBuf, nrOfSamples);//chgx added 20160415:drop the earlyest far signal
		if(aecm->aecmCore->totCount>0)
		{
			aecm->aecmCore->farFrameDropCnt++;
		}		
	}

#ifndef PC_DEMO_VERSION
	{
		int16_t i;		
		int16_t farTmp[160];
		float factor=aecm->aecmCore->farEndScaleFactor/1024.0f;
		if (nrOfSamples > 160)
		{
			nrOfSamples = 160;
		}
		for(i=0;i<nrOfSamples;i++)
		{
			farTmp[i] = farend[i] * factor;
		}
		WebRtc_WriteBuffer(aecm->farendBuf, farTmp, (size_t)nrOfSamples);		
	}
#else
	WebRtc_WriteBuffer(aecm->farendBuf, farend, (size_t)nrOfSamples);
#endif
#ifdef AEC_DEBUG

	if ((farend!=NULL)&&(aecm->aecmCore->farFile != NULL))
    {
		fwrite(farend, sizeof(int16_t), nrOfSamples, aecm->aecmCore->farFile);	  
	}
#endif

	 
   // WebRtc_WriteBuffer(aecm->farendBuf, farend, (size_t) nrOfSamples);

#ifdef FAR_END_BUF_LEVEL_WRITE
	if(aecm->aecmCore->totalRenderCnt<32768-512)
	{
	 aecm->aecmCore->totalRenderCnt+=512;
	}
	else
	{
		aecm->aecmCore->totalRenderCnt=0;
	}
	{
		short i;
		for(i=0;i<nrOfSamples;i++)
		{
			fwrite(&aecm->aecmCore->totalRenderCnt, sizeof(int16_t), 1, aecm->aecmCore->farEndBufLevelFile);
		}
		  
	 }
#endif
    return retVal;
}
int32_t WebRtcAecm_GetEchoState(void *aecmInst)
{
	AecMobile* aecm = aecmInst;
	return aecm->aecmCore->stream_has_echo;
}
#ifdef SUPPORT_32K
// Square root of Hanning window in Q14.
//#if defined(WEBRTC_DETECT_ARM_NEON) || defined(WEBRTC_ARCH_ARM_NEON)
// Table is defined in an ARM assembly file.
//extern const ALIGN8_BEG int16_t WebRtcAecm_kSqrtHanning256[] ALIGN8_END;
//#else
static const ALIGN8_BEG int16_t WebRtcAecm_kSqrtHanning256[] ALIGN8_END = {	
	0,           202,         404 ,        605    ,     807   ,     1009      ,  1210   ,     1411,
	1612 ,       1813 ,       2013   ,     2214  ,      2413  ,      2613 ,       2812 ,       3011,
	3209  ,      3406 ,       3604 ,       3800 ,       3996   ,     4192   ,     4387   ,     4581,
	4774 ,       4967 ,       5159,        5350,        5540 ,       5730,        5919,        6106,
	6293,        6479,        6664,        6848,        7031,        7212,        7393,        7573,
	7751,        7928,        8104,        8279,        8453,        8625,        8796,        8966,
	9134,        9301,        9466,        9630,        9793,        9954,       10113,       10272,
	10428,       10583,       10736,       10888,       11038,       11186,       11333,       11478,
	11621,       11762,       11902,       12040,       12176,       12310,       12442,       12572,
	12701,       12827,       12952,       13075,       13195,       13314,       13431,       13545,
    13658,       13768,       13877,       13983,       14087,       14189,       14289,       14386,
	14482,       14575,       14666,       14755,       14842,       14926,       15008,       15088,
	15166,       15241,       15314,       15384,       15453,       15519,       15582,       15643,
	15702,       15759,       15813,       15864,       15913,       15960,       16005,       16047,
	16086,       16123,       16158,       16190,       16220,       16247,       16272,       16294,
	16314,       16331,       16346,       16359,       16369,       16376,       16381,       16384
};
//#endif
static void WindowAndFFT256(AecMobile* aecm,int16_t* fft,const int16_t* time_signal,ComplexInt16* freq_signal,int time_signal_scaling) 
{
		int i = 0;
		int len=PART_LEN*2;
		// FFT of signal
		for (i = 0; i <len ; i++) {
			// Window time domain signal and insert into real part of
			// transformation array |fft|
			int16_t scaled_time_signal = time_signal[i] << time_signal_scaling;
			fft[i] = (int16_t)((scaled_time_signal * WebRtcAecm_kSqrtHanning256[i]) >> 14);
			scaled_time_signal = time_signal[i + len] << time_signal_scaling;
			fft[len + i] = (int16_t)((
				scaled_time_signal * WebRtcAecm_kSqrtHanning256[len - i]) >> 14);
		}

		// Do forward FFT, then take only the first PART_LEN complex samples,
		// and change signs of the imaginary parts.
		WebRtcSpl_RealForwardFFT(aecm->real_fft_for32k, fft, (int16_t*)freq_signal);
		for (i = 0; i < len; i++) {
			freq_signal[i].imag = -freq_signal[i].imag;
		}	
		freq_signal[0].imag = 0;
		freq_signal[len].imag = 0;
}
static void InverseFFTAndWindow256(AecMobile* aecm,	int16_t* fft,	ComplexInt16* efw,	int16_t* output) 
{
		int i, j, outCFFT;
		int32_t tmp32no1;
		// Reuse |efw| for the inverse FFT output after transferring
		// the contents to |fft|.
		int16_t* ifft_out = (int16_t*)efw;
		int16_t len=2*PART_LEN;
		int16_t len2 = 2*len;

		// Synthesis
		for (i = 1, j = 2; i < len; i += 1, j += 2) {
			fft[j] = efw[i].real;
			fft[j + 1] = -efw[i].imag;
		}
		fft[0] = efw[0].real;
		fft[1] = -efw[0].imag;

		fft[len2] = efw[len].real;
		fft[len2 + 1] = -efw[len].imag;

		// Inverse FFT. Keep outCFFT to scale the samples in the next block.
		outCFFT = WebRtcSpl_RealInverseFFT(aecm->real_fft_for32k, fft, ifft_out);
		for (i = 0; i < len; i++) {
			ifft_out[i] = (int16_t)WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(ifft_out[i], WebRtcAecm_kSqrtHanning256[i], 14);
			tmp32no1 = WEBRTC_SPL_SHIFT_W32((int32_t)ifft_out[i],outCFFT );
			output[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,tmp32no1 + aecm->outBufFor32k[i],WEBRTC_SPL_WORD16_MIN);
			tmp32no1 = (ifft_out[len + i] *WebRtcAecm_kSqrtHanning256[len - i]) >> 14;
			tmp32no1 = WEBRTC_SPL_SHIFT_W32(tmp32no1,outCFFT);
			aecm->outBufFor32k[i] = (int16_t)WEBRTC_SPL_SAT(WEBRTC_SPL_WORD16_MAX,tmp32no1,WEBRTC_SPL_WORD16_MIN);
		}

	
}
//WebRtcSpl_RealInverseFFT(aecm->real_fft, fft, ifft_out)
#endif
int32_t WebRtcAecm_Process(void *aecmInst, const int16_t *nearendNoisy,
                           const int16_t *nearendClean, int16_t *out,
                           int16_t nrOfSamples, int16_t msInSndCardBuf)
{
  AecMobile* aecm = aecmInst;
    int32_t retVal = 0;
    short i;
    short nmbrOfFilledBuffers;
    short nBlocks10ms;
    short nFrames;

    if (aecm == NULL)
    {
        return -1;
    }

#ifdef SUPPORT_32K	
	if((aecm->inOutSampleRate==32000)&&(nrOfSamples==320))
	{
		int32_t i,samVal;		
		int32_t reVal32=-1;
		int16_t outTmp[160];		
		//step1:down sample input to 16k
		if(nearendClean)
		{
#ifdef AEC_DEBUG
			if(aecm->nearFile32 != NULL)
			{
				fwrite(nearendClean,sizeof(int16_t),320,aecm->nearFile32);
			}	
#endif
			WebRtcSpl_DownBy2ShortToInt(nearendClean,320,aecm->resampleOutNear,aecm->resampleStateNear);

		}
		else
		{
#ifdef AEC_DEBUG
			if(aecm->nearFile32 != NULL)
			{
				fwrite(nearendNoisy,sizeof(int16_t),320,aecm->nearFile32);
			}	
#endif
			WebRtcSpl_DownBy2ShortToInt(nearendNoisy,320,aecm->resampleOutNear,aecm->resampleStateNear);

		}
		for(i=0;i<160;i++)
		{
			samVal = aecm->resampleOutNear[i]>>15;
			if(samVal>32767)
			{
				samVal=32767;
			}
			else if(samVal<-32768)
			{
				samVal=-32768;
			}
			aecm->resampleOutShortNear[i]=(int16_t)samVal;
			aecm->resampleOutShortNearClean[i]=(int16_t)samVal;						
		}		
		
		//step2 :call 16k process function
		aecm->aecmCore->processBlockCnt=0;
		
		WebRtc_WriteBuffer(aecm->nearNoisyFrameBufFor32k, nearendNoisy, 320);
		reVal32 = WebRtcAecm_Process(aecmInst, aecm->resampleOutShortNear,aecm->resampleOutShortNearClean, outTmp,160,msInSndCardBuf);
		{
			int16_t* nearBlockDataPtr;
			int16_t nearBlockData32k[PART_LEN*2];
			ComplexInt16 freqData[256];
			int16_t fft[512];
			int16_t size;
			int16_t*out_ptr;
			int16_t outBlock[128];
			for (i=0;i<aecm->aecmCore->processBlockCnt;i++)
			{
				int m;
				WebRtc_ReadBuffer(aecm->nearNoisyFrameBufFor32k, (void**) &nearBlockDataPtr, nearBlockData32k,PART_LEN*2);
				memmove(&aecm->nearBufFor32k[0],&aecm->nearBufFor32k[128],PART_LEN*2*sizeof(int16_t));
				memmove(&aecm->nearBufFor32k[128],nearBlockDataPtr,PART_LEN*2*sizeof(int16_t));
				WindowAndFFT256(aecm,fft,aecm->nearBufFor32k,freqData,0);
				for(m=0;m<PART_LEN1;m++)
				{
					freqData[m].real=(int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(freqData[m].real,aecm->aecmCore->HnlInLowBand16k[i][m],14));
					freqData[m].imag=(int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(freqData[m].imag,aecm->aecmCore->HnlInLowBand16k[i][m],14));
				}
				for(m=PART_LEN1;m<PART_LEN1+PART_LEN;m++)
				{
					freqData[m].real=(int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(freqData[m].real,aecm->aecmCore->avgHnlInLowBand16k[i],14));
					freqData[m].imag=(int16_t)(WEBRTC_SPL_MUL_16_16_RSFT_WITH_ROUND(freqData[m].imag,aecm->aecmCore->avgHnlInLowBand16k[i],14));
				}
				InverseFFTAndWindow256(aecm,fft,freqData,outBlock);
				 WebRtc_WriteBuffer(aecm->outFrameBufFor32k, outBlock, PART_LEN*2);
			}
			aecm->aecmCore->processBlockCnt=0;
			size = (int) WebRtc_available_read(aecm->outFrameBufFor32k);
			if (size < 320)
			{
				WebRtc_MoveReadPtr(aecm->outFrameBufFor32k, size - 320);
			}

			// Obtain an output frame.
			WebRtc_ReadBuffer(aecm->outFrameBufFor32k, (void**) &out_ptr, out, 320);
			if (out_ptr != out) {
				// ReadBuffer() hasn't copied to |out| in this case.
				memcpy(out, out_ptr, 320 * sizeof(int16_t));
			}
		}
		//WebRtc_WriteBuffer(aecm->outFrameBufFor32k, farFrame, FRAME_LEN);
		/*
		if(reVal32==0)
		{
			int32_t tmp,m;
			m=0;
			for(i=0;i<318;i+=2)
			{						
				tmp =outTmp[m];
				out[i]=tmp;
				tmp +=outTmp[m+1];
				out[i+1]=tmp>>1;
				m++;
			}
			out[318]=outTmp[159];
			out[319]=outTmp[159];				
		}*/
		return reVal32;
	}	
#endif
    if (nearendNoisy == NULL)
    {
        aecm->lastError = AECM_NULL_POINTER_ERROR;
        return -1;
    }

    if (out == NULL)
    {
        aecm->lastError = AECM_NULL_POINTER_ERROR;
        return -1;
    }

    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }


    if (nrOfSamples != 80 && nrOfSamples != 160)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }

    if (msInSndCardBuf < 0)
    {
        msInSndCardBuf = 0;
        aecm->lastError = AECM_BAD_PARAMETER_WARNING;
        retVal = -1;
    } else if (msInSndCardBuf > 500)
    {
        msInSndCardBuf = 500;
        aecm->lastError = AECM_BAD_PARAMETER_WARNING;
        retVal = -1;
    }
    msInSndCardBuf += 10;
    aecm->msInSndCardBuf = msInSndCardBuf;
	aecm->aecmCore->nearProcessCnt++;
    nFrames = nrOfSamples / FRAME_LEN;
    nBlocks10ms = nFrames / aecm->aecmCore->mult;


    if (aecm->ECstartup)
    {
        if (nearendClean == NULL)
        {
            if (out != nearendNoisy)
            {
                memcpy(out, nearendNoisy, sizeof(short) * nrOfSamples);
            }
        } else if (out != nearendClean)
        {
            memcpy(out, nearendClean, sizeof(short) * nrOfSamples);
        }

        nmbrOfFilledBuffers =
            (short) WebRtc_available_read(aecm->farendBuf) / FRAME_LEN;
        // The AECM is in the start up mode
        // AECM is disabled until the soundcard buffer and farend buffers are OK

        // Mechanism to ensure that the soundcard buffer is reasonably stable.
        if (aecm->checkBuffSize)
        {
            aecm->checkBufSizeCtr++;
            // Before we fill up the far end buffer we require the amount of data on the
            // sound card to be stable (+/-8 ms) compared to the first value. This
            // comparison is made during the following 4 consecutive frames. If it seems
            // to be stable then we start to fill up the far end buffer.

            if (aecm->counter == 0)
            {
                aecm->firstVal = aecm->msInSndCardBuf;
                aecm->sum = 0;
            }

            if (abs(aecm->firstVal - aecm->msInSndCardBuf)
                    < WEBRTC_SPL_MAX(0.2 * aecm->msInSndCardBuf, kSampMsNb))
            {
                aecm->sum += aecm->msInSndCardBuf;
                aecm->counter++;
            } else
            {
                aecm->counter = 0;
            }

            if (aecm->counter * nBlocks10ms >= 6)
            {
                // The farend buffer size is determined in blocks of 80 samples
                // Use 75% of the average value of the soundcard buffer
                aecm->bufSizeStart
                        = WEBRTC_SPL_MIN((3 * aecm->sum
                                        * aecm->aecmCore->mult) / (aecm->counter * 40), BUF_SIZE_FRAMES);
				if(aecm->bufSizeStart<aecm->aecmCore->mult)
				{
					aecm->bufSizeStart=aecm->aecmCore->mult;
				}
                // buffersize has now been determined
                aecm->checkBuffSize = 0;
            }

            if (aecm->checkBufSizeCtr * nBlocks10ms > 50)
            {
                // for really bad sound cards, don't disable echocanceller for more than 0.5 sec
                aecm->bufSizeStart = WEBRTC_SPL_MIN((3 * aecm->msInSndCardBuf
                                * aecm->aecmCore->mult) / 40, BUF_SIZE_FRAMES);
                aecm->checkBuffSize = 0;
            }
        }

        // if checkBuffSize changed in the if-statement above
        if (!aecm->checkBuffSize)
        {
            // soundcard buffer is now reasonably stable
            // When the far end buffer is filled with approximately the same amount of
            // data as the amount on the sound card we end the start up phase and start
            // to cancel echoes.
		//	aecm->bufSizeStart=nmbrOfFilledBuffers;//chgx ?a：∴???：??：∴??|━??：o?3：?D：＜?┷?2??：1：a?2?|━?：o|━?：oAPK?D
            if (nmbrOfFilledBuffers == aecm->bufSizeStart)
            {
                aecm->ECstartup = 0; // Enable the AECM
				 WebRtc_MoveReadPtr(aecm->farendBuf,                        //219
                                   (int) WebRtc_available_read(aecm->farendBuf)
								   - (int) aecm->aecmCore->mult * FRAME_LEN);
				
            } else if (nmbrOfFilledBuffers > aecm->bufSizeStart)
            {
                WebRtc_MoveReadPtr(aecm->farendBuf,
                                   (int) WebRtc_available_read(aecm->farendBuf)
                                   - (int) aecm->bufSizeStart * FRAME_LEN);
                aecm->ECstartup = 0;
            }
			else 
			{
				WebRtc_MoveReadPtr(aecm->farendBuf,
					(int)WebRtc_available_read(aecm->farendBuf)
					- (int)aecm->aecmCore->mult * FRAME_LEN);
				aecm->ECstartup = 0;
			}
        }

    } else
    {
        // AECM is enabled

        // Note only 1 block supported for nb and 2 blocks for wb
        for (i = 0; i < nFrames; i++)
        {
            int16_t farend[FRAME_LEN];
            const int16_t* farend_ptr = NULL;

            nmbrOfFilledBuffers =
                (short) WebRtc_available_read(aecm->farendBuf) / FRAME_LEN;

            // Check that there is data in the far end buffer
            if (nmbrOfFilledBuffers > 0)
            {
				if (nmbrOfFilledBuffers > 10 * aecm->aecmCore->mult)
				{
					// if we find that there are too many samples in farendBuf,we should skip some samples.
					WebRtc_MoveReadPtr(aecm->farendBuf, FRAME_LEN*((nmbrOfFilledBuffers - 10 * aecm->aecmCore->mult)/4));//20170221
				}
                // Get the next 80 samples from the farend buffer
                WebRtc_ReadBuffer(aecm->farendBuf, (void**) &farend_ptr, farend,
                                  FRAME_LEN);

                // Always store the last frame for use when we run out of data
                memcpy(&(aecm->farendOld[i][0]), farend_ptr,
                       FRAME_LEN * sizeof(short));
#ifdef FAR_FILE_REAL_WRITE
				 
				if ((farend_ptr!=NULL)&&(aecm->aecmCore->farFile_real != NULL))
				{
					 fwrite(farend_ptr, sizeof(short), FRAME_LEN, aecm->aecmCore->farFile_real);
				}
#endif
#ifdef FAR_FILE_FILL_WRITE
				{ 
					int16_t write_value=0;
					int16_t i;					
					if (aecm->aecmCore->farFile_fill != NULL)
					{
						for(i=0;i<FRAME_LEN;i++)
						{
							fwrite(&write_value, sizeof(int16_t), 1, aecm->aecmCore->farFile_fill);
						}
					}
				}
#endif
            }
			else
            {
                // We have no data so we use the last played frame
                memcpy(farend, &(aecm->farendOld[i][0]), FRAME_LEN * sizeof(short));
                farend_ptr = farend;
            }

            // Call buffer delay estimator when all data is extracted,
            // i,e. i = 0 for NB and i = 1 for WB
#ifndef CLOSE_EST_BUF_DELAY
            if ((i == 0 && aecm->sampFreq == 8000) || (i == 1 && aecm->sampFreq == 16000))
            {
                WebRtcAecm_EstBufDelay(aecm, aecm->msInSndCardBuf);//219 test only
            }
#endif
			
            // Call the AECM
            /*WebRtcAecm_ProcessFrame(aecm->aecmCore, farend, &nearend[FRAME_LEN * i],
             &out[FRAME_LEN * i], aecm->knownDelay);*/

            if (WebRtcAecm_ProcessFrame(aecm->aecmCore,
                                        farend_ptr,
                                        &nearendNoisy[FRAME_LEN * i],
                                        (nearendClean
                                         ? &nearendClean[FRAME_LEN * i]
                                         : NULL),
                                        &out[FRAME_LEN * i]) == -1)
                return -1;
        }
    }


 
    return retVal;
}
int32_t WebRtcAecm_enable_afc(void* aecmInst, int16_t enableAfc)//added by chgx 20160220
{
	AecMobile* aecm = (AecMobile*)aecmInst;
	if (aecm)
	{
		aecm->aecmCore->afcEnable = enableAfc;
	}
	return 0;
}
int32_t WebRtcAecm_set_config(void *aecmInst, AecmConfig config)
{
  AecMobile* aecm = (AecMobile*)aecmInst;
  float nlpLevelCoef[5]={0.75f,1.0f,1.25f,1.5f,2.5f};

    if (aecm == NULL)
    {
        return -1;
    }

    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }

    if (config.cngMode != AecmFalse && config.cngMode != AecmTrue)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
    aecm->aecmCore->cngMode = config.cngMode;

    if (config.echoMode < 0 || config.echoMode > 4)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }

	aecm->aecmCore->AecmMode = config.echoMode;

	if (config.tailLenMs <32 || config.tailLenMs >256)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
	aecm->aecmCore->tailLenMs = config.tailLenMs;

	if (config.nlpLevel < 0 || config.nlpLevel > 4)
    {
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
	else
    {
	    aecm->aecmCore->nlpLevel = config.nlpLevel;
	    aecm->aecmCore->supGain = SUPGAIN_DEFAULT *nlpLevelCoef[aecm->aecmCore->nlpLevel];
		aecm->aecmCore->supGainOld = SUPGAIN_DEFAULT *nlpLevelCoef[aecm->aecmCore->nlpLevel];
		aecm->aecmCore->supGainErrParamA = SUPGAIN_ERROR_PARAM_A *nlpLevelCoef[aecm->aecmCore->nlpLevel];
		aecm->aecmCore->supGainErrParamD = SUPGAIN_ERROR_PARAM_D *nlpLevelCoef[aecm->aecmCore->nlpLevel];
		aecm->aecmCore->supGainErrParamDiffAB = (SUPGAIN_ERROR_PARAM_A - SUPGAIN_ERROR_PARAM_B)*nlpLevelCoef[aecm->aecmCore->nlpLevel];
		aecm->aecmCore->supGainErrParamDiffBD = (SUPGAIN_ERROR_PARAM_B - SUPGAIN_ERROR_PARAM_D )*nlpLevelCoef[aecm->aecmCore->nlpLevel];
    }  
    
    return 0;
}

int32_t WebRtcAecm_get_config(void *aecmInst, AecmConfig *config)
{
  AecMobile* aecm = aecmInst;

    if (aecm == NULL)
    {
        return -1;
    }

    if (config == NULL)
    {
        aecm->lastError = AECM_NULL_POINTER_ERROR;
        return -1;
    }

    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }

    config->cngMode = aecm->aecmCore->cngMode;    
	config->echoMode=aecm->aecmCore->AecmMode;
	config->nlpLevel=aecm->aecmCore->nlpLevel;
	config->tailLenMs=aecm->aecmCore->tailLenMs;
    return 0;
}

int32_t WebRtcAecm_InitEchoPath(void* aecmInst,
                                const void* echo_path,
                                size_t size_bytes)
{
  AecMobile* aecm = aecmInst;
    const int16_t* echo_path_ptr = echo_path;

    if (aecmInst == NULL) {
      return -1;
    }
    if (echo_path == NULL) {
      aecm->lastError = AECM_NULL_POINTER_ERROR;
      return -1;
    }
    if (size_bytes != WebRtcAecm_echo_path_size_bytes())
    {
        // Input channel size does not match the size of AECM
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }

    WebRtcAecm_InitEchoPathCore(aecm->aecmCore, echo_path_ptr);

    return 0;
}

int32_t WebRtcAecm_GetEchoPath(void* aecmInst,
                               void* echo_path,
                               size_t size_bytes)
{
  AecMobile* aecm = aecmInst;
    int16_t* echo_path_ptr = echo_path;

    if (aecmInst == NULL) {
      return -1;
    }
    if (echo_path == NULL) {
      aecm->lastError = AECM_NULL_POINTER_ERROR;
      return -1;
    }
    if (size_bytes != WebRtcAecm_echo_path_size_bytes())
    {
        // Input channel size does not match the size of AECM
        aecm->lastError = AECM_BAD_PARAMETER_ERROR;
        return -1;
    }
    if (aecm->initFlag != kInitCheck)
    {
        aecm->lastError = AECM_UNINITIALIZED_ERROR;
        return -1;
    }

    memcpy(echo_path_ptr, aecm->aecmCore->channelStored, size_bytes);
    return 0;
}

size_t WebRtcAecm_echo_path_size_bytes()
{
    return (PART_LEN1 * sizeof(int16_t));
}

int32_t WebRtcAecm_get_error_code(void *aecmInst)
{
  AecMobile* aecm = aecmInst;

    if (aecm == NULL)
    {
        return -1;
    }

    return aecm->lastError;
}
#ifndef CLOSE_EST_BUF_DELAY
static int WebRtcAecm_EstBufDelay(AecMobile* aecm, short msInSndCardBuf) {
    short delayNew, nSampSndCard;
    short nSampFar = (short) WebRtc_available_read(aecm->farendBuf);
    short diff;

    nSampSndCard = msInSndCardBuf * kSampMsNb * aecm->aecmCore->mult;

    delayNew = nSampSndCard - nSampFar;

    if (delayNew < FRAME_LEN)
    {
        WebRtc_MoveReadPtr(aecm->farendBuf, FRAME_LEN);
        delayNew += FRAME_LEN;
    }

    aecm->filtDelay = WEBRTC_SPL_MAX(0, (8 * aecm->filtDelay + 2 * delayNew) / 10);

    diff = aecm->filtDelay - aecm->knownDelay;
    if (diff > 224)
    {
        if (aecm->lastDelayDiff < 96)
        {
            aecm->timeForDelayChange = 0;
        } else
        {
            aecm->timeForDelayChange++;
        }
    } else if (diff < 96 && aecm->knownDelay > 0)
    {
        if (aecm->lastDelayDiff > 224)
        {
            aecm->timeForDelayChange = 0;
        } else
        {
            aecm->timeForDelayChange++;
        }
    } else
    {
        aecm->timeForDelayChange = 0;
    }
    aecm->lastDelayDiff = diff;

    if (aecm->timeForDelayChange > 25)
    {
        aecm->knownDelay = WEBRTC_SPL_MAX((int)aecm->filtDelay - 160, 0);
    }
    return 0;
}
#endif
#ifndef CLOSE_DELAY_COMPARE
static int WebRtcAecm_DelayComp(AecMobile* aecm) {
    int nSampFar = (int) WebRtc_available_read(aecm->farendBuf);
    int nSampSndCard, delayNew, nSampAdd;
    const int maxStuffSamp = 10 * FRAME_LEN;	
    nSampSndCard = aecm->msInSndCardBuf * kSampMsNb * aecm->aecmCore->mult;
    delayNew = nSampSndCard - nSampFar;

    if (delayNew > FAR_BUF_LEN - FRAME_LEN * aecm->aecmCore->mult)
    {
        // The difference of the buffer sizes is larger than the maximum
        // allowed known delay. Compensate by stuffing the buffer.
        nSampAdd = (int)(WEBRTC_SPL_MAX(((nSampSndCard >> 1) - nSampFar),
                FRAME_LEN));
        nSampAdd = WEBRTC_SPL_MIN(nSampAdd, maxStuffSamp);

       WebRtc_MoveReadPtr(aecm->farendBuf, -nSampAdd);
        aecm->delayChange = 1; // the delay needs to be updated
    }

    return 0;
}
#endif
