#include "smd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <memory.h>
#include <assert.h>
#include "typedefs.h"
#include "typedef_fx.h"

#include "analysis.h"
#include "signal_processing_library.h"
//#define SMD_DEBUG
#ifdef SMD_DEBUG
#include<stdio.h>
#endif
const int kDefaultSampleRateHz = 48000;
const int kDefaultFrameRateHz = 50;
const int kDefaultFrameSizeSamples = 960;//kDefaultSampleRateHz / kDefaultFrameRateHz;
const float kDefaultThreshold = 0.5f;
const int channels = 1;

//#define SMD_RECOGNIZE_FRAME_SIZE	600		//6s
typedef struct {	
	short pcmData20Ms[960];
	int srcMemTmp[480];
	CELTMode *celt_mode;					//模型参数
	WebRtcSpl_State8khzTo48khz state;		//重采样状态
	WebRtcSpl_State16khzTo48khz state16To48;
	TonalityAnalysisState mAnalysis_state;
	float musicProb;
	int dataLeft;
#ifdef SMD_DEBUG
	FILE*fpSrcIn;
	FILE*fpSrcOut;
#endif
} smd_inst_t;

float smd_get_music_prob(smd_inst_t *inst, short *sm_data)
{	
	AnalysisInfo analysis_info = {0};
	TonalityAnalysisState* analysis_state=&inst->mAnalysis_state;

	if (!inst || !sm_data) {
		return 0.0f;
	}
	//memset(&inst->mAnalysis_state, 0, sizeof (inst->mAnalysis_state));
	//判别音乐和说话声
	run_analysis(analysis_state,
		inst->celt_mode,
		sm_data,
		kDefaultFrameSizeSamples,
		kDefaultFrameSizeSamples,
		0,   
		-2,  
		channels,
		kDefaultSampleRateHz,
		16,                       
		downmix_int,
		&analysis_info);
	return analysis_info.music_prob;	
}


smd_h  smd_handle_create(void)
{
	smd_inst_t *inst = NULL;		
	

	inst = (smd_inst_t *)malloc(sizeof (*inst));
	if (!inst) {
		return SMD_INVALID_H;
	}
	memset(inst, 0, sizeof (*inst));

	//for resample
	WebRtcSpl_ResetResample8khzTo48khz(&inst->state);
	WebRtcSpl_ResetResample16khzTo48khz(&inst->state16To48);
	inst->celt_mode = opus_custom_mode_create(kDefaultSampleRateHz,
			kDefaultFrameSizeSamples, NULL);
	memset(&inst->mAnalysis_state, 0, sizeof (inst->mAnalysis_state));
#ifdef SMD_DEBUG
	inst->fpSrcIn=fopen("smdSrcIn.pcm","wb");
	inst->fpSrcOut=fopen("smdSrcOut.pcm","wb");
#endif
	return (smd_h)inst;
}

float  smd_handle_recv(smd_h handle, short *data, int sampleNum,int sampleRate)
{	
	smd_inst_t *inst = NULL;	
	inst = (smd_inst_t *)handle;
	if(handle==SMD_INVALID_H)
	{
		return 0.0f;
	}
	if(sampleNum*100!=sampleRate)
	{
		sampleRate = 8000;
		sampleNum = 80;
	}
#ifdef SMD_DEBUG
		if(inst->fpSrcIn)
		{
			fwrite(data,sizeof(short),sampleNum,inst->fpSrcIn);
		}
#endif
	inst->dataLeft+=10;
	if(8000==sampleRate)
	{
		WebRtcSpl_Resample8khzTo48khz(data, &inst->pcmData20Ms[480], &inst->state, inst->srcMemTmp);
	}
	else if(16000==sampleRate)
	{
		 WebRtcSpl_Resample16khzTo48khz(data, &inst->pcmData20Ms[480], &inst->state16To48, inst->srcMemTmp);
	}
	else if(48000==sampleRate)
	{
		memcpy (&inst->pcmData20Ms[480],data, sampleNum*sizeof(short));
	}
	while(inst->dataLeft>=20)
	{
		inst->musicProb=smd_get_music_prob(inst, inst->pcmData20Ms);
		inst->dataLeft-=20;
#ifdef SMD_DEBUG
		if(inst->fpSrcOut)
		{
			fwrite(inst->pcmData20Ms,sizeof(short),480*2,inst->fpSrcOut);
		}		
#endif
	}
	if(inst->dataLeft<0)
	{
		inst->dataLeft=0;
	}
	memmove(&inst->pcmData20Ms[0],&inst->pcmData20Ms[480],480*sizeof(short));
	return 	inst->musicProb;
}

float* smd_get_band_tonality(smd_h handle)
{
	smd_inst_t *inst = NULL;	
	inst = (smd_inst_t *)handle;
	if(handle==SMD_INVALID_H)
	{
		return NULL;
	}
	else
	{
		return &inst->mAnalysis_state.band_tonality[0];
	}
}

void  smd_handle_destory(smd_h handle)
{
	smd_inst_t *inst = NULL;

	if (SMD_INVALID_H == handle) {
		return;
	}
	inst = (smd_inst_t *)handle;
#ifdef SMD_DEBUG
		if(inst->fpSrcIn)
		{
			fclose(inst->fpSrcIn);
			inst->fpSrcIn=NULL;
		}
		if(inst->fpSrcOut)
		{
			fclose(inst->fpSrcOut);
			inst->fpSrcOut=NULL;
		}
#endif
	free(inst);
}

/*#define NN 160
#define SAMPLE_RATE NN*100
int main()
{	
	FILE *f = NULL;
	FILE *fmode = NULL;
	float musicProb;
	int musicProbFixedValue;	
	int frameCnt=0;
	short data[NN] = {0};
	smd_h handle = SMD_INVALID_H;
	

	handle = smd_handle_create();
	if (SMD_INVALID_H == handle) {
		printf("smd_handle_create is failed.");
		return -1;
	}

	if(NN==480)
	{
		f = fopen("E://record//smd//48k//today48mono.pcm","rb");
		fmode = fopen("musicProbQ14_48k.pcm","wb");
	}
	else if(NN==160)
	{
		//aecNear1
		f = fopen("E://record//smd//16k//aecNear1.pcm","rb");//greatestLove16mono today16mono.pcm  haibin16.wav  luohuasheng16.wav  out-aecm64.pcm  yuyandemeili16.wav  out-aecm64_music
		fmode = fopen("musicProbQ14_16k.pcm","wb");
	}
	else if(NN==80)
	{
		f = fopen("E://record//smd//8k//out-aecm64.pcm","rb");//today16mono.pcm  haibin16.wav  luohuasheng16.wav  out-aecm64.pcm  yuyandemeili16.wav  out-aecm64_music
		fmode = fopen("musicProbQ14_8k.pcm","wb");
	}	
	
	if (!f) {
		printf("can not open pcm file");
		return -1;
	}

	printf("\n --- Process Begin ---\n");

	while (NN== fread(data, sizeof(short), NN, f)) 
	{
		musicProb= smd_handle_recv(handle, data, NN,SAMPLE_RATE);
		musicProbFixedValue = (musicProb)*16384;		
		printf("frame%d,music_prob=%f\n",frameCnt,musicProb);
		frameCnt++;
		if(fmode!=NULL)
		{
			int j;
			for(j=0;j<NN;j++)
			{
			  fwrite(&musicProbFixedValue,sizeof(short),1,fmode);
			}
		}	
	}

	fclose(f);
	if(fmode!=NULL)
	{
		fclose(fmode);
	}
	printf("\n --- Process Over ---\n");
	smd_handle_destory(handle);
	return 0;
}*/
