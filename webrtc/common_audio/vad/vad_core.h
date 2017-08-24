/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


/*
 * This header file includes the descriptions of the core VAD calls.
 */

#ifndef WEBRTC_COMMON_AUDIO_VAD_VAD_CORE_H_
#define WEBRTC_COMMON_AUDIO_VAD_VAD_CORE_H_

#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"
#include "webrtc/typedefs.h"

enum { kNumChannels = 6 };  // Number of frequency bands (named channels).
enum { kNumGaussians = 2 };  // Number of Gaussians per channel in the GMM.
enum { kTableSize = kNumChannels * kNumGaussians };
enum { kMinEnergy = 10 };  // Minimum energy required to trigger audio signal.


#ifndef G729B_VAD
//#define G729B_VAD
#endif

#ifdef G729B_VAD
#include <stdio.h>
#include <stdlib.h>
#endif

#ifdef G729B_VAD
#define		VAD_L_TOTAL     240        /* Total size of speech buffer.              */
#define		VAD_L_WINDOW    240        /* Window size in LP analysis.               */
#define		VAD_L_NEXT      40         /* Lookahead in LP analysis.                 */
#define		VAD_L_FRAME     80         /* Frame size.                               */
#define		VAD_L_SUBFR     40         /* Subframe size.                            */
#define		VAD_M           10         /* Order of LP filter.                       */
#define		VAD_MP1         (VAD_M+1)      /* Order of LP filter + 1                    */
#define		VAD_NC          5          /*  VAD_NC = VAD_M/2                                 */
#define     VAD_GRID_POINTS 60         /* search grid                               */
#define     VAD_NP            12                  /* Increased LPC order */
#define     VAD_NOISE         0
#define     VAD_VOICE         1
#define     VAD_INIT_FRAME    32
#define     VAD_INIT_COUNT    20
#define     VAD_ZC_START      120
#define     VAD_ZC_END        200

#define VAD_MAX_32 (int32_t)0x7fffffffL
#define VAD_MIN_32 (int32_t)0x80000000L

#define VAD_MAX_16 (int16_t)0x7fff
#define VAD_MIN_16 (int16_t)0x8000
#endif
typedef struct VadInstT_
{

    int vad;
    int32_t downsampling_filter_states[4];
    WebRtcSpl_State48khzTo8khz state_48_to_8;
    int16_t noise_means[kTableSize];
    int16_t speech_means[kTableSize];
    int16_t noise_stds[kTableSize];
    int16_t speech_stds[kTableSize];
    // TODO(bjornv): Change to |frame_count|.
    int32_t frame_counter;
    int16_t over_hang; // Over Hang
    int16_t num_of_speech;
    // TODO(bjornv): Change to |age_vector|.
    int16_t index_vector[16 * kNumChannels];
    int16_t low_value_vector[16 * kNumChannels];
    // TODO(bjornv): Change to |median|.
    int16_t mean_value[kNumChannels];
    int16_t upper_state[5];
    int16_t lower_state[5];
    int16_t hp_filter_state[4];
    int16_t over_hang_max_1[3];
    int16_t over_hang_max_2[3];
    int16_t individual[3];
    int16_t total[3];

    int init_flag;

#ifdef G729B_VAD
	int16_t old_speech[VAD_L_TOTAL];
	int16_t *speech;
	int16_t *p_window;
	int16_t *new_speech;
	int16_t lsp_old[VAD_M];
	int16_t lsp_old_q[VAD_M];
	int16_t pastVad;
	int16_t ppastVad;

	int16_t y2_hi;
	int16_t y2_lo;
	int16_t y1_hi;
	int16_t y1_lo;
	int16_t x0;
	int16_t x1;

	int16_t frame;
	int16_t MeanLSF[VAD_M];
	int16_t Min_buffer[16];
	int16_t Prev_Min, Next_Min, Min;
	int16_t MeanE, MeanSE, MeanSLE, MeanSZC;
	int16_t prev_energy;
	int16_t count_sil, count_update, count_ext;
	int16_t flag, v_flag, less_count;

	int16_t VadDecision[3];
#endif

} VadInstT;

// Initializes the core VAD component. The default aggressiveness mode is
// controlled by |kDefaultMode| in vad_core.c.
//
// - self [i/o] : Instance that should be initialized
//
// returns      : 0 (OK), -1 (NULL pointer in or if the default mode can't be
//                set)
int WebRtcVad_InitCore(VadInstT* self);

/****************************************************************************
 * WebRtcVad_set_mode_core(...)
 *
 * This function changes the VAD settings
 *
 * Input:
 *      - inst      : VAD instance
 *      - mode      : Aggressiveness degree
 *                    0 (High quality) - 3 (Highly aggressive)
 *
 * Output:
 *      - inst      : Changed  instance
 *
 * Return value     :  0 - Ok
 *                    -1 - Error
 */

int WebRtcVad_set_mode_core(VadInstT* self, int mode);

/****************************************************************************
 * WebRtcVad_CalcVad48khz(...)
 * WebRtcVad_CalcVad32khz(...)
 * WebRtcVad_CalcVad16khz(...)
 * WebRtcVad_CalcVad8khz(...)
 *
 * Calculate probability for active speech and make VAD decision.
 *
 * Input:
 *      - inst          : Instance that should be initialized
 *      - speech_frame  : Input speech frame
 *      - frame_length  : Number of input samples
 *
 * Output:
 *      - inst          : Updated filter states etc.
 *
 * Return value         : VAD decision
 *                        0 - No active speech
 *                        1-6 - Active speech
 */
int WebRtcVad_CalcVad48khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length);
int WebRtcVad_CalcVad32khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length);
int WebRtcVad_CalcVad16khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length);
int WebRtcVad_CalcVad8khz(VadInstT* inst, const int16_t* speech_frame,
                          int frame_length);

#ifdef G729B_VAD
extern int16_t Vad_Overflow;
extern int16_t Vad_Carry;

#define VAD_MAX_32 (int32_t)0x7fffffffL
#define VAD_MIN_32 (int32_t)0x80000000L

#define VAD_MAX_16 (int16_t)0x7fff
#define VAD_MIN_16 (int16_t)0x8000

int16_t Vad_sature(int32_t L_var1);             /* Limit to 16 bits,    1 */
int16_t Vad_add(int16_t var1, int16_t var2);     /* Short Vad_add,           1 */
int16_t Vad_sub(int16_t var1, int16_t var2);     /* Short Vad_sub,           1 */
int16_t Vad_abs_s(int16_t var1);                /* Short abs,           1 */
int16_t Vad_shl(int16_t var1, int16_t var2);     /* Short shift left,    1 */
int16_t Vad_shr(int16_t var1, int16_t var2);     /* Short shift right,   1 */
int16_t Vad_mult(int16_t var1, int16_t var2);    /* Short Vad_mult,          1 */
int32_t Vad_L_mult(int16_t var1, int16_t var2);  /* Long Vad_mult,           1 */
int16_t Vad_negate(int16_t var1);               /* Short Vad_negate,        1 */
int16_t Vad_extract_h(int32_t L_var1);          /* Extract high,        1 */
int16_t Vad_extract_l(int32_t L_var1);          /* Extract low,         1 */
int16_t Vad_round(int32_t L_var1);              /* Vad_round,               1 */
int32_t Vad_L_mac(int32_t L_var3, int16_t var1, int16_t var2); /* Mac,    1 */
int32_t Vad_L_msu(int32_t L_var3, int16_t var1, int16_t var2); /* Msu,    1 */
int32_t Vad_L_add(int32_t L_var1, int32_t L_var2);   /* Long Vad_add,        2 */
int32_t Vad_L_sub(int32_t L_var1, int32_t L_var2);   /* Long Vad_sub,        2 */
int32_t Vad_L_negate(int32_t L_var1);               /* Long Vad_negate,     2 */
int16_t Vad_mult_r(int16_t var1, int16_t var2);  /* Vad_mult with Vad_round,     2 */
int32_t Vad_L_shl(int32_t L_var1, int16_t var2); /* Long shift left,     2 */
int32_t Vad_L_shr(int32_t L_var1, int16_t var2); /* Long shift right,    2 */
int32_t Vad_L_deposit_h(int16_t var1);       /* 16 bit var1 -> MSB,     2 */
int32_t Vad_L_deposit_l(int16_t var1);       /* 16 bit var1 -> LSB,     2 */

int32_t Vad_L_abs(int32_t L_var1);            /* Long abs,              3 */

int16_t Vad_norm_s(int16_t var1);             /* Short norm,           15 */

int16_t Vad_div_s(int16_t var1, int16_t var2); /* Short division,       18 */

int16_t Vad_norm_l(int32_t L_var1);           /* Long norm,            30 */

void   Vad_L_Extract(int32_t L_32, int16_t *hi, int16_t *lo);
int32_t Vad_L_Comp(int16_t hi, int16_t lo);
int32_t Vad_Mpy_32(int16_t hi1, int16_t lo1, int16_t hi2, int16_t lo2);
int32_t Vad_Mpy_32_16(int16_t hi, int16_t lo, int16_t n);
int32_t Vad_Div_32(int32_t L_num, int16_t denom_hi, int16_t denom_lo);

void Vad_Log2(int32_t L_x, int16_t *exponent, int16_t *fraction);

int WebRtcVad_CalcVad8khz_G729B(VadInstT *self, short* audio_frame, int frame_length);

void Vad_HignPass_Process(VadInstT *self, int16_t *signal_in, int16_t *signal_out, int16_t lg);

void Vad_Main_Process(VadInstT *self, int16_t frame, int16_t vad_enable, short *vadDecision);


/*--------------------------------------------------------------------------*
* LPC analysis and filtering                                               *
*--------------------------------------------------------------------------*/

void Vad_Autocorr(int16_t x[], int16_t m, int16_t r_h[], int16_t r_l[], int16_t *exp_R0);

void Vad_Lag_window(int16_t m, int16_t r_h[], int16_t r_l[]);

void Vad_Levinson(int16_t Rh[], int16_t Rl[], int16_t A[], int16_t rc[], int16_t *Err);

void Vad_Az_lsp(int16_t a[], int16_t lsp[], int16_t old_lsp[]);

void Vad_Lsp_lsf(int16_t lsp[], int16_t lsf[], int16_t m);

void Vad_Copy(int16_t x[], int16_t y[], int16_t L);

void Vad_Set_zero(int16_t x[], int16_t L);

void Vad_New(VadInstT *self, int16_t rc, int16_t *lsf, int16_t *r_h, int16_t *r_l, int16_t exp_R0,
	int16_t *sigpp, int16_t frm_count, int16_t prev_marker, int16_t pprev_marker, int16_t *marker);
#endif

#endif  // WEBRTC_COMMON_AUDIO_VAD_VAD_CORE_H_
