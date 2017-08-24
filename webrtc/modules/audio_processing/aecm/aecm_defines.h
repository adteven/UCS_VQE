/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_AUDIO_PROCESSING_AECM_AECM_DEFINES_H_
#define WEBRTC_MODULES_AUDIO_PROCESSING_AECM_AECM_DEFINES_H_
#define SUPPORT_32K
/*#define PC_DEMO_VERSION*/
#ifdef PC_DEMO_VERSION
#define  FAR_FILE "aecFar.pcm"
#define  FAR_BLOCK_FILE "aecFar_block.pcm"
#define  FAR_ALIGNED_FILE "aecFar_aligned.pcm"
#define  NEAR_FILE "aecNear.pcm"
#define  NEAR_CLEAN_FILE "aecNear1.pcm"
#define  OUT_FILE "aecOut_20160808.pcm"
#define  MODE_FILE "aecmMode.pcm"
#ifdef SUPPORT_32K
#define FAR_FILE_NAME_32 "far32.pcm"
#define NEAR_FILE_NAME_32 "near32.pcm"
#endif
#else
#define  FAR_FILE "/mnt/sdcard/aecFar.pcm"
#define  FAR_BLOCK_FILE "/mnt/sdcard/aecFar_block.pcm"
#define  FAR_ALIGNED_FILE "/mnt/sdcard/aecFar_aligned.pcm"
#define  NEAR_FILE "/mnt/sdcard/aecNear.pcm"
#define  NEAR_CLEAN_FILE "/mnt/sdcard/aecNear1.pcm"
#define  OUT_FILE "/mnt/sdcard/aecOut_20160808.pcm"
#define  MODE_FILE "/mnt/sdcard/aecmMode.pcm"
#ifdef SUPPORT_32K
#define FAR_FILE_NAME_32 "/mnt/sdcard/far32.pcm"
#define NEAR_FILE_NAME_32 "/mnt/sdcard/near32.pcm"
#endif
#endif

#define DEFAULT_ECHO_MODE 3 
#define NEAR_FAR_EN_RATIO_BUF_LEN     MAX_BUF_LEN*2
#define SPEC_BUF_LEN MAX_BUF_LEN

#define CLOSE_EST_BUF_DELAY  // 219 
#define CLOSE_DELAY_COMPARE 

//#define MUSIC_OR_SPEECH
#define SMD_BUF_MAX_SIZE 1280
#define DEFAUL_NEAR_SIGNAL_TYPE 0  //speech
#define K_MAX_PREF_BAND_MODIFICATION

//#define AFC
#ifdef AFC
#define PEAK_NUM   4
#define VERY_LONG_TERM_BLOCK_NUM  750
#endif

#define AECM_DYNAMIC_Q                 /* Turn on/off dynamic Q-domain. */

/* Algorithm parameters */
#define FRAME_LEN       80             /* Total frame length, 10 ms. */

#define PART_LEN        64             /* Length of partition. */
#define PART_LEN_SHIFT  7              /* Length of (PART_LEN * 2) in base 2. */

#define PART_LEN1       (PART_LEN + 1)  /* Unique fft coefficients. */
#define PART_LEN2       (PART_LEN << 1) /* Length of partition * 2. */
#define PART_LEN4       (PART_LEN << 2) /* Length of partition * 4. */
#define FAR_BUF_LEN     PART_LEN4       /* Length of buffers. */


#define MAX_DELAY       200



/* Counter parameters */
#define CONV_LEN        512          /* Convergence length used at startup. */
#define CONV_LEN2       (CONV_LEN << 1) /* Used at startup. */

/* Energy parameters */
#define MAX_BUF_LEN     64           /* History length of energy signals. */
#define FAR_ENERGY_MIN  1025         /* Lowest Far energy level: At least 2 */
                                     /* in energy. */
#define FAR_ENERGY_DIFF 929          /* Allowed difference between max */
                                     /* and min. */
#define ENERGY_DEV_OFFSET       0    /* The energy error offset in Q8. */

#define ENERGY_DEV_TOL  200          /* The energy estimation tolerance (Q8). */
#define FAR_ENERGY_VAD_REGION   230  /* Far VAD tolerance region. */

/* Stepsize parameters */
#define MU_MIN          10          /* Min stepsize 2^-MU_MIN (far end energy */
                                    /* dependent). */
#define MU_MAX          1           /* Max stepsize 2^-MU_MAX (far end energy */
                                    /* dependent). */
#define MU_DIFF         9           /* MU_MIN - MU_MAX */

/* Channel parameters */



#define MIN_MSE_COUNT   20 /* Min number of consecutive blocks with enough */
                           /* far end energy to compare channel estimates. */
#define MIN_MSE_DIFF    16 /* The ratio between adapted and stored channel to */
                           /* accept a new storage (0.8 in Q-MSE_RESOLUTION). */

#define MSE_RESOLUTION  5           /* MSE parameter resolution. */
#define RESOLUTION_CHANNEL16    12  /* W16 Channel in Q-RESOLUTION_CHANNEL16. */
#define RESOLUTION_CHANNEL32    28  /* W32 Channel in Q-RESOLUTION_CHANNEL. */
#define CHANNEL_VAD     16          /* Minimum energy in frequency band */
                                    /* to update channel. */

/* Suppression gain parameters: SUPGAIN parameters in Q-(RESOLUTION_SUPGAIN). */
#define RESOLUTION_SUPGAIN      8     /* Channel in Q-(RESOLUTION_SUPGAIN). */
#define SUPGAIN_DEFAULT (1 << RESOLUTION_SUPGAIN)  /* Default. */
#define SUPGAIN_ERROR_PARAM_A   3072  /* Estimation error parameter */
                                      /* (Maximum gain) (8 in Q8). */
#define SUPGAIN_ERROR_PARAM_B   1536  /* Estimation error parameter */
                                      /* (Gain before going down). */
#define SUPGAIN_ERROR_PARAM_D   SUPGAIN_DEFAULT /* Estimation error parameter */
                                /* (Should be the same as Default) (1 in Q8). */
#define SUPGAIN_EPC_DT  200     /* SUPGAIN_ERROR_PARAM_C * ENERGY_DEV_TOL */

/* Defines for "check delay estimation" */
#define CORR_WIDTH      31      /* Number of samples to correlate over. */
#define CORR_MAX        16      /* Maximum correlation offset. */
#define CORR_MAX_BUF    63
#define CORR_DEV        4
#define CORR_MAX_LEVEL  20
#define CORR_MAX_LOW    4
#define CORR_BUF_LEN    (CORR_MAX << 1) + 1
/* Note that CORR_WIDTH + 2*CORR_MAX <= MAX_BUF_LEN. */

#define ONE_Q14         (1 << 14)

/* NLP defines */
#define NLP_COMP_LOW    3277    /* 0.2 in Q14 */
#define NLP_COMP_HIGH   ONE_Q14 /* 1 in Q14 */

#endif
