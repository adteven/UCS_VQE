/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/common_audio/vad/vad_core.h"

#include "webrtc/common_audio/signal_processing/include/signal_processing_library.h"
#include "webrtc/common_audio/vad/vad_filterbank.h"
#include "webrtc/common_audio/vad/vad_gmm.h"
#include "webrtc/common_audio/vad/vad_sp.h"
#include "webrtc/typedefs.h"

// Spectrum Weighting
static const int16_t kSpectrumWeight[kNumChannels] = { 6, 8, 10, 12, 14, 16 };
static const int16_t kNoiseUpdateConst = 655; // Q15
static const int16_t kSpeechUpdateConst = 6554; // Q15
static const int16_t kBackEta = 154; // Q8
// Minimum difference between the two models, Q5
static const int16_t kMinimumDifference[kNumChannels] = {
    544, 544, 576, 576, 576, 576 };
// Upper limit of mean value for speech model, Q7
static const int16_t kMaximumSpeech[kNumChannels] = {
    11392, 11392, 11520, 11520, 11520, 11520 };
// Minimum value for mean value
static const int16_t kMinimumMean[kNumGaussians] = { 640, 768 };
// Upper limit of mean value for noise model, Q7
static const int16_t kMaximumNoise[kNumChannels] = {
    9216, 9088, 8960, 8832, 8704, 8576 };
// Start values for the Gaussian models, Q7
// Weights for the two Gaussians for the six channels (noise)
static const int16_t kNoiseDataWeights[kTableSize] = {
    34, 62, 72, 66, 53, 25, 94, 66, 56, 62, 75, 103 };
// Weights for the two Gaussians for the six channels (speech)
static const int16_t kSpeechDataWeights[kTableSize] = {
    48, 82, 45, 87, 50, 47, 80, 46, 83, 41, 78, 81 };
// Means for the two Gaussians for the six channels (noise)
static const int16_t kNoiseDataMeans[kTableSize] = {
    6738, 4892, 7065, 6715, 6771, 3369, 7646, 3863, 7820, 7266, 5020, 4362 };
// Means for the two Gaussians for the six channels (speech)
static const int16_t kSpeechDataMeans[kTableSize] = {
    8306, 10085, 10078, 11823, 11843, 6309, 9473, 9571, 10879, 7581, 8180, 7483
};
// Stds for the two Gaussians for the six channels (noise)
static const int16_t kNoiseDataStds[kTableSize] = {
    378, 1064, 493, 582, 688, 593, 474, 697, 475, 688, 421, 455 };
// Stds for the two Gaussians for the six channels (speech)
static const int16_t kSpeechDataStds[kTableSize] = {
    555, 505, 567, 524, 585, 1231, 509, 828, 492, 1540, 1079, 850 };

// Constants used in GmmProbability().
//
// Maximum number of counted speech (VAD = 1) frames in a row.
static const int16_t kMaxSpeechFrames = 6;
// Minimum standard deviation for both speech and noise.
static const int16_t kMinStd = 384;

// Constants in WebRtcVad_InitCore().
// Default aggressiveness mode.
static const short kDefaultMode = 0;
static const int kInitCheck = 42;

// Constants used in WebRtcVad_set_mode_core().
//
// Thresholds for different frame lengths (10 ms, 20 ms and 30 ms).
//
// Mode 0, Quality.
static const int16_t kOverHangMax1Q[3] = { 8, 4, 3 };
static const int16_t kOverHangMax2Q[3] = { 14, 7, 5 };
static const int16_t kLocalThresholdQ[3] = { 24, 21, 24 };
static const int16_t kGlobalThresholdQ[3] = { 57, 48, 57 };
// Mode 1, Low bitrate.
static const int16_t kOverHangMax1LBR[3] = { 8, 4, 3 };
static const int16_t kOverHangMax2LBR[3] = { 14, 7, 5 };
static const int16_t kLocalThresholdLBR[3] = { 37, 32, 37 };
static const int16_t kGlobalThresholdLBR[3] = { 100, 80, 100 };
// Mode 2, Aggressive.
static const int16_t kOverHangMax1AGG[3] = { 6, 3, 2 };
static const int16_t kOverHangMax2AGG[3] = { 9, 5, 3 };
static const int16_t kLocalThresholdAGG[3] = { 82, 78, 82 };
static const int16_t kGlobalThresholdAGG[3] = { 285, 260, 285 };
// Mode 3, Very aggressive.
static const int16_t kOverHangMax1VAG[3] = { 6, 3, 2 };
static const int16_t kOverHangMax2VAG[3] = { 9, 5, 3 };
static const int16_t kLocalThresholdVAG[3] = { 94, 94, 94 };
static const int16_t kGlobalThresholdVAG[3] = { 1100, 1050, 1100 };

// Calculates the weighted average w.r.t. number of Gaussians. The |data| are
// updated with an |offset| before averaging.
//
// - data     [i/o] : Data to average.
// - offset   [i]   : An offset added to |data|.
// - weights  [i]   : Weights used for averaging.
//
// returns          : The weighted average.
static int32_t WeightedAverage(int16_t* data, int16_t offset,
                               const int16_t* weights) {
  int k;
  int32_t weighted_average = 0;

  for (k = 0; k < kNumGaussians; k++) {
    data[k * kNumChannels] += offset;
    weighted_average += data[k * kNumChannels] * weights[k * kNumChannels];
  }
  return weighted_average;
}

// Calculates the probabilities for both speech and background noise using
// Gaussian Mixture Models (GMM). A hypothesis-test is performed to decide which
// type of signal is most probable.
//
// - self           [i/o] : Pointer to VAD instance
// - features       [i]   : Feature vector of length |kNumChannels|
//                          = log10(energy in frequency band)
// - total_power    [i]   : Total power in audio frame.
// - frame_length   [i]   : Number of input samples
//
// - returns              : the VAD decision (0 - noise, 1 - speech).
static int16_t GmmProbability(VadInstT* self, int16_t* features,
                              int16_t total_power, int frame_length) {
  int channel, k;
  int16_t feature_minimum;
  int16_t h0, h1;
  int16_t log_likelihood_ratio;
  int16_t vadflag = 0;
  int16_t shifts_h0, shifts_h1;
  int16_t tmp_s16, tmp1_s16, tmp2_s16;
  int16_t diff;
  int gaussian;
  int16_t nmk, nmk2, nmk3, smk, smk2, nsk, ssk;
  int16_t delt, ndelt;
  int16_t maxspe, maxmu;
  int16_t deltaN[kTableSize], deltaS[kTableSize];
  int16_t ngprvec[kTableSize] = { 0 };  // Conditional probability = 0.
  int16_t sgprvec[kTableSize] = { 0 };  // Conditional probability = 0.
  int32_t h0_test, h1_test;
  int32_t tmp1_s32, tmp2_s32;
  int32_t sum_log_likelihood_ratios = 0;
  int32_t noise_global_mean, speech_global_mean;
  int32_t noise_probability[kNumGaussians], speech_probability[kNumGaussians];
  int16_t overhead1, overhead2, individualTest, totalTest;

  // Set various thresholds based on frame lengths (80, 160 or 240 samples).
  if (frame_length == 80) {
    overhead1 = self->over_hang_max_1[0];
    overhead2 = self->over_hang_max_2[0];
    individualTest = self->individual[0];
    totalTest = self->total[0];
  } else if (frame_length == 160) {
    overhead1 = self->over_hang_max_1[1];
    overhead2 = self->over_hang_max_2[1];
    individualTest = self->individual[1];
    totalTest = self->total[1];
  } else {
    overhead1 = self->over_hang_max_1[2];
    overhead2 = self->over_hang_max_2[2];
    individualTest = self->individual[2];
    totalTest = self->total[2];
  }

  if (total_power > kMinEnergy) {
    // The signal power of current frame is large enough for processing. The
    // processing consists of two parts:
    // 1) Calculating the likelihood of speech and thereby a VAD decision.
    // 2) Updating the underlying model, w.r.t., the decision made.

    // The detection scheme is an LRT with hypothesis
    // H0: Noise
    // H1: Speech
    //
    // We combine a global LRT with local tests, for each frequency sub-band,
    // here defined as |channel|.
    for (channel = 0; channel < kNumChannels; channel++) {
      // For each channel we model the probability with a GMM consisting of
      // |kNumGaussians|, with different means and standard deviations depending
      // on H0 or H1.
      h0_test = 0;
      h1_test = 0;
      for (k = 0; k < kNumGaussians; k++) {
        gaussian = channel + k * kNumChannels;
        // Probability under H0, that is, probability of frame being noise.
        // Value given in Q27 = Q7 * Q20.
        tmp1_s32 = WebRtcVad_GaussianProbability(features[channel],
                                                 self->noise_means[gaussian],
                                                 self->noise_stds[gaussian],
                                                 &deltaN[gaussian]);
        noise_probability[k] = kNoiseDataWeights[gaussian] * tmp1_s32;
        h0_test += noise_probability[k];  // Q27

        // Probability under H1, that is, probability of frame being speech.
        // Value given in Q27 = Q7 * Q20.
        tmp1_s32 = WebRtcVad_GaussianProbability(features[channel],
                                                 self->speech_means[gaussian],
                                                 self->speech_stds[gaussian],
                                                 &deltaS[gaussian]);
        speech_probability[k] = kSpeechDataWeights[gaussian] * tmp1_s32;
        h1_test += speech_probability[k];  // Q27
      }

      // Calculate the log likelihood ratio: log2(Pr{X|H1} / Pr{X|H1}).
      // Approximation:
      // log2(Pr{X|H1} / Pr{X|H1}) = log2(Pr{X|H1}*2^Q) - log2(Pr{X|H1}*2^Q)
      //                           = log2(h1_test) - log2(h0_test)
      //                           = log2(2^(31-shifts_h1)*(1+b1))
      //                             - log2(2^(31-shifts_h0)*(1+b0))
      //                           = shifts_h0 - shifts_h1
      //                             + log2(1+b1) - log2(1+b0)
      //                          ~= shifts_h0 - shifts_h1
      //
      // Note that b0 and b1 are values less than 1, hence, 0 <= log2(1+b0) < 1.
      // Further, b0 and b1 are independent and on the average the two terms
      // cancel.
      shifts_h0 = WebRtcSpl_NormW32(h0_test);
      shifts_h1 = WebRtcSpl_NormW32(h1_test);
      if (h0_test == 0) {
        shifts_h0 = 31;
      }
      if (h1_test == 0) {
        shifts_h1 = 31;
      }
      log_likelihood_ratio = shifts_h0 - shifts_h1;

      // Update |sum_log_likelihood_ratios| with spectrum weighting. This is
      // used for the global VAD decision.
      sum_log_likelihood_ratios +=
          (int32_t) (log_likelihood_ratio * kSpectrumWeight[channel]);

      // Local VAD decision.
      if ((log_likelihood_ratio << 2) > individualTest) {
        vadflag = 1;
      }

      // TODO(bjornv): The conditional probabilities below are applied on the
      // hard coded number of Gaussians set to two. Find a way to generalize.
      // Calculate local noise probabilities used later when updating the GMM.
      h0 = (int16_t) (h0_test >> 12);  // Q15
      if (h0 > 0) {
        // High probability of noise. Assign conditional probabilities for each
        // Gaussian in the GMM.
        tmp1_s32 = (noise_probability[0] & 0xFFFFF000) << 2;  // Q29
        ngprvec[channel] = (int16_t) WebRtcSpl_DivW32W16(tmp1_s32, h0);  // Q14
        ngprvec[channel + kNumChannels] = 16384 - ngprvec[channel];
      } else {
        // Low noise probability. Assign conditional probability 1 to the first
        // Gaussian and 0 to the rest (which is already set at initialization).
        ngprvec[channel] = 16384;
      }

      // Calculate local speech probabilities used later when updating the GMM.
      h1 = (int16_t) (h1_test >> 12);  // Q15
      if (h1 > 0) {
        // High probability of speech. Assign conditional probabilities for each
        // Gaussian in the GMM. Otherwise use the initialized values, i.e., 0.
        tmp1_s32 = (speech_probability[0] & 0xFFFFF000) << 2;  // Q29
        sgprvec[channel] = (int16_t) WebRtcSpl_DivW32W16(tmp1_s32, h1);  // Q14
        sgprvec[channel + kNumChannels] = 16384 - sgprvec[channel];
      }
    }

    // Make a global VAD decision.
    vadflag |= (sum_log_likelihood_ratios >= totalTest);

    // Update the model parameters.
    maxspe = 12800;
    for (channel = 0; channel < kNumChannels; channel++) {

      // Get minimum value in past which is used for long term correction in Q4.
      feature_minimum = WebRtcVad_FindMinimum(self, features[channel], channel);

      // Compute the "global" mean, that is the sum of the two means weighted.
      noise_global_mean = WeightedAverage(&self->noise_means[channel], 0,
                                          &kNoiseDataWeights[channel]);
      tmp1_s16 = (int16_t) (noise_global_mean >> 6);  // Q8

      for (k = 0; k < kNumGaussians; k++) {
        gaussian = channel + k * kNumChannels;

        nmk = self->noise_means[gaussian];
        smk = self->speech_means[gaussian];
        nsk = self->noise_stds[gaussian];
        ssk = self->speech_stds[gaussian];

        // Update noise mean vector if the frame consists of noise only.
        nmk2 = nmk;
        if (!vadflag) {
          // deltaN = (x-mu)/sigma^2
          // ngprvec[k] = |noise_probability[k]| /
          //   (|noise_probability[0]| + |noise_probability[1]|)

          // (Q14 * Q11 >> 11) = Q14.
          delt = (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(ngprvec[gaussian],
                                                     deltaN[gaussian],
                                                     11);
          // Q7 + (Q14 * Q15 >> 22) = Q7.
          nmk2 = nmk + (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(delt,
                                                           kNoiseUpdateConst,
                                                           22);
        }

        // Long term correction of the noise mean.
        // Q8 - Q8 = Q8.
        ndelt = (feature_minimum << 4) - tmp1_s16;
        // Q7 + (Q8 * Q8) >> 9 = Q7.
        nmk3 = nmk2 + (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(ndelt, kBackEta, 9);

        // Control that the noise mean does not drift to much.
        tmp_s16 = (int16_t) ((k + 5) << 7);
        if (nmk3 < tmp_s16) {
          nmk3 = tmp_s16;
        }
        tmp_s16 = (int16_t) ((72 + k - channel) << 7);
        if (nmk3 > tmp_s16) {
          nmk3 = tmp_s16;
        }
        self->noise_means[gaussian] = nmk3;

        if (vadflag) {
          // Update speech mean vector:
          // |deltaS| = (x-mu)/sigma^2
          // sgprvec[k] = |speech_probability[k]| /
          //   (|speech_probability[0]| + |speech_probability[1]|)

          // (Q14 * Q11) >> 11 = Q14.
          delt = (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(sgprvec[gaussian],
                                                     deltaS[gaussian],
                                                     11);
          // Q14 * Q15 >> 21 = Q8.
          tmp_s16 = (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(delt,
                                                        kSpeechUpdateConst,
                                                        21);
          // Q7 + (Q8 >> 1) = Q7. With rounding.
          smk2 = smk + ((tmp_s16 + 1) >> 1);

          // Control that the speech mean does not drift to much.
          maxmu = maxspe + 640;
          if (smk2 < kMinimumMean[k]) {
            smk2 = kMinimumMean[k];
          }
          if (smk2 > maxmu) {
            smk2 = maxmu;
          }
          self->speech_means[gaussian] = smk2;  // Q7.

          // (Q7 >> 3) = Q4. With rounding.
          tmp_s16 = ((smk + 4) >> 3);

          tmp_s16 = features[channel] - tmp_s16;  // Q4
          // (Q11 * Q4 >> 3) = Q12.
          tmp1_s32 = WEBRTC_SPL_MUL_16_16_RSFT(deltaS[gaussian], tmp_s16, 3);
          tmp2_s32 = tmp1_s32 - 4096;
          tmp_s16 = sgprvec[gaussian] >> 2;
          // (Q14 >> 2) * Q12 = Q24.
          tmp1_s32 = tmp_s16 * tmp2_s32;

          tmp2_s32 = tmp1_s32 >> 4;  // Q20

          // 0.1 * Q20 / Q7 = Q13.
          if (tmp2_s32 > 0) {
            tmp_s16 = (int16_t) WebRtcSpl_DivW32W16(tmp2_s32, ssk * 10);
          } else {
            tmp_s16 = (int16_t) WebRtcSpl_DivW32W16(-tmp2_s32, ssk * 10);
            tmp_s16 = -tmp_s16;
          }
          // Divide by 4 giving an update factor of 0.025 (= 0.1 / 4).
          // Note that division by 4 equals shift by 2, hence,
          // (Q13 >> 8) = (Q13 >> 6) / 4 = Q7.
          tmp_s16 += 128;  // Rounding.
          ssk += (tmp_s16 >> 8);
          if (ssk < kMinStd) {
            ssk = kMinStd;
          }
          self->speech_stds[gaussian] = ssk;
        } else {
          // Update GMM variance vectors.
          // deltaN * (features[channel] - nmk) - 1
          // Q4 - (Q7 >> 3) = Q4.
          tmp_s16 = features[channel] - (nmk >> 3);
          // (Q11 * Q4 >> 3) = Q12.
          tmp1_s32 = WEBRTC_SPL_MUL_16_16_RSFT(deltaN[gaussian], tmp_s16, 3);
          tmp1_s32 -= 4096;

          // (Q14 >> 2) * Q12 = Q24.
          tmp_s16 = (ngprvec[gaussian] + 2) >> 2;
          tmp2_s32 = tmp_s16 * tmp1_s32;
          // Q20  * approx 0.001 (2^-10=0.0009766), hence,
          // (Q24 >> 14) = (Q24 >> 4) / 2^10 = Q20.
          tmp1_s32 = tmp2_s32 >> 14;

          // Q20 / Q7 = Q13.
          if (tmp1_s32 > 0) {
            tmp_s16 = (int16_t) WebRtcSpl_DivW32W16(tmp1_s32, nsk);
          } else {
            tmp_s16 = (int16_t) WebRtcSpl_DivW32W16(-tmp1_s32, nsk);
            tmp_s16 = -tmp_s16;
          }
          tmp_s16 += 32;  // Rounding
          nsk += tmp_s16 >> 6;  // Q13 >> 6 = Q7.
          if (nsk < kMinStd) {
            nsk = kMinStd;
          }
          self->noise_stds[gaussian] = nsk;
        }
      }

      // Separate models if they are too close.
      // |noise_global_mean| in Q14 (= Q7 * Q7).
      noise_global_mean = WeightedAverage(&self->noise_means[channel], 0,
                                          &kNoiseDataWeights[channel]);

      // |speech_global_mean| in Q14 (= Q7 * Q7).
      speech_global_mean = WeightedAverage(&self->speech_means[channel], 0,
                                           &kSpeechDataWeights[channel]);

      // |diff| = "global" speech mean - "global" noise mean.
      // (Q14 >> 9) - (Q14 >> 9) = Q5.
      diff = (int16_t) (speech_global_mean >> 9) -
          (int16_t) (noise_global_mean >> 9);
      if (diff < kMinimumDifference[channel]) {
        tmp_s16 = kMinimumDifference[channel] - diff;

        // |tmp1_s16| = ~0.8 * (kMinimumDifference - diff) in Q7.
        // |tmp2_s16| = ~0.2 * (kMinimumDifference - diff) in Q7.
        tmp1_s16 = (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(13, tmp_s16, 2);
        tmp2_s16 = (int16_t) WEBRTC_SPL_MUL_16_16_RSFT(3, tmp_s16, 2);

        // Move Gaussian means for speech model by |tmp1_s16| and update
        // |speech_global_mean|. Note that |self->speech_means[channel]| is
        // changed after the call.
        speech_global_mean = WeightedAverage(&self->speech_means[channel],
                                             tmp1_s16,
                                             &kSpeechDataWeights[channel]);

        // Move Gaussian means for noise model by -|tmp2_s16| and update
        // |noise_global_mean|. Note that |self->noise_means[channel]| is
        // changed after the call.
        noise_global_mean = WeightedAverage(&self->noise_means[channel],
                                            -tmp2_s16,
                                            &kNoiseDataWeights[channel]);
      }

      // Control that the speech & noise means do not drift to much.
      maxspe = kMaximumSpeech[channel];
      tmp2_s16 = (int16_t) (speech_global_mean >> 7);
      if (tmp2_s16 > maxspe) {
        // Upper limit of speech model.
        tmp2_s16 -= maxspe;

        for (k = 0; k < kNumGaussians; k++) {
          self->speech_means[channel + k * kNumChannels] -= tmp2_s16;
        }
      }

      tmp2_s16 = (int16_t) (noise_global_mean >> 7);
      if (tmp2_s16 > kMaximumNoise[channel]) {
        tmp2_s16 -= kMaximumNoise[channel];

        for (k = 0; k < kNumGaussians; k++) {
          self->noise_means[channel + k * kNumChannels] -= tmp2_s16;
        }
      }
    }
    self->frame_counter++;
  }

  // Smooth with respect to transition hysteresis.
  if (!vadflag) {
    if (self->over_hang > 0) {
      vadflag = 2 + self->over_hang;
      self->over_hang--;
    }
    self->num_of_speech = 0;
  } else {
    self->num_of_speech++;
    if (self->num_of_speech > kMaxSpeechFrames) {
      self->num_of_speech = kMaxSpeechFrames;
      self->over_hang = overhead2;
    } else {
      self->over_hang = overhead1;
    }
  }
  return vadflag;
}

// Initialize the VAD. Set aggressiveness mode to default value.
int WebRtcVad_InitCore(VadInstT* self) {
  int i;

  if (self == NULL) {
    return -1;
  }

  // Initialization of general struct variables.
  self->vad = 1;  // Speech active (=1).
  self->frame_counter = 0;
  self->over_hang = 0;
  self->num_of_speech = 0;

  // Initialization of downsampling filter state.
  memset(self->downsampling_filter_states, 0,
         sizeof(self->downsampling_filter_states));

  // Initialization of 48 to 8 kHz downsampling.
  WebRtcSpl_ResetResample48khzTo8khz(&self->state_48_to_8);

  // Read initial PDF parameters.
  for (i = 0; i < kTableSize; i++) {
    self->noise_means[i] = kNoiseDataMeans[i];
    self->speech_means[i] = kSpeechDataMeans[i];
    self->noise_stds[i] = kNoiseDataStds[i];
    self->speech_stds[i] = kSpeechDataStds[i];
  }

  // Initialize Index and Minimum value vectors.
  for (i = 0; i < 16 * kNumChannels; i++) {
    self->low_value_vector[i] = 10000;
    self->index_vector[i] = 0;
  }

  // Initialize splitting filter states.
  memset(self->upper_state, 0, sizeof(self->upper_state));
  memset(self->lower_state, 0, sizeof(self->lower_state));

  // Initialize high pass filter states.
  memset(self->hp_filter_state, 0, sizeof(self->hp_filter_state));

  // Initialize mean value memory, for WebRtcVad_FindMinimum().
  for (i = 0; i < kNumChannels; i++) {
    self->mean_value[i] = 1600;
  }

  // Set aggressiveness mode to default (=|kDefaultMode|).
  if (WebRtcVad_set_mode_core(self, kDefaultMode) != 0) {
    return -1;
  }

  self->init_flag = kInitCheck;

#ifdef G729B_VAD
  self->new_speech = self->old_speech + VAD_L_TOTAL - VAD_L_FRAME;
  self->speech = self->new_speech - VAD_L_NEXT;
  self->p_window = self->old_speech + VAD_L_TOTAL - VAD_L_WINDOW;

  Vad_Set_zero(self->old_speech, VAD_L_TOTAL);
  self->lsp_old[0] = 30000;
  self->lsp_old[1] = 26000;
  self->lsp_old[2] = 21000;
  self->lsp_old[3] = 15000;
  self->lsp_old[4] = 8000;
  self->lsp_old[5] = 0;
  self->lsp_old[6] = -8000;
  self->lsp_old[7] = -15000;
  self->lsp_old[8] = -21000;
  self->lsp_old[9] = -26000;

  Vad_Copy(self->lsp_old, self->lsp_old_q, VAD_M);

  self->pastVad = 1;
  self->ppastVad = 1;

  self->frame = 0;

  Vad_Set_zero(self->MeanLSF, VAD_M);

  self->MeanSE = 0;
  self->MeanSLE = 0;
  self->MeanE = 0;
  self->MeanSZC = 0;
  self->count_sil = 0;
  self->count_update = 0;
  self->count_ext = 0;
  self->less_count = 0;
  self->flag = 1;
  self->Min = VAD_MAX_16;

  self->y2_hi = 0;
  self->y2_lo = 0;
  self->y1_hi = 0;
  self->y1_lo = 0;
  self->x0 = 0;
  self->x1 = 0;

  self->VadDecision[0] = 0;
  self->VadDecision[1] = 0;
  self->VadDecision[2] = 0;

#endif

  return 0;
}

// Set aggressiveness mode
int WebRtcVad_set_mode_core(VadInstT* self, int mode) {
  int return_value = 0;

  switch (mode) {
    case 0:
      // Quality mode.
      memcpy(self->over_hang_max_1, kOverHangMax1Q,
             sizeof(self->over_hang_max_1));
      memcpy(self->over_hang_max_2, kOverHangMax2Q,
             sizeof(self->over_hang_max_2));
      memcpy(self->individual, kLocalThresholdQ,
             sizeof(self->individual));
      memcpy(self->total, kGlobalThresholdQ,
             sizeof(self->total));
      break;
    case 1:
      // Low bitrate mode.
      memcpy(self->over_hang_max_1, kOverHangMax1LBR,
             sizeof(self->over_hang_max_1));
      memcpy(self->over_hang_max_2, kOverHangMax2LBR,
             sizeof(self->over_hang_max_2));
      memcpy(self->individual, kLocalThresholdLBR,
             sizeof(self->individual));
      memcpy(self->total, kGlobalThresholdLBR,
             sizeof(self->total));
      break;
    case 2:
      // Aggressive mode.
      memcpy(self->over_hang_max_1, kOverHangMax1AGG,
             sizeof(self->over_hang_max_1));
      memcpy(self->over_hang_max_2, kOverHangMax2AGG,
             sizeof(self->over_hang_max_2));
      memcpy(self->individual, kLocalThresholdAGG,
             sizeof(self->individual));
      memcpy(self->total, kGlobalThresholdAGG,
             sizeof(self->total));
      break;
    case 3:
      // Very aggressive mode.
      memcpy(self->over_hang_max_1, kOverHangMax1VAG,
             sizeof(self->over_hang_max_1));
      memcpy(self->over_hang_max_2, kOverHangMax2VAG,
             sizeof(self->over_hang_max_2));
      memcpy(self->individual, kLocalThresholdVAG,
             sizeof(self->individual));
      memcpy(self->total, kGlobalThresholdVAG,
             sizeof(self->total));
      break;
    default:
      return_value = -1;
      break;
  }

  return return_value;
}

// Calculate VAD decision by first extracting feature values and then calculate
// probability for both speech and background noise.

int WebRtcVad_CalcVad48khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length) {
  int vad;
  int i;
  int16_t speech_nb[240];  // 30 ms in 8 kHz.
  // |tmp_mem| is a temporary memory used by resample function, length is
  // frame length in 10 ms (480 samples) + 256 extra.
  int32_t tmp_mem[480 + 256] = { 0 };
  const int kFrameLen10ms48khz = 480;
  const int kFrameLen10ms8khz = 80;
  int num_10ms_frames = frame_length / kFrameLen10ms48khz;

  for (i = 0; i < num_10ms_frames; i++) {
    WebRtcSpl_Resample48khzTo8khz(speech_frame,
                                  &speech_nb[i * kFrameLen10ms8khz],
                                  &inst->state_48_to_8,
                                  tmp_mem);
  }

  // Do VAD on an 8 kHz signal
#ifdef G729B_VAD
  vad = WebRtcVad_CalcVad8khz_G729B(inst, speech_nb, frame_length / 6);
#else
  vad = WebRtcVad_CalcVad8khz(inst, speech_nb, frame_length / 6);
#endif
  //vad = WebRtcVad_CalcVad8khz(inst, speech_nb, frame_length / 6);

  return vad;
}

int WebRtcVad_CalcVad32khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length)
{
    int len, vad;
    int16_t speechWB[480]; // Downsampled speech frame: 960 samples (30ms in SWB)
    int16_t speechNB[240]; // Downsampled speech frame: 480 samples (30ms in WB)


    // Downsample signal 32->16->8 before doing VAD
    WebRtcVad_Downsampling(speech_frame, speechWB, &(inst->downsampling_filter_states[2]),
                           frame_length);
    len = frame_length / 2;

    WebRtcVad_Downsampling(speechWB, speechNB, inst->downsampling_filter_states, len);
    len /= 2;

    // Do VAD on an 8 kHz signal
#ifdef G729B_VAD
	vad = WebRtcVad_CalcVad8khz_G729B(inst, speechNB, len);
#else
	vad = WebRtcVad_CalcVad8khz(inst, speechNB, len);
#endif
    //vad = WebRtcVad_CalcVad8khz(inst, speechNB, len);

    return vad;
}

int WebRtcVad_CalcVad16khz(VadInstT* inst, const int16_t* speech_frame,
                           int frame_length)
{
    int len, vad;
    int16_t speechNB[240]; // Downsampled speech frame: 480 samples (30ms in WB)

    // Wideband: Downsample signal before doing VAD
    WebRtcVad_Downsampling(speech_frame, speechNB, inst->downsampling_filter_states,
                           frame_length);

    len = frame_length / 2;
#ifdef G729B_VAD
	vad = WebRtcVad_CalcVad8khz_G729B(inst, speechNB, len);
#else
	vad = WebRtcVad_CalcVad8khz(inst, speechNB, len);
#endif
    //vad = WebRtcVad_CalcVad8khz(inst, speechNB, len);

    return vad;
}

int WebRtcVad_CalcVad8khz(VadInstT* inst, const int16_t* speech_frame,
                          int frame_length)
{
    int16_t feature_vector[kNumChannels], total_power;

    // Get power in the bands
    total_power = WebRtcVad_CalculateFeatures(inst, speech_frame, frame_length,
                                              feature_vector);

    // Make a VAD
    inst->vad = GmmProbability(inst, feature_vector, total_power, frame_length);

    return inst->vad;
}

#ifdef G729B_VAD
int16_t Vad_sature(int32_t L_var1);

int16_t Vad_Overflow = 0;
int16_t Vad_Carry = 0;

int16_t Vad_sature(int32_t L_var1)
{
	int16_t var_out;

	if (L_var1 > 0X00007fffL)
	{
		Vad_Overflow = 1;
		var_out = VAD_MAX_16;
	}
	else if (L_var1 < (int32_t)0xffff8000L)
	{
		Vad_Overflow = 1;
		var_out = VAD_MIN_16;
	}
	else
	{
		Vad_Overflow = 0;
		var_out = Vad_extract_l(L_var1);
	}

	return(var_out);
}

int16_t Vad_add(int16_t var1, int16_t var2)
{
	int16_t var_out;
	int32_t L_somme;

	L_somme = (int32_t)var1 + var2;
	var_out = Vad_sature(L_somme);
	return(var_out);
}


int16_t Vad_sub(int16_t var1, int16_t var2)
{
	int16_t var_out;
	int32_t L_diff;

	L_diff = (int32_t)var1 - var2;
	var_out = Vad_sature(L_diff);
	return(var_out);
}

int16_t Vad_abs_s(int16_t var1)
{
	int16_t var_out;

	if (var1 == (int16_t)0X8000)
	{
		var_out = VAD_MAX_16;
	}
	else
	{
		if (var1 < 0)
		{
			var_out = -var1;
		}
		else
		{
			var_out = var1;
		}
	}
	return(var_out);
}


int16_t Vad_shl(int16_t var1, int16_t var2)
{
	int16_t var_out;
	int32_t resultat;

	if (var2 < 0)
	{
		var_out = Vad_shr(var1, -var2);
	}
	else
	{
		resultat = (int32_t)var1 * ((int32_t)1 << var2);
		if ((var2 > 15 && var1 != 0) || (resultat != (int32_t)((int16_t)resultat)))
		{
			Vad_Overflow = 1;
			var_out = (var1 > 0) ? VAD_MAX_16 : VAD_MIN_16;
		}
		else
		{
			var_out = Vad_extract_l(resultat);
		}
	}
	return(var_out);
}



int16_t Vad_shr(int16_t var1, int16_t var2)
{
	int16_t var_out;

	if (var2 < 0)
	{
		var_out = Vad_shl(var1, -var2);
	}
	else
	{
		if (var2 >= 15)
		{
			var_out = (var1 < 0) ? (int16_t)(-1) : (int16_t)0;
		}
		else
		{
			if (var1 < 0)
			{
				var_out = ~((~var1) >> var2);
			}
			else
			{
				var_out = var1 >> var2;
			}
		}
	}

	return(var_out);
}

int16_t Vad_mult(int16_t var1, int16_t var2)
{
	int16_t var_out;
	int32_t L_produit;

	L_produit = (int32_t)var1 * (int32_t)var2;

	L_produit = (L_produit & (int32_t)0xffff8000L) >> 15;

	if (L_produit & (int32_t)0x00010000L)
		L_produit = L_produit | (int32_t)0xffff0000L;

	var_out = Vad_sature(L_produit);
	return(var_out);
}


int32_t Vad_L_mult(int16_t var1, int16_t var2)
{
	int32_t L_var_out;

	L_var_out = (int32_t)var1 * (int32_t)var2;
	if (L_var_out != (int32_t)0x40000000L)
	{
		L_var_out *= 2;
	}
	else
	{
		Vad_Overflow = 1;
		L_var_out = VAD_MAX_32;
	}

	return(L_var_out);
}

int16_t Vad_negate(int16_t var1)
{
	int16_t var_out;

	var_out = (var1 == VAD_MIN_16) ? VAD_MAX_16 : -var1;
	return(var_out);
}

int16_t Vad_extract_h(int32_t L_var1)
{
	int16_t var_out;

	var_out = (int16_t)(L_var1 >> 16);
	return(var_out);
}

int16_t Vad_extract_l(int32_t L_var1)
{
	int16_t var_out;

	var_out = (int16_t)L_var1;
	return(var_out);
}


int16_t Vad_round(int32_t L_var1)
{
	int16_t var_out;
	int32_t L_arrondi;

	L_arrondi = Vad_L_add(L_var1, (int32_t)0x00008000);
	var_out = Vad_extract_h(L_arrondi);
	return(var_out);
}


int32_t Vad_L_mac(int32_t L_var3, int16_t var1, int16_t var2)
{
	int32_t L_var_out;
	int32_t L_produit;

	L_produit = Vad_L_mult(var1, var2);
	L_var_out = Vad_L_add(L_var3, L_produit);
	return(L_var_out);
}

int32_t Vad_L_msu(int32_t L_var3, int16_t var1, int16_t var2)
{
	int32_t L_var_out;
	int32_t L_produit;

	L_produit = Vad_L_mult(var1, var2);
	L_var_out = Vad_L_sub(L_var3, L_produit);
	return(L_var_out);
}


int32_t Vad_L_add(int32_t L_var1, int32_t L_var2)
{
	int32_t L_var_out;

	L_var_out = L_var1 + L_var2;

	if (((L_var1 ^ L_var2) & VAD_MIN_32) == 0)
	{
		if ((L_var_out ^ L_var1) & VAD_MIN_32)
		{
			L_var_out = (L_var1 < 0) ? VAD_MIN_32 : VAD_MAX_32;
			Vad_Overflow = 1;
		}
	}
	return(L_var_out);
}

int32_t Vad_L_sub(int32_t L_var1, int32_t L_var2)
{
	int32_t L_var_out;

	L_var_out = L_var1 - L_var2;

	if (((L_var1 ^ L_var2) & VAD_MIN_32) != 0)
	{
		if ((L_var_out ^ L_var1) & VAD_MIN_32)
		{
			L_var_out = (L_var1 < 0L) ? VAD_MIN_32 : VAD_MAX_32;
			Vad_Overflow = 1;
		}
	}
	return(L_var_out);
}



int32_t Vad_L_negate(int32_t L_var1)
{
	int32_t L_var_out;

	L_var_out = (L_var1 == VAD_MIN_32) ? VAD_MAX_32 : -L_var1;
	return(L_var_out);
}


int16_t Vad_mult_r(int16_t var1, int16_t var2)
{
	int16_t var_out;
	int32_t L_produit_arr;

	L_produit_arr = (int32_t)var1 * (int32_t)var2; /* product */
	L_produit_arr += (int32_t)0x00004000;        /* Vad_round */
	L_produit_arr &= (int32_t)0xffff8000L;
	L_produit_arr >>= 15;                        /* shift */

	if (L_produit_arr & (int32_t)0x00010000L)   /* sign extend when necessary */
	{
		L_produit_arr |= (int32_t)0xffff0000L;
	}

	var_out = Vad_sature(L_produit_arr);
	return(var_out);
}


int32_t Vad_L_shl(int32_t L_var1, int16_t var2)
{
	int32_t L_var_out;

	/* initialization used only to suppress Microsoft Visual C++ warnings */
	L_var_out = 0L;

	if (var2 <= 0)
	{
		L_var_out = Vad_L_shr(L_var1, -var2);
	}
	else
	{
		for (; var2>0; var2--)
		{
			if (L_var1 > (int32_t)0X3fffffffL)
			{
				Vad_Overflow = 1;
				L_var_out = VAD_MAX_32;
				break;
			}
			else
			{
				if (L_var1 < (int32_t)0xc0000000L)
				{
					Vad_Overflow = 1;
					L_var_out = VAD_MIN_32;
					break;
				}
			}
			L_var1 *= 2;
			L_var_out = L_var1;
		}
	}
	return(L_var_out);
}


int32_t Vad_L_shr(int32_t L_var1, int16_t var2)
{
	int32_t L_var_out;

	if (var2 < 0)
	{
		L_var_out = Vad_L_shl(L_var1, -var2);
	}
	else
	{
		if (var2 >= 31)
		{
			L_var_out = (L_var1 < 0L) ? -1 : 0;
		}
		else
		{
			if (L_var1<0)
			{
				L_var_out = ~((~L_var1) >> var2);
			}
			else
			{
				L_var_out = L_var1 >> var2;
			}
		}
	}
	return(L_var_out);
}


int32_t Vad_L_deposit_h(int16_t var1)
{
	int32_t L_var_out;

	L_var_out = (int32_t)var1 << 16;
	return(L_var_out);
}


int32_t Vad_L_deposit_l(int16_t var1)
{
	int32_t L_var_out;

	L_var_out = (int32_t)var1;
	return(L_var_out);
}


int32_t Vad_L_abs(int32_t L_var1)
{
	int32_t L_var_out;

	if (L_var1 == VAD_MIN_32)
	{
		L_var_out = VAD_MAX_32;
	}
	else
	{
		if (L_var1 < 0)
		{
			L_var_out = -L_var1;
		}
		else
		{
			L_var_out = L_var1;
		}
	}

	return(L_var_out);
}

int16_t Vad_norm_s(int16_t var1)
{
	int16_t var_out;

	if (var1 == 0)
	{
		var_out = 0;
	}
	else
	{
		if (var1 == (int16_t)0xffff)
		{
			var_out = 15;
		}
		else
		{
			if (var1 < 0)
			{
				var1 = ~var1;
			}

			for (var_out = 0; var1 < 0x4000; var_out++)
			{
				var1 <<= 1;
			}
		}
	}

	return(var_out);
}


int16_t Vad_div_s(int16_t var1, int16_t var2)
{
	int16_t var_out = 0;
	int16_t iteration;
	int32_t L_num;
	int32_t L_denom;

	if ((var1 > var2) || (var1 < 0) || (var2 < 0))
	{
		printf("Division Error var1=%d  var2=%d\n", var1, var2);
		exit(0);
	}

	if (var2 == 0)
	{
		printf("Division by 0, Fatal error \n");
		exit(0);
	}

	if (var1 == 0)
	{
		var_out = 0;
	}
	else
	{
		if (var1 == var2)
		{
			var_out = VAD_MAX_16;
		}
		else
		{
			L_num = Vad_L_deposit_l(var1);
			L_denom = Vad_L_deposit_l(var2);

			for (iteration = 0; iteration<15; iteration++)
			{
				var_out <<= 1;
				L_num <<= 1;

				if (L_num >= L_denom)
				{
					L_num = Vad_L_sub(L_num, L_denom);
					var_out = Vad_add(var_out, 1);
				}
			}
		}
	}

	return(var_out);
}


int16_t Vad_norm_l(int32_t L_var1)
{
	int16_t var_out;

	if (L_var1 == 0)
	{
		var_out = 0;
	}
	else
	{
		if (L_var1 == (int32_t)0xffffffffL)
		{
			var_out = 31;
		}
		else
		{
			if (L_var1 < 0)
			{
				L_var1 = ~L_var1;
			}

			for (var_out = 0; L_var1 < (int32_t)0x40000000L; var_out++)
			{
				L_var1 <<= 1;
			}
		}
	}

	return(var_out);
}


void Vad_L_Extract(int32_t L_32, int16_t *hi, int16_t *lo)
{
	*hi = Vad_extract_h(L_32);
	*lo = Vad_extract_l(Vad_L_msu(Vad_L_shr(L_32, 1), *hi, 16384));  /* lo = L_32>>1   */
	return;
}

int32_t Vad_L_Comp(int16_t hi, int16_t lo)
{
	int32_t L_32;

	L_32 = Vad_L_deposit_h(hi);
	return(Vad_L_mac(L_32, lo, 1));          /* = hi<<16 + lo<<1 */
}


int32_t Vad_Mpy_32(int16_t hi1, int16_t lo1, int16_t hi2, int16_t lo2)
{
	int32_t L_32;

	L_32 = Vad_L_mult(hi1, hi2);
	L_32 = Vad_L_mac(L_32, Vad_mult(hi1, lo2), 1);
	L_32 = Vad_L_mac(L_32, Vad_mult(lo1, hi2), 1);

	return(L_32);
}


int32_t Vad_Mpy_32_16(int16_t hi, int16_t lo, int16_t n)
{
	int32_t L_32;

	L_32 = Vad_L_mult(hi, n);
	L_32 = Vad_L_mac(L_32, Vad_mult(lo, n), 1);

	return(L_32);
}


int32_t Vad_Div_32(int32_t L_num, int16_t denom_hi, int16_t denom_lo)
{
	int16_t approx, hi, lo, n_hi, n_lo;
	int32_t L_32;


	/* First approximation: 1 / L_denom = 1/denom_hi */

	approx = Vad_div_s((int16_t)0x3fff, denom_hi);    /* result in Q14 */
	/* Note: 3fff = 0.5 in Q15 */

	/* 1/L_denom = approx * (2.0 - L_denom * approx) */

	L_32 = Vad_Mpy_32_16(denom_hi, denom_lo, approx); /* result in Q30 */


	L_32 = Vad_L_sub((int32_t)0x7fffffffL, L_32);      /* result in Q30 */

	Vad_L_Extract(L_32, &hi, &lo);

	L_32 = Vad_Mpy_32_16(hi, lo, approx);             /* = 1/L_denom in Q29 */

	/* L_num * (1/L_denom) */

	Vad_L_Extract(L_32, &hi, &lo);
	Vad_L_Extract(L_num, &n_hi, &n_lo);
	L_32 = Vad_Mpy_32(n_hi, n_lo, hi, lo);            /* result in Q29   */
	L_32 = Vad_L_shl(L_32, 2);                        /* From Q29 to Q31 */

	return(L_32);
}

/*___________________________________________________________________________
|                                                                           |
|   Function Name : Vad_Log2()                                                  |
|                                                                           |
|       Compute Vad_Log2(L_x).                                                  |
|       L_x is positive.                                                    |
|                                                                           |
|       if L_x is negative or zero, result is 0.                            |
|---------------------------------------------------------------------------|
|  Algorithm:                                                               |
|                                                                           |
|   The function Vad_Log2(L_x) is approximated by a vad_table and linear            |
|   interpolation.                                                          |
|                                                                           |
|   1- Normalization of L_x.                                                |
|   2- exponent = 30-exponent                                               |
|   3- i = bit25-b31 of L_x,    32 <= i <= 63  ->because of normalization.  |
|   4- a = bit10-b24                                                        |
|   5- i -=32                                                               |
|   6- fraction = vad_tablog[i]<<16 - (vad_tablog[i] - vad_tablog[i+1]) * a * 2            |
|___________________________________________________________________________|
*/
/*-----------------------------------------------------*
| Table for routine Vad_Log2().                           |
-----------------------------------------------------*/

static int16_t vad_tablog[33] = {
	0, 1455, 2866, 4236, 5568, 6863, 8124, 9352, 10549, 11716,
	12855, 13967, 15054, 16117, 17156, 18172, 19167, 20142, 21097, 22033,
	22951, 23852, 24735, 25603, 26455, 27291, 28113, 28922, 29716, 30497,
	31266, 32023, 32767 };

void Vad_Log2(
	int32_t L_x,       /* (i) Q0 : input value                                 */
	int16_t *exponent, /* (o) Q0 : Integer part of Vad_Log2.   (range: 0<=val<=30) */
	int16_t *fraction  /* (o) Q15: Fractional  part of Vad_Log2. (range: 0<=val<1) */
	)
{
	int16_t exp, i, a, tmp;
	int32_t L_y;

	if (L_x <= (int32_t)0)
	{
		*exponent = 0;
		*fraction = 0;
		return;
	}

	exp = Vad_norm_l(L_x);
	L_x = Vad_L_shl(L_x, exp);               /* L_x is normalized */

	*exponent = Vad_sub(30, exp);

	L_x = Vad_L_shr(L_x, 9);
	i = Vad_extract_h(L_x);                 /* Extract b25-b31 */
	L_x = Vad_L_shr(L_x, 1);
	a = Vad_extract_l(L_x);                 /* Extract b10-b24 of fraction */
	a = a & (int16_t)0x7fff;

	i = Vad_sub(i, 32);

	L_y = Vad_L_deposit_h(vad_tablog[i]);          /* vad_tablog[i] << 16        */
	tmp = Vad_sub(vad_tablog[i], vad_tablog[i + 1]);      /* vad_tablog[i] - vad_tablog[i+1] */
	L_y = Vad_L_msu(L_y, tmp, a);             /* L_y -= tmp*a*2        */

	*fraction = Vad_extract_h(L_y);

	return;
}









/* Hamming_cos window for LPC analysis.           */
/*   Create with function ham_cos(window,200,40)  */
static int16_t vad_hamwindow[VAD_L_WINDOW] = {
	2621, 2623, 2629, 2638, 2651, 2668, 2689, 2713, 2741, 2772,
	2808, 2847, 2890, 2936, 2986, 3040, 3097, 3158, 3223, 3291,
	3363, 3438, 3517, 3599, 3685, 3774, 3867, 3963, 4063, 4166,
	4272, 4382, 4495, 4611, 4731, 4853, 4979, 5108, 5240, 5376,
	5514, 5655, 5800, 5947, 6097, 6250, 6406, 6565, 6726, 6890,
	7057, 7227, 7399, 7573, 7750, 7930, 8112, 8296, 8483, 8672,
	8863, 9057, 9252, 9450, 9650, 9852, 10055, 10261, 10468, 10677,
	10888, 11101, 11315, 11531, 11748, 11967, 12187, 12409, 12632, 12856,
	13082, 13308, 13536, 13764, 13994, 14225, 14456, 14688, 14921, 15155,
	15389, 15624, 15859, 16095, 16331, 16568, 16805, 17042, 17279, 17516,
	17754, 17991, 18228, 18465, 18702, 18939, 19175, 19411, 19647, 19882,
	20117, 20350, 20584, 20816, 21048, 21279, 21509, 21738, 21967, 22194,
	22420, 22644, 22868, 23090, 23311, 23531, 23749, 23965, 24181, 24394,
	24606, 24816, 25024, 25231, 25435, 25638, 25839, 26037, 26234, 26428,
	26621, 26811, 26999, 27184, 27368, 27548, 27727, 27903, 28076, 28247,
	28415, 28581, 28743, 28903, 29061, 29215, 29367, 29515, 29661, 29804,
	29944, 30081, 30214, 30345, 30472, 30597, 30718, 30836, 30950, 31062,
	31170, 31274, 31376, 31474, 31568, 31659, 31747, 31831, 31911, 31988,
	32062, 32132, 32198, 32261, 32320, 32376, 32428, 32476, 32521, 32561,
	32599, 32632, 32662, 32688, 32711, 32729, 32744, 32755, 32763, 32767,
	32767, 32741, 32665, 32537, 32359, 32129, 31850, 31521, 31143, 30716,
	30242, 29720, 29151, 28538, 27879, 27177, 26433, 25647, 24821, 23957,
	23055, 22117, 21145, 20139, 19102, 18036, 16941, 15820, 14674, 13505,
	12315, 11106, 9879, 8637, 7381, 6114, 4838, 3554, 2264, 971 };



/*-----------------------------------------------------*
| Table of Vad_Lag_window for autocorrelation.            |
| noise floor = 1.0001   = (0.9999  on r[1] ..r[10])  |
| Bandwidth expansion = 60 Hz                         |
|                                                     |
| Special double precision format. See "oper_32b.c"   |
|                                                     |
| lag_wind[0] =  1.00000000    (not stored)           |
| lag_wind[1] =  0.99879038                           |
| lag_wind[2] =  0.99546897                           |
| lag_wind[3] =  0.98995781                           |
| lag_wind[4] =  0.98229337                           |
| lag_wind[5] =  0.97252619                           |
| lag_wind[6] =  0.96072036                           |
| lag_wind[7] =  0.94695264                           |
| lag_wind[8] =  0.93131179                           |
| lag_wind[9] =  0.91389757                           |
| lag_wind[10]=  0.89481968                           |
| lag_wind[11]=  0.87419660                           |
| lag_wind[12]=  0.85215437                           |
-----------------------------------------------------*/

static int16_t vad_lag_h[VAD_M + 2] = { 32728, 32619, 32438, 32187, 31867, 31480, 31029, 30517, 29946, 29321, 28645, 27923 };

static int16_t vad_lag_l[VAD_M + 2] = { 11904, 17280, 30720, 25856, 24192, 28992, 24384, 7360, 19520, 14784, 22092, 12924 };

/*-----------------------------------------------------*
| Tables for function Lsf_lsp() and Vad_Lsp_lsf()         |
-----------------------------------------------------*/

/* vad_table of cos(x) in Q15 */

static int16_t vad_table[65] = {
	32767, 32729, 32610, 32413, 32138, 31786, 31357, 30853,
	30274, 29622, 28899, 28106, 27246, 26320, 25330, 24279,
	23170, 22006, 20788, 19520, 18205, 16846, 15447, 14010,
	12540, 11039, 9512, 7962, 6393, 4808, 3212, 1608,
	0, -1608, -3212, -4808, -6393, -7962, -9512, -11039,
	-12540, -14010, -15447, -16846, -18205, -19520, -20788, -22006,
	-23170, -24279, -25330, -26320, -27246, -28106, -28899, -29622,
	-30274, -30853, -31357, -31786, -32138, -32413, -32610, -32729,
	-32768L };

/* vad_slope in Q12 used to compute y = acos(x) */

static int16_t vad_slope[64] = {
	-26887, -8812, -5323, -3813, -2979, -2444, -2081, -1811,
	-1608, -1450, -1322, -1219, -1132, -1059, -998, -946,
	-901, -861, -827, -797, -772, -750, -730, -713,
	-699, -687, -677, -668, -662, -657, -654, -652,
	-652, -654, -657, -662, -668, -677, -687, -699,
	-713, -730, -750, -772, -797, -827, -861, -901,
	-946, -998, -1059, -1132, -1219, -1322, -1450, -1608,
	-1811, -2081, -2444, -2979, -3813, -5323, -8812, -26887 };


/*-------------------------------------------------------------*
*  Table for az_lsf()                                         *
*                                                             *
* Vector vad_grid[] is in Q15                                     *
*                                                             *
* vad_grid[0] = 1.0;                                              *
* vad_grid[grid_points+1] = -1.0;                                 *
* for (i = 1; i < grid_points; i++)                           *
*   vad_grid[i] = cos((6.283185307*i)/(2.0*grid_points));         *
*                                                             *
*-------------------------------------------------------------*/

static int16_t vad_grid[VAD_GRID_POINTS + 1] = {
	32760, 32723, 32588, 32364, 32051, 31651,
	31164, 30591, 29935, 29196, 28377, 27481,
	26509, 25465, 24351, 23170, 21926, 20621,
	19260, 17846, 16384, 14876, 13327, 11743,
	10125, 8480, 6812, 5126, 3425, 1714,
	0, -1714, -3425, -5126, -6812, -8480,
	-10125, -11743, -13327, -14876, -16384, -17846,
	-19260, -20621, -21926, -23170, -24351, -25465,
	-26509, -27481, -28377, -29196, -29935, -30591,
	-31164, -31651, -32051, -32364, -32588, -32723,
	-32760 };


void Vad_Autocorr(int16_t x[],      /* (i)    : Input signal                      */
	int16_t m,        /* (i)    : LPC order                         */
	int16_t r_h[],    /* (o)    : Autocorrelations  (msb)           */
	int16_t r_l[],    /* (o)    : Autocorrelations  (lsb)           */
	int16_t *exp_R0
	)
{
	int16_t i, j, norm;
	int16_t y[VAD_L_WINDOW];
	int32_t sum;

	extern int16_t Vad_Overflow;

	/* Windowing of signal */

	for (i = 0; i<VAD_L_WINDOW; i++)
	{
		y[i] = Vad_mult_r(x[i], vad_hamwindow[i]);
	}

	/* Compute r[0] and test for Vad_Overflow */

	/* should exp_R0 be initialized to 2 ??? */
	*exp_R0 = 1;

	do {
		Vad_Overflow = 0;
		sum = 1;                   /* Avoid case of all zeros */
		for (i = 0; i<VAD_L_WINDOW; i++)
		{
			sum = Vad_L_mac(sum, y[i], y[i]);
		}

		/* If Vad_Overflow divide y[] by 4 */

		if (Vad_Overflow != 0)
		{
			for (i = 0; i<VAD_L_WINDOW; i++)
			{
				y[i] = Vad_shr(y[i], 2);
			}
			*exp_R0 = Vad_add((*exp_R0), 4);
			Vad_Overflow = 1;
		}
	} while (Vad_Overflow != 0);

	/* Normalization of r[0] */

	norm = Vad_norm_l(sum);
	sum = Vad_L_shl(sum, norm);
	Vad_L_Extract(sum, &r_h[0], &r_l[0]);     /* Put in DPF format (see oper_32b) */
	*exp_R0 = Vad_sub(*exp_R0, norm);

	/* r[1] to r[m] */

	for (i = 1; i <= m; i++)
	{
		sum = 0;
		for (j = 0; j<VAD_L_WINDOW - i; j++)
		{
			sum = Vad_L_mac(sum, y[j], y[j + i]);
		}

		sum = Vad_L_shl(sum, norm);
		Vad_L_Extract(sum, &r_h[i], &r_l[i]);
	}
	return;
}


/*-------------------------------------------------------*
* Function Vad_Lag_window()                                 *
*                                                       *
* Vad_Lag_window on autocorrelations.                       *
*                                                       *
* r[i] *= lag_wind[i]                                   *
*                                                       *
*  r[i] and lag_wind[i] are in special double precision.*
*  See "oper_32b.c" for the format                      *
*                                                       *
*-------------------------------------------------------*/

void Vad_Lag_window(
	int16_t m,         /* (i)     : LPC order                        */
	int16_t r_h[],     /* (i/o)   : Autocorrelations  (msb)          */
	int16_t r_l[]      /* (i/o)   : Autocorrelations  (lsb)          */
	)
{
	int16_t i;
	int32_t x;

	for (i = 1; i <= m; i++)
	{
		x = Vad_Mpy_32(r_h[i], r_l[i], vad_lag_h[i - 1], vad_lag_l[i - 1]);
		Vad_L_Extract(x, &r_h[i], &r_l[i]);
	}
	return;
}


/*___________________________________________________________________________
|                                                                           |
|      Vad_Levinson-DURBIN algorithm in double precision                        |
|      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~                        |
|---------------------------------------------------------------------------|
|                                                                           |
| Algorithm                                                                 |
|                                                                           |
|       R[i]    autocorrelations.                                           |
|       A[i]    filter coefficients.                                        |
|       K       reflection coefficients.                                    |
|       Alpha   prediction gain.                                            |
|                                                                           |
|       Initialization:                                                     |
|               A[0] = 1                                                    |
|               K    = -R[1]/R[0]                                           |
|               A[1] = K                                                    |
|               Alpha = R[0] * (1-K**2]                                     |
|                                                                           |
|       Do for  i = 2 to VAD_M                                                  |
|                                                                           |
|            S =  SUM ( R[j]*A[i-j] ,j=1,i-1 ) +  R[i]                      |
|                                                                           |
|            K = -S / Alpha                                                 |
|                                                                           |
|            An[j] = A[j] + K*A[i-j]   for j=1 to i-1                       |
|                                      where   An[i] = new A[i]             |
|            An[i]=K                                                        |
|                                                                           |
|            Alpha=Alpha * (1-K**2)                                         |
|                                                                           |
|       END                                                                 |
|                                                                           |
| Remarks on the dynamics of the calculations.                              |
|                                                                           |
|       The numbers used are in double precision in the following format :  |
|       A = AH <<16 + AL<<1.  AH and AL are 16 bit signed integers.         |
|       Since the LSB's also contain a sign bit, this format does not       |
|       correspond to standard 32 bit integers.  We use this format since   |
|       it allows fast execution of multiplications and divisions.          |
|                                                                           |
|       "DPF" will refer to this special format in the following text.      |
|       See oper_32b.c                                                      |
|                                                                           |
|       The R[i] were normalized in routine AUTO (hence, R[i] < 1.0).       |
|       The K[i] and Alpha are theoretically < 1.0.                         |
|       The A[i], for a sampling frequency of 8 kHz, are in practice        |
|       always inferior to 16.0.                                            |
|                                                                           |
|       These characteristics allow straigthforward fixed-point             |
|       implementation.  We choose to represent the parameters as           |
|       follows :                                                           |
|                                                                           |
|               R[i]    Q31   +- .99..                                      |
|               K[i]    Q31   +- .99..                                      |
|               Alpha   Normalized -> mantissa in Q31 plus exponent         |
|               A[i]    Q27   +- 15.999..                                   |
|                                                                           |
|       The additions are performed in 32 bit.  For the summation used      |
|       to calculate the K[i], we multiply numbers in Q31 by numbers        |
|       in Q27, with the result of the multiplications in Q27,              |
|       resulting in a dynamic of +- 16.  This is sufficient to avoid       |
|       Vad_Overflow, since the final result of the summation is                |
|       necessarily < 1.0 as both the K[i] and Alpha are                    |
|       theoretically < 1.0.                                                |
|___________________________________________________________________________|
*/


/* Last A(z) for case of unstable filter */

static int16_t vad_old_A[VAD_M + 1] = { 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int16_t vad_old_rc[2] = { 0, 0 };

void Vad_Levinson(
	int16_t Rh[],      /* (i)     : Rh[VAD_M+1] Vector of autocorrelations (msb) */
	int16_t Rl[],      /* (i)     : Rl[VAD_M+1] Vector of autocorrelations (lsb) */
	int16_t A[],       /* (o) Q12 : A[VAD_M]    LPC coefficients  (m = 10)       */
	int16_t rc[],      /* (o) Q15 : rc[VAD_M]   Reflection coefficients.         */
	int16_t *Err       /* (o)     : Residual energy                          */
	)
{
	int16_t i, j;
	int16_t hi, lo;
	int16_t Kh, Kl;                /* reflection coefficient; hi and lo           */
	int16_t alp_h, alp_l, alp_exp; /* Prediction gain; hi lo and exponent         */
	int16_t Ah[VAD_M + 1], Al[VAD_M + 1];      /* LPC coef. in double prec.                   */
	int16_t Anh[VAD_M + 1], Anl[VAD_M + 1];    /* LPC coef.for next iteration in double prec. */
	int32_t t0, t1, t2;            /* temporary variable                          */


	/* K = A[1] = -R[1] / R[0] */

	t1 = Vad_L_Comp(Rh[1], Rl[1]);           /* R[1] in Q31      */
	t2 = Vad_L_abs(t1);                      /* abs R[1]         */
	t0 = Vad_Div_32(t2, Rh[0], Rl[0]);       /* R[1]/R[0] in Q31 */
	if (t1 > 0) t0 = Vad_L_negate(t0);          /* -R[1]/R[0]       */
	Vad_L_Extract(t0, &Kh, &Kl);              /* K in DPF         */
	rc[0] = Kh;
	t0 = Vad_L_shr(t0, 4);                     /* A[1] in Q27      */
	Vad_L_Extract(t0, &Ah[1], &Al[1]);        /* A[1] in DPF      */

	/*  Alpha = R[0] * (1-K**2) */

	t0 = Vad_Mpy_32(Kh, Kl, Kh, Kl);          /* K*K      in Q31 */
	t0 = Vad_L_abs(t0);                       /* Some case <0 !! */
	t0 = Vad_L_sub((int32_t)0x7fffffffL, t0); /* 1 - K*K  in Q31 */
	Vad_L_Extract(t0, &hi, &lo);              /* DPF format      */
	t0 = Vad_Mpy_32(Rh[0], Rl[0], hi, lo);    /* Alpha in Q31    */

	/* Normalize Alpha */

	alp_exp = Vad_norm_l(t0);
	t0 = Vad_L_shl(t0, alp_exp);
	Vad_L_Extract(t0, &alp_h, &alp_l);         /* DPF format    */

	/*--------------------------------------*
	* ITERATIONS  I=2 to VAD_M                 *
	*--------------------------------------*/

	for (i = 2; i <= VAD_M; i++)
	{

		/* t0 = SUM ( R[j]*A[i-j] ,j=1,i-1 ) +  R[i] */

		t0 = 0;
		for (j = 1; j<i; j++)
		{
			t0 = Vad_L_add(t0, Vad_Mpy_32(Rh[j], Rl[j], Ah[i - j], Al[i - j]));
		}

		t0 = Vad_L_shl(t0, 4);                  /* result in Q27 -> convert to Q31 */
		/* No Vad_Overflow possible            */
		t1 = Vad_L_Comp(Rh[i], Rl[i]);
		t0 = Vad_L_add(t0, t1);                /* Vad_add R[i] in Q31                 */

		/* K = -t0 / Alpha */

		t1 = Vad_L_abs(t0);
		t2 = Vad_Div_32(t1, alp_h, alp_l);     /* abs(t0)/Alpha                   */
		if (t0 > 0) t2 = Vad_L_negate(t2);       /* K =-t0/Alpha                    */
		t2 = Vad_L_shl(t2, alp_exp);           /* denormalize; compare to Alpha   */
		Vad_L_Extract(t2, &Kh, &Kl);           /* K in DPF                        */
		rc[i - 1] = Kh;

		/* Test for unstable filter. If unstable keep old A(z) */

		if (Vad_sub(Vad_abs_s(Kh), 32750) > 0)
		{
			for (j = 0; j <= VAD_M; j++)
			{
				A[j] = vad_old_A[j];
			}
			rc[0] = vad_old_rc[0];        /* only two rc coefficients are needed */
			rc[1] = vad_old_rc[1];
			return;
		}

		/*------------------------------------------*
		*  Compute new LPC coeff. -> An[i]         *
		*  An[j]= A[j] + K*A[i-j]     , j=1 to i-1 *
		*  An[i]= K                                *
		*------------------------------------------*/


		for (j = 1; j<i; j++)
		{
			t0 = Vad_Mpy_32(Kh, Kl, Ah[i - j], Al[i - j]);
			t0 = Vad_L_add(t0, Vad_L_Comp(Ah[j], Al[j]));
			Vad_L_Extract(t0, &Anh[j], &Anl[j]);
		}
		t2 = Vad_L_shr(t2, 4);                  /* t2 = K in Q31 ->convert to Q27  */
		Vad_L_Extract(t2, &Anh[i], &Anl[i]);    /* An[i] in Q27                    */

		/*  Alpha = Alpha * (1-K**2) */

		t0 = Vad_Mpy_32(Kh, Kl, Kh, Kl);          /* K*K      in Q31 */
		t0 = Vad_L_abs(t0);                       /* Some case <0 !! */
		t0 = Vad_L_sub((int32_t)0x7fffffffL, t0); /* 1 - K*K  in Q31 */
		Vad_L_Extract(t0, &hi, &lo);              /* DPF format      */
		t0 = Vad_Mpy_32(alp_h, alp_l, hi, lo);   /* Alpha in Q31    */

		/* Normalize Alpha */

		j = Vad_norm_l(t0);
		t0 = Vad_L_shl(t0, j);
		Vad_L_Extract(t0, &alp_h, &alp_l);         /* DPF format    */
		alp_exp = Vad_add(alp_exp, j);             /* Vad_add normalization to alp_exp */

		/* A[j] = An[j] */

		for (j = 1; j <= i; j++)
		{
			Ah[j] = Anh[j];
			Al[j] = Anl[j];
		}
	}

	*Err = Vad_shr(alp_h, alp_exp);

	/* Truncate A[i] in Q27 to Q12 with rounding */

	A[0] = 4096;
	for (i = 1; i <= VAD_M; i++)
	{
		t0 = Vad_L_Comp(Ah[i], Al[i]);
		vad_old_A[i] = A[i] = Vad_round(Vad_L_shl(t0, 1));
	}
	vad_old_rc[0] = rc[0];
	vad_old_rc[1] = rc[1];

	return;
}




/*-------------------------------------------------------------*
*  procedure Vad_Az_lsp:                                          *
*            ~~~~~~                                           *
*   Compute the LSPs from  the LPC coefficients  (order=10)   *
*-------------------------------------------------------------*/

/* local function */

static int16_t Vad_Chebps_11(int16_t x, int16_t f[], int16_t n);
static int16_t Vad_Chebps_10(int16_t x, int16_t f[], int16_t n);

void Vad_Az_lsp(
	int16_t a[],        /* (i) Q12 : predictor coefficients              */
	int16_t lsp[],      /* (o) Q15 : line spectral pairs                 */
	int16_t old_lsp[]   /* (i)     : old lsp[] (in case not found 10 roots) */
	)
{
	int16_t i, j, nf, ip;
	int16_t xlow, ylow, xhigh, yhigh, xmid, ymid, xint;
	int16_t x, y, sign, exp;
	int16_t *coef;
	int16_t f1[VAD_M / 2 + 1], f2[VAD_M / 2 + 1];
	int32_t t0, L_temp;
	int16_t  ovf_coef;
	int16_t(*pChebps)(int16_t x, int16_t f[], int16_t n);

	/*-------------------------------------------------------------*
	*  find the sum and diff. pol. F1(z) and F2(z)                *
	*    F1(z) <--- F1(z)/(1+z**-1) & F2(z) <--- F2(z)/(1-z**-1)  *
	*                                                             *
	* f1[0] = 1.0;                                                *
	* f2[0] = 1.0;                                                *
	*                                                             *
	* for (i = 0; i< VAD_NC; i++)                                     *
	* {                                                           *
	*   f1[i+1] = a[i+1] + a[VAD_M-i] - f1[i] ;                       *
	*   f2[i+1] = a[i+1] - a[VAD_M-i] + f2[i] ;                       *
	* }                                                           *
	*-------------------------------------------------------------*/

	ovf_coef = 0;
	pChebps = Vad_Chebps_11;

	f1[0] = 2048;          /* f1[0] = 1.0 is in Q11 */
	f2[0] = 2048;          /* f2[0] = 1.0 is in Q11 */

	for (i = 0; i< VAD_NC; i++)
	{
		Vad_Overflow = 0;
		t0 = Vad_L_mult(a[i + 1], 16384);          /* x = (a[i+1] + a[VAD_M-i]) >> 1        */
		t0 = Vad_L_mac(t0, a[VAD_M - i], 16384);       /*    -> From Q12 to Q11             */
		x = Vad_extract_h(t0);
		if (Vad_Overflow)
		{
			ovf_coef = 1;
		}

		Vad_Overflow = 0;
		f1[i + 1] = Vad_sub(x, f1[i]);    /* f1[i+1] = a[i+1] + a[VAD_M-i] - f1[i] */
		if (Vad_Overflow)
		{
			ovf_coef = 1;
		}

		Vad_Overflow = 0;
		t0 = Vad_L_mult(a[i + 1], 16384);          /* x = (a[i+1] - a[VAD_M-i]) >> 1        */
		t0 = Vad_L_msu(t0, a[VAD_M - i], 16384);       /*    -> From Q12 to Q11             */
		x = Vad_extract_h(t0);
		if (Vad_Overflow)
		{
			ovf_coef = 1;
		}

		Vad_Overflow = 0;
		f2[i + 1] = Vad_add(x, f2[i]);    /* f2[i+1] = a[i+1] - a[VAD_M-i] + f2[i] */
		if (Vad_Overflow)
		{
			ovf_coef = 1;
		}
	}

	if (ovf_coef)
	{
		pChebps = Vad_Chebps_10;

		f1[0] = 1024;          /* f1[0] = 1.0 is in Q10 */
		f2[0] = 1024;          /* f2[0] = 1.0 is in Q10 */

		for (i = 0; i< VAD_NC; i++)
		{
			t0 = Vad_L_mult(a[i + 1], 8192);          /* x = (a[i+1] + a[VAD_M-i]) >> 1        */
			t0 = Vad_L_mac(t0, a[VAD_M - i], 8192);       /*    -> From Q11 to Q10             */
			x = Vad_extract_h(t0);
			f1[i + 1] = Vad_sub(x, f1[i]);    /* f1[i+1] = a[i+1] + a[VAD_M-i] - f1[i] */

			t0 = Vad_L_mult(a[i + 1], 8192);          /* x = (a[i+1] - a[VAD_M-i]) >> 1        */
			t0 = Vad_L_msu(t0, a[VAD_M - i], 8192);       /*    -> From Q11 to Q10             */
			x = Vad_extract_h(t0);
			f2[i + 1] = Vad_add(x, f2[i]);    /* f2[i+1] = a[i+1] - a[VAD_M-i] + f2[i] */
		}
	}

	/*-------------------------------------------------------------*
	* find the LSPs using the Chebichev pol. evaluation           *
	*-------------------------------------------------------------*/

	nf = 0;          /* number of found frequencies */
	ip = 0;          /* indicator for f1 or f2      */

	coef = f1;

	xlow = vad_grid[0];
	ylow = (*pChebps)(xlow, coef, VAD_NC);

	j = 0;
	while ((nf < VAD_M) && (j < VAD_GRID_POINTS))
	{
		j = Vad_add(j, 1);
		xhigh = xlow;
		yhigh = ylow;
		xlow = vad_grid[j];
		ylow = (*pChebps)(xlow, coef, VAD_NC);

		L_temp = Vad_L_mult(ylow, yhigh);
		if (L_temp <= (int32_t)0)
		{

			/* divide 4 times the interval */

			for (i = 0; i < 4; i++)
			{
				xmid = Vad_add(Vad_shr(xlow, 1), Vad_shr(xhigh, 1)); /* xmid = (xlow + xhigh)/2 */

				ymid = (*pChebps)(xmid, coef, VAD_NC);

				L_temp = Vad_L_mult(ylow, ymid);
				if (L_temp <= (int32_t)0)
				{
					yhigh = ymid;
					xhigh = xmid;
				}
				else
				{
					ylow = ymid;
					xlow = xmid;
				}
			}

			/*-------------------------------------------------------------*
			* Linear interpolation                                        *
			*    xint = xlow - ylow*(xhigh-xlow)/(yhigh-ylow);            *
			*-------------------------------------------------------------*/

			x = Vad_sub(xhigh, xlow);
			y = Vad_sub(yhigh, ylow);

			if (y == 0)
			{
				xint = xlow;
			}
			else
			{
				sign = y;
				y = Vad_abs_s(y);
				exp = Vad_norm_s(y);
				y = Vad_shl(y, exp);
				y = Vad_div_s((int16_t)16383, y);
				t0 = Vad_L_mult(x, y);
				t0 = Vad_L_shr(t0, Vad_sub(20, exp));
				y = Vad_extract_l(t0);            /* y= (xhigh-xlow)/(yhigh-ylow) in Q11 */

				if (sign < 0) y = Vad_negate(y);

				t0 = Vad_L_mult(ylow, y);                  /* result in Q26 */
				t0 = Vad_L_shr(t0, 11);                    /* result in Q15 */
				xint = Vad_sub(xlow, Vad_extract_l(t0));         /* xint = xlow - ylow*y */
			}

			lsp[nf] = xint;
			xlow = xint;
			nf = Vad_add(nf, 1);

			if (ip == 0)
			{
				ip = 1;
				coef = f2;
			}
			else
			{
				ip = 0;
				coef = f1;
			}
			ylow = (*pChebps)(xlow, coef, VAD_NC);

		}
	}

	/* Check if VAD_M roots found */

	if (Vad_sub(nf, VAD_M) < 0)
	{
		for (i = 0; i<VAD_M; i++)
		{
			lsp[i] = old_lsp[i];
		}

		/* printf("\n !!Not 10 roots found in Vad_Az_lsp()!!!\n"); */
	}

	return;
}

/*--------------------------------------------------------------*
* function  Vad_Chebps_11, Vad_Chebps_10:                              *
*           ~~~~~~~~~~~~~~~~~~~~                               *
*    Evaluates the Chebichev polynomial series                 *
*--------------------------------------------------------------*
*                                                              *
*  The polynomial order is                                     *
*     n = VAD_M/2   (VAD_M is the prediction order)                    *
*  The polynomial is given by                                  *
*    C(x) = T_n(x) + f(1)T_n-1(x) + ... +f(n-1)T_1(x) + f(n)/2 *
* Arguments:                                                   *
*  x:     input value of evaluation; x = cos(frequency) in Q15 *
*  f[]:   coefficients of the pol.                             *
*                         in Q11(Vad_Chebps_11), in Q10(Vad_Chebps_10) *
*  n:     order of the pol.                                    *
*                                                              *
* The value of C(x) is returned. (Saturated to +-1.99 in Q14)  *
*                                                              *
*--------------------------------------------------------------*/
static int16_t Vad_Chebps_11(int16_t x, int16_t f[], int16_t n)
{
	int16_t i, cheb;
	int16_t b0_h, b0_l, b1_h, b1_l, b2_h, b2_l;
	int32_t t0;

	/* Note: All computation are done in Q24. */

	b2_h = 256;                    /* b2 = 1.0 in Q24 DPF */
	b2_l = 0;

	t0 = Vad_L_mult(x, 512);                  /* 2*x in Q24          */
	t0 = Vad_L_mac(t0, f[1], 4096);           /* + f[1] in Q24       */
	Vad_L_Extract(t0, &b1_h, &b1_l);          /* b1 = 2*x + f[1]     */

	for (i = 2; i<n; i++)
	{
		t0 = Vad_Mpy_32_16(b1_h, b1_l, x);      /* t0 = 2.0*x*b1              */
		t0 = Vad_L_shl(t0, 1);
		t0 = Vad_L_mac(t0, b2_h, (int16_t)-32768L); /* t0 = 2.0*x*b1 - b2         */
		t0 = Vad_L_msu(t0, b2_l, 1);
		t0 = Vad_L_mac(t0, f[i], 4096);         /* t0 = 2.0*x*b1 - b2 + f[i]; */

		Vad_L_Extract(t0, &b0_h, &b0_l);        /* b0 = 2.0*x*b1 - b2 + f[i]; */

		b2_l = b1_l;                 /* b2 = b1; */
		b2_h = b1_h;
		b1_l = b0_l;                 /* b1 = b0; */
		b1_h = b0_h;
	}

	t0 = Vad_Mpy_32_16(b1_h, b1_l, x);        /* t0 = x*b1;              */
	t0 = Vad_L_mac(t0, b2_h, (int16_t)-32768L);  /* t0 = x*b1 - b2          */
	t0 = Vad_L_msu(t0, b2_l, 1);
	t0 = Vad_L_mac(t0, f[i], 2048);           /* t0 = x*b1 - b2 + f[i]/2 */

	t0 = Vad_L_shl(t0, 6);                    /* Q24 to Q30 with saturation */
	cheb = Vad_extract_h(t0);                 /* Result in Q14              */


	return(cheb);
}


static int16_t Vad_Chebps_10(int16_t x, int16_t f[], int16_t n)
{
	int16_t i, cheb;
	int16_t b0_h, b0_l, b1_h, b1_l, b2_h, b2_l;
	int32_t t0;

	/* Note: All computation are done in Q23. */

	b2_h = 128;                    /* b2 = 1.0 in Q23 DPF */
	b2_l = 0;

	t0 = Vad_L_mult(x, 256);                  /* 2*x in Q23          */
	t0 = Vad_L_mac(t0, f[1], 4096);           /* + f[1] in Q23       */
	Vad_L_Extract(t0, &b1_h, &b1_l);          /* b1 = 2*x + f[1]     */

	for (i = 2; i<n; i++)
	{
		t0 = Vad_Mpy_32_16(b1_h, b1_l, x);      /* t0 = 2.0*x*b1              */
		t0 = Vad_L_shl(t0, 1);
		t0 = Vad_L_mac(t0, b2_h, (int16_t)-32768L); /* t0 = 2.0*x*b1 - b2         */
		t0 = Vad_L_msu(t0, b2_l, 1);
		t0 = Vad_L_mac(t0, f[i], 4096);         /* t0 = 2.0*x*b1 - b2 + f[i]; */

		Vad_L_Extract(t0, &b0_h, &b0_l);        /* b0 = 2.0*x*b1 - b2 + f[i]; */

		b2_l = b1_l;                 /* b2 = b1; */
		b2_h = b1_h;
		b1_l = b0_l;                 /* b1 = b0; */
		b1_h = b0_h;
	}

	t0 = Vad_Mpy_32_16(b1_h, b1_l, x);        /* t0 = x*b1;              */
	t0 = Vad_L_mac(t0, b2_h, (int16_t)-32768L);  /* t0 = x*b1 - b2          */
	t0 = Vad_L_msu(t0, b2_l, 1);
	t0 = Vad_L_mac(t0, f[i], 2048);           /* t0 = x*b1 - b2 + f[i]/2 */

	t0 = Vad_L_shl(t0, 7);                    /* Q23 to Q30 with saturation */
	cheb = Vad_extract_h(t0);                 /* Result in Q14              */


	return(cheb);
}


void Vad_Lsp_lsf(
	int16_t lsp[],    /* (i) Q15 : lsp[m] (range: -1<=val<1)                */
	int16_t lsf[],    /* (o) Q15 : lsf[m] normalized (range: 0.0<=val<=0.5) */
	int16_t m         /* (i)     : LPC order                                */
	)
{
	int16_t i, ind, tmp;
	int32_t L_tmp;

	ind = 63;    /* begin at end of vad_table -1 */

	for (i = m - (int16_t)1; i >= 0; i--)
	{
		/* find value in vad_table that is just greater than lsp[i] */
		while (Vad_sub(vad_table[ind], lsp[i]) < 0)
		{
			ind = Vad_sub(ind, 1);
		}

		/* acos(lsp[i])= ind*256 + ( ( lsp[i]-vad_table[ind] ) * vad_slope[ind] )/4096 */

		L_tmp = Vad_L_mult(Vad_sub(lsp[i], vad_table[ind]), vad_slope[ind]);
		tmp = Vad_round(Vad_L_shl(L_tmp, 3));     /*(lsp[i]-vad_table[ind])*vad_slope[ind])>>12*/
		lsf[i] = Vad_add(tmp, Vad_shl(ind, 8));
	}
	return;
}


void Vad_Set_zero(int16_t x[], int16_t L)
{
	int16_t i;

	for (i = 0; i < L; i++)
	{
		x[i] = 0;
	}
	return;
}

void Vad_Copy(int16_t x[], int16_t y[], int16_t L)
{
	int16_t i;

	for (i = 0; i < L; i++)
	{
		y[i] = x[i];
	}
	return;
}

void Vad_Main_Process(VadInstT *self, int16_t frame, int16_t vad_enable, short *vadDecision)
{
	int16_t r_l[VAD_NP + 1], r_h[VAD_NP + 1];
	int16_t rc[VAD_M];
	int16_t A_t[(VAD_MP1)* 2];
	int16_t lsp_new[VAD_M];
	int16_t lsf_new[VAD_M];
	int16_t temp;
	int16_t rh_nbe[VAD_MP1];
	int16_t exp_R0, Vad;

	/* LP analysis */
	Vad_Autocorr(self->p_window, VAD_NP, r_h, r_l, &exp_R0);
	Vad_Copy(r_h, rh_nbe, VAD_MP1);
	Vad_Lag_window(VAD_NP, r_h, r_l);
	Vad_Levinson(r_h, r_l, &A_t[VAD_MP1], rc, &temp);
	Vad_Az_lsp(&A_t[VAD_MP1], lsp_new, self->lsp_old);

	/* ------ VAD ------- */
	Vad_Lsp_lsf(lsp_new, lsf_new, VAD_M);
	Vad_New(self, rc[1], lsf_new, r_h, r_l, exp_R0, self->p_window, self->frame, self->pastVad, self->ppastVad, &Vad);
	*vadDecision = Vad;

	self->ppastVad = self->pastVad;
	self->pastVad = Vad;

	Vad_Copy(&(self->old_speech[VAD_L_FRAME]), &(self->old_speech[0]), VAD_L_TOTAL - VAD_L_FRAME);

	return;
}

int WebRtcVad_CalcVad8khz_G729B(VadInstT* self, short* audio_frame, int frame_length)
{
	short vad = 0;
	short vad_enable = 1;
	short i = 0, j = 0;
	//	VadInstT* self = (VadInstT*) handle;

	// 	if (handle == NULL) 
	// 	{
	// 		return -1;
	// 	}

	if (audio_frame == NULL)
	{
		return -1;
	}

	if (frame_length != 80 && frame_length != 160 && frame_length != 240)
	{
		return -1;
	}


	for (i = 0, j = 0; i < frame_length; i = i + 80, j++)
	{
		if (self->frame == 32767)
		{
			self->frame = 256;
		}
		else
		{
			self->frame++;
		}

		Vad_HignPass_Process(self, audio_frame + j * 80, self->new_speech, VAD_L_FRAME);

		Vad_Main_Process(self, self->frame, vad_enable, &self->VadDecision[j]);

		vad = vad | self->VadDecision[j];
	}

	return vad;
}



/* VAD constants */
static int16_t vad_lbf_corr[VAD_NP + 1] = { 7869, 7011, 4838, 2299, 321, -660, -782, -484, -164, 3, 39, 21, 4 };
static int16_t vad_shift_fx[33] = { 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 5, 0 };
static int16_t vad_factor_fx[33] = { 32767, 16913, 17476, 18079, 18725, 19418, 20165, 20972,
21845, 22795, 23831, 24966, 26214, 27594, 29127, 30840, 32767, 17476, 18725, 20165,
21845, 23831, 26214, 29127, 32767, 18725, 21845, 26214, 32767, 21845, 32767, 32767, 0 };

//dSLE: differential low band energy 
//dSE: differential full band energy
//SD: differential spectral distortion
//dSZC: differential zero crossing rate
static int16_t Vad_MakeDec(int16_t dSLE, int16_t dSE, int16_t SD, int16_t dSZC)
{
	int32_t acc0;

	/* SD vs dSZC */
	acc0 = Vad_L_mult(dSZC, -14680);          /* Q15*Q23*2 = Q39 */
	acc0 = Vad_L_mac(acc0, 8192, -28521);     /* Q15*Q23*2 = Q39 */
	acc0 = Vad_L_shr(acc0, 8);                /* Q39 -> Q31 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(SD));
	if (acc0 > 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSZC, 19065);           /* Q15*Q22*2 = Q38 */
	acc0 = Vad_L_mac(acc0, 8192, -19446);     /* Q15*Q22*2 = Q38 */
	acc0 = Vad_L_shr(acc0, 7);                /* Q38 -> Q31 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(SD));
	if (acc0 > 0)
	{
		return(VAD_VOICE);
	}

	/* dSE vs dSZC */
	acc0 = Vad_L_mult(dSZC, 20480);           /* Q15*Q13*2 = Q29 */
	acc0 = Vad_L_mac(acc0, 8192, 16384);      /* Q13*Q15*2 = Q29 */
	acc0 = Vad_L_shr(acc0, 2);                /* Q29 -> Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSE));
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSZC, -16384);          /* Q15*Q13*2 = Q29 */
	acc0 = Vad_L_mac(acc0, 8192, 19660);      /* Q13*Q15*2 = Q29 */
	acc0 = Vad_L_shr(acc0, 2);                /* Q29 -> Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSE));
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSE, 32767);            /* Q11*Q15*2 = Q27 */
	acc0 = Vad_L_mac(acc0, 1024, 30802);      /* Q10*Q16*2 = Q27 */
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	/* dSE vs SD */
	acc0 = Vad_L_mult(SD, -28160);            /* Q15*Q5*2 = Q22 */
	acc0 = Vad_L_mac(acc0, 64, 19988);        /* Q6*Q14*2 = Q22 */
	acc0 = Vad_L_mac(acc0, dSE, 512);         /* Q11*Q9*2 = Q22 */
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(SD, 32767);             /* Q15*Q15*2 = Q31 */
	acc0 = Vad_L_mac(acc0, 32, -30199);       /* Q5*Q25*2 = Q31 */
	if (acc0 > 0)
	{
		return(VAD_VOICE);
	}

	/* dSLE vs dSZC */
	acc0 = Vad_L_mult(dSZC, -20480);          /* Q15*Q13*2 = Q29 */
	acc0 = Vad_L_mac(acc0, 8192, 22938);      /* Q13*Q15*2 = Q29 */
	acc0 = Vad_L_shr(acc0, 2);                /* Q29 -> Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSE));
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSZC, 23831);           /* Q15*Q13*2 = Q29 */
	acc0 = Vad_L_mac(acc0, 4096, 31576);      /* Q12*Q16*2 = Q29 */
	acc0 = Vad_L_shr(acc0, 2);                /* Q29 -> Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSE));
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSE, 32767);            /* Q11*Q15*2 = Q27 */
	acc0 = Vad_L_mac(acc0, 2048, 17367);      /* Q11*Q15*2 = Q27 */
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	/* dSLE vs SD */
	acc0 = Vad_L_mult(SD, -22400);            /* Q15*Q4*2 = Q20 */
	acc0 = Vad_L_mac(acc0, 32, 25395);        /* Q5*Q14*2 = Q20 */
	acc0 = Vad_L_mac(acc0, dSLE, 256);        /* Q11*Q8*2 = Q20 */
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	/* dSLE vs dSE */
	acc0 = Vad_L_mult(dSE, -30427);           /* Q11*Q15*2 = Q27 */
	acc0 = Vad_L_mac(acc0, 256, -29959);      /* Q8*Q18*2 = Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSLE));
	if (acc0 > 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSE, -23406);           /* Q11*Q15*2 = Q27 */
	acc0 = Vad_L_mac(acc0, 512, 28087);       /* Q19*Q17*2 = Q27 */
	acc0 = Vad_L_add(acc0, Vad_L_deposit_h(dSLE));
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	acc0 = Vad_L_mult(dSE, 24576);            /* Q11*Q14*2 = Q26 */
	acc0 = Vad_L_mac(acc0, 1024, 29491);      /* Q10*Q15*2 = Q26 */
	acc0 = Vad_L_mac(acc0, dSLE, 16384);      /* Q11*Q14*2 = Q26 */
	if (acc0 < 0)
	{
		return(VAD_VOICE);
	}

	return (VAD_NOISE);
}


void Vad_New(VadInstT *self, int16_t rc, int16_t *lsf, int16_t *r_h, int16_t *r_l, int16_t exp_R0,
	int16_t *sigpp, int16_t frm_count, int16_t prev_marker, int16_t pprev_marker, int16_t *marker)
{
	int32_t acc0;
	int16_t i, j, exp, frac;
	int16_t ENERGY, ENERGY_low, SD, ZC, dSE, dSLE, dSZC;
	int16_t COEF, C_COEF, COEFZC, C_COEFZC, COEFSD, C_COEFSD;

	/* compute the frame energy */
	acc0 = Vad_L_Comp(r_h[0], r_l[0]);
	Vad_Log2(acc0, &exp, &frac);
	acc0 = Vad_Mpy_32_16(exp, frac, 9864);
	i = Vad_sub(exp_R0, 1);
	i = Vad_sub(i, 1);
	acc0 = Vad_L_mac(acc0, 9864, i);
	acc0 = Vad_L_shl(acc0, 11);
	ENERGY = Vad_extract_h(acc0);
	ENERGY = Vad_sub(ENERGY, 4875);

	/* compute the low band energy */
	acc0 = 0;
	for (i = 1; i <= VAD_NP; i++)
	{
		acc0 = Vad_L_mac(acc0, r_h[i], vad_lbf_corr[i]);
	}
	acc0 = Vad_L_shl(acc0, 1);
	acc0 = Vad_L_mac(acc0, r_h[0], vad_lbf_corr[0]);
	Vad_Log2(acc0, &exp, &frac);
	acc0 = Vad_Mpy_32_16(exp, frac, 9864);
	i = Vad_sub(exp_R0, 1);
	i = Vad_sub(i, 1);
	acc0 = Vad_L_mac(acc0, 9864, i);
	acc0 = Vad_L_shl(acc0, 11);
	ENERGY_low = Vad_extract_h(acc0);
	ENERGY_low = Vad_sub(ENERGY_low, 4875);

	/* compute SD */
	acc0 = 0;
	for (i = 0; i<VAD_M; i++)
	{
		j = Vad_sub(lsf[i], self->MeanLSF[i]);
		acc0 = Vad_L_mac(acc0, j, j);
	}
	SD = Vad_extract_h(acc0);      /* Q15 */

	/* compute # zero crossing */
	ZC = 0;
	for (i = VAD_ZC_START + 1; i <= VAD_ZC_END; i++)
	{
		if (Vad_mult(sigpp[i - 1], sigpp[i]) < 0)
		{
			ZC = Vad_add(ZC, 410);     /* Q15 */
		}
	}

	/* Initialize and update Mins */
	if (Vad_sub(frm_count, 129) < 0)
	{
		if (Vad_sub(ENERGY, self->Min) < 0)
		{
			self->Min = ENERGY;
			self->Prev_Min = ENERGY;
		}

		if ((frm_count & 0x0007) == 0)
		{
			i = Vad_sub(Vad_shr(frm_count, 3), 1);
			self->Min_buffer[i] = self->Min;
			self->Min = VAD_MAX_16;
		}
	}

	if ((frm_count & 0x0007) == 0)
	{
		self->Prev_Min = self->Min_buffer[0];
		for (i = 1; i<16; i++)
		{
			if (Vad_sub(self->Min_buffer[i], self->Prev_Min) < 0)
			{
				self->Prev_Min = self->Min_buffer[i];
			}
		}
	}

	if (Vad_sub(frm_count, 129) >= 0)
	{
		if (((frm_count & 0x0007) ^ (0x0001)) == 0)
		{
			self->Min = self->Prev_Min;
			self->Next_Min = VAD_MAX_16;
		}

		if (Vad_sub(ENERGY, self->Min) < 0)
		{
			self->Min = ENERGY;
		}

		if (Vad_sub(ENERGY, self->Next_Min) < 0)
		{
			self->Next_Min = ENERGY;
		}

		if ((frm_count & 0x0007) == 0)
		{
			for (i = 0; i<15; i++)
			{
				self->Min_buffer[i] = self->Min_buffer[i + 1];
			}
			self->Min_buffer[15] = self->Next_Min;
			self->Prev_Min = self->Min_buffer[0];
			for (i = 1; i<16; i++)
			{
				if (Vad_sub(self->Min_buffer[i], self->Prev_Min) < 0)
				{
					self->Prev_Min = self->Min_buffer[i];
				}
			}
		}
	}

	if (Vad_sub(frm_count, VAD_INIT_FRAME) <= 0)
	{
		if (Vad_sub(ENERGY, 3072) < 0)
		{
			*marker = VAD_NOISE;
			self->less_count++;
		}
		else
		{
			*marker = VAD_VOICE;
			acc0 = Vad_L_deposit_h(self->MeanE);
			acc0 = Vad_L_mac(acc0, ENERGY, 1024);
			self->MeanE = Vad_extract_h(acc0);
			acc0 = Vad_L_deposit_h(self->MeanSZC);
			acc0 = Vad_L_mac(acc0, ZC, 1024);
			self->MeanSZC = Vad_extract_h(acc0);
			for (i = 0; i<VAD_M; i++)
			{
				acc0 = Vad_L_deposit_h(self->MeanLSF[i]);
				acc0 = Vad_L_mac(acc0, lsf[i], 1024);
				self->MeanLSF[i] = Vad_extract_h(acc0);
			}
		}
	}

	if (Vad_sub(frm_count, VAD_INIT_FRAME) >= 0)
	{
		if (Vad_sub(frm_count, VAD_INIT_FRAME) == 0)
		{
			acc0 = Vad_L_mult(self->MeanE, vad_factor_fx[self->less_count]);
			acc0 = Vad_L_shl(acc0, vad_shift_fx[self->less_count]);
			self->MeanE = Vad_extract_h(acc0);

			acc0 = Vad_L_mult(self->MeanSZC, vad_factor_fx[self->less_count]);
			acc0 = Vad_L_shl(acc0, vad_shift_fx[self->less_count]);
			self->MeanSZC = Vad_extract_h(acc0);

			for (i = 0; i<VAD_M; i++)
			{
				acc0 = Vad_L_mult(self->MeanLSF[i], vad_factor_fx[self->less_count]);
				acc0 = Vad_L_shl(acc0, vad_shift_fx[self->less_count]);
				self->MeanLSF[i] = Vad_extract_h(acc0);
			}

			self->MeanSE = Vad_sub(self->MeanE, 2048);   /* Q11 */
			self->MeanSLE = Vad_sub(self->MeanE, 2458);  /* Q11 */
		}

		dSE = Vad_sub(self->MeanSE, ENERGY);
		dSLE = Vad_sub(self->MeanSLE, ENERGY_low);
		dSZC = Vad_sub(self->MeanSZC, ZC);

		if (Vad_sub(ENERGY, 3072) < 0)
		{
			*marker = VAD_NOISE;
		}
		else
		{
			*marker = Vad_MakeDec(dSLE, dSE, SD, dSZC);
		}

		self->v_flag = 0;
		if ((prev_marker == VAD_VOICE) && (*marker == VAD_NOISE) && (Vad_add(dSE, 410)<0) && (Vad_sub(ENERGY, 3072)>0))
		{
			*marker = VAD_VOICE;
			self->v_flag = 1;
		}

		if (self->flag == 1)
		{
			if ((pprev_marker == VAD_VOICE) && (prev_marker == VAD_VOICE) && (*marker == VAD_NOISE) && (Vad_sub(Vad_abs_s(Vad_sub(self->prev_energy, ENERGY)), 614) <= 0))
			{
				self->count_ext++;
				*marker = VAD_VOICE;
				self->v_flag = 1;
				if (Vad_sub(self->count_ext, 4) <= 0)
				{
					self->flag = 1;
				}
				else
				{
					self->count_ext = 0;
					self->flag = 0;
				}
			}
		}
		else
		{
			self->flag = 1;
		}

		if (*marker == VAD_NOISE)
		{
			self->count_sil++;
		}

		if ((*marker == VAD_VOICE) && (Vad_sub(self->count_sil, 10) > 0) && (Vad_sub(Vad_sub(ENERGY, self->prev_energy), 614) <= 0))
		{
			*marker = VAD_NOISE;
			self->count_sil = 0;
		}

		if (*marker == VAD_VOICE)
		{
			self->count_sil = 0;
		}

		if ((Vad_sub(Vad_sub(ENERGY, 614), self->MeanSE)<0) && (Vad_sub(frm_count, 128) > 0) && (!self->v_flag) && (Vad_sub(rc, 19661) < 0))
		{
			*marker = VAD_NOISE;
		}

		if ((Vad_sub(Vad_sub(ENERGY, 614), self->MeanSE) < 0) && (Vad_sub(rc, 24576) < 0) && (Vad_sub(SD, 83) < 0))
		{
			self->count_update++;
			if (Vad_sub(self->count_update, VAD_INIT_COUNT) < 0)
			{
				COEF = 24576;
				C_COEF = 8192;
				COEFZC = 26214;
				C_COEFZC = 6554;
				COEFSD = 19661;
				C_COEFSD = 13017;
			}
			else
			{
				if (Vad_sub(self->count_update, VAD_INIT_COUNT + 10) < 0)
				{
					COEF = 31130;
					C_COEF = 1638;
					COEFZC = 30147;
					C_COEFZC = 2621;
					COEFSD = 21299;
					C_COEFSD = 11469;
				}
				else
				{
					if (Vad_sub(self->count_update, VAD_INIT_COUNT + 20) < 0)
					{
						COEF = 31785;
						C_COEF = 983;
						COEFZC = 30802;
						C_COEFZC = 1966;
						COEFSD = 22938;
						C_COEFSD = 9830;
					}
					else
					{
						if (Vad_sub(self->count_update, VAD_INIT_COUNT + 30) < 0)
						{
							COEF = 32440;
							C_COEF = 328;
							COEFZC = 31457;
							C_COEFZC = 1311;
							COEFSD = 24576;
							C_COEFSD = 8192;
						}
						else
						{
							if (Vad_sub(self->count_update, VAD_INIT_COUNT + 40) < 0)
							{
								COEF = 32604;
								C_COEF = 164;
								COEFZC = 32440;
								C_COEFZC = 328;
								COEFSD = 24576;
								C_COEFSD = 8192;
							}
							else
							{
								COEF = 32604;
								C_COEF = 164;
								COEFZC = 32702;
								C_COEFZC = 66;
								COEFSD = 24576;
								C_COEFSD = 8192;
							}
						}
					}
				}
			}


			/* compute MeanSE */
			acc0 = Vad_L_mult(COEF, self->MeanSE);
			acc0 = Vad_L_mac(acc0, C_COEF, ENERGY);
			self->MeanSE = Vad_extract_h(acc0);

			/* compute MeanSLE */
			acc0 = Vad_L_mult(COEF, self->MeanSLE);
			acc0 = Vad_L_mac(acc0, C_COEF, ENERGY_low);
			self->MeanSLE = Vad_extract_h(acc0);

			/* compute MeanSZC */
			acc0 = Vad_L_mult(COEFZC, self->MeanSZC);
			acc0 = Vad_L_mac(acc0, C_COEFZC, ZC);
			self->MeanSZC = Vad_extract_h(acc0);

			/* compute MeanLSF */
			for (i = 0; i<VAD_M; i++)
			{
				acc0 = Vad_L_mult(COEFSD, self->MeanLSF[i]);
				acc0 = Vad_L_mac(acc0, C_COEFSD, lsf[i]);
				self->MeanLSF[i] = Vad_extract_h(acc0);
			}
		}

		if ((Vad_sub(frm_count, 128) > 0) && (((Vad_sub(self->MeanSE, self->Min) < 0) && (Vad_sub(SD, 83) < 0)) || (Vad_sub(self->MeanSE, self->Min) > 2048)))
		{
			self->MeanSE = self->Min;
			self->count_update = 0;
		}
	}

	self->prev_energy = ENERGY;
}



/* filter coefficients (fc = 140 Hz, coeff. b[] is divided by 2) */

static int16_t vad_b140[3] = { 1899, -3798, 1899 };      /* 1/2 in Q12 */
static int16_t vad_a140[3] = { 4096, 7807, -3733 };      /* Q12 */

/*------------------------------------------------------------------------*
* 2nd order high pass filter with cut off frequency at 140 Hz.           *
* Designed with SPPACK efi command -40 dB att, 0.25 ri.                  *
*                                                                        *
* Algorithm:                                                             *
*                                                                        *
*  y[i] = b[0]*x[i]/2 + b[1]*x[i-1]/2 + b[2]*x[i-2]/2                    *
*                     + a[1]*y[i-1]   + a[2]*y[i-2];                     *
*                                                                        *
*     b[3] = {0.92727435E+00, -0.18544941E+01, 0.92727435E+00};          *
*     a[3] = {0.10000000E+01, 0.19059465E+01, -0.91140240E+00};          *
*                                                                        *
*  Input are divided by two in the filtering process.                    *
*-----------------------------------------------------------------------*/


void Vad_HignPass_Process(VadInstT *self,
	int16_t *signal_in,    /* input/output signal */
	int16_t *signal_out,    /* input/output signal */
	int16_t lg)          /* length of signal    */
{
	int16_t i, x2;
	int32_t L_tmp;

	for (i = 0; i<lg; i++)
	{
		x2 = self->x1;
		self->x1 = self->x0;
		self->x0 = signal_in[i];

		/*  y[i] = b[0]*x[i]/2 + b[1]*x[i-1]/2 + vad_b140[2]*x[i-2]/2  */
		/*                     + a[1]*y[i-1] + a[2] * y[i-2];      */

		L_tmp = Vad_Mpy_32_16(self->y1_hi, self->y1_lo, vad_a140[1]);
		L_tmp = Vad_L_add(L_tmp, Vad_Mpy_32_16(self->y2_hi, self->y2_lo, vad_a140[2]));
		L_tmp = Vad_L_mac(L_tmp, self->x0, vad_b140[0]);
		L_tmp = Vad_L_mac(L_tmp, self->x1, vad_b140[1]);
		L_tmp = Vad_L_mac(L_tmp, x2, vad_b140[2]);
		L_tmp = Vad_L_shl(L_tmp, 3);      /* Q28 --> Q31 (Q12 --> Q15) */
		signal_out[i] = Vad_round(L_tmp);

		self->y2_hi = self->y1_hi;
		self->y2_lo = self->y1_lo;
		Vad_L_Extract(L_tmp, &(self->y1_hi), &(self->y1_lo));
	}
	return;
}

#endif
