# Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.

LOCAL_PATH := $(call my-dir)/../
include $(CLEAR_VARS)

LOCAL_MODULE_CLASS := STATIC_LIBRARIES
LOCAL_MODULE := libcommon_audio
LOCAL_MODULE_TAGS := optional
LOCAL_CPP_EXTENSION := .cc .cpp

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
	-DWEBRTC_POSIX \
    -DWEBRTC_LINUX \
	-DWEBRTC_ANDROID \
	-D__STDC_CONSTANT_MACROS \
	-D__STDC_FORMAT_MACROS

ifeq ($(TARGET_ARCH_ABI),$(filter $(TARGET_ARCH_ABI), armeabi-v7a armeabi))
LOCAL_CFLAGS += \
	-DWEBRTC_ARCH_ARM \
    -DWEBRTC_ARCH_ARM_V7 \
	-DWEBRTC_DETECT_ARM_NEON \
	-DDL_ARM_NEON_OPTIONAL \
	-mfloat-abi=softfp \
	-mfpu=neon
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
LOCAL_CFLAGS += \
	-DWEBRTC_ARCH_ARM64_NEON \
	-DDL_ARM_NEON
endif
	
LOCAL_CXXFLAGS := \
	-std=gnu++11
	
LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/../.. \
	$(LOCAL_PATH)/resampler/include \
	$(LOCAL_PATH)/signal_processing/include \
	$(LOCAL_PATH)/vad/include

LOCAL_SRC_FILES := \
	$(LOCAL_PATH)/audio_ring_buffer.cc \
	$(LOCAL_PATH)/audio_util.cc \
	$(LOCAL_PATH)/blocker.cc \
	$(LOCAL_PATH)/channel_buffer.cc \
	$(LOCAL_PATH)/fir_filter.cc \
	$(LOCAL_PATH)/ring_buffer.c \
	$(LOCAL_PATH)/window_generator.cc \
	$(LOCAL_PATH)/lapped_transform.cc \
	$(LOCAL_PATH)/real_fourier.cc \
	$(LOCAL_PATH)/resampler/push_resampler.cc \
	$(LOCAL_PATH)/resampler/push_sinc_resampler.cc \
	$(LOCAL_PATH)/resampler/resampler.cc \
	$(LOCAL_PATH)/resampler/sinc_resampler.cc \
	$(LOCAL_PATH)/vad/vad.cc \
	$(LOCAL_PATH)/vad/webrtc_vad.c \
	$(LOCAL_PATH)/vad/vad_core.c \
	$(LOCAL_PATH)/vad/vad_filterbank.c \
	$(LOCAL_PATH)/vad/vad_gmm.c \
	$(LOCAL_PATH)/vad/vad_sp.c \
	$(LOCAL_PATH)/signal_processing/auto_corr_to_refl_coef.c \
	$(LOCAL_PATH)/signal_processing/auto_correlation.c \
	$(LOCAL_PATH)/signal_processing/complex_fft.c \
	$(LOCAL_PATH)/signal_processing/copy_set_operations.c \
	$(LOCAL_PATH)/signal_processing/cross_correlation.c \
	$(LOCAL_PATH)/signal_processing/division_operations.c \
	$(LOCAL_PATH)/signal_processing/dot_product_with_scale.c \
	$(LOCAL_PATH)/signal_processing/downsample_fast.c \
	$(LOCAL_PATH)/signal_processing/energy.c \
	$(LOCAL_PATH)/signal_processing/filter_ar.c \
	$(LOCAL_PATH)/signal_processing/filter_ma_fast_q12.c \
	$(LOCAL_PATH)/signal_processing/get_hanning_window.c \
	$(LOCAL_PATH)/signal_processing/get_scaling_square.c \
	$(LOCAL_PATH)/signal_processing/ilbc_specific_functions.c \
	$(LOCAL_PATH)/signal_processing/levinson_durbin.c \
	$(LOCAL_PATH)/signal_processing/lpc_to_refl_coef.c \
	$(LOCAL_PATH)/signal_processing/min_max_operations.c \
	$(LOCAL_PATH)/signal_processing/randomization_functions.c \
	$(LOCAL_PATH)/signal_processing/refl_coef_to_lpc.c \
	$(LOCAL_PATH)/signal_processing/real_fft.c \
	$(LOCAL_PATH)/signal_processing/resample.c \
	$(LOCAL_PATH)/signal_processing/resample_48khz.c \
	$(LOCAL_PATH)/signal_processing/resample_by_2.c \
	$(LOCAL_PATH)/signal_processing/resample_by_2_internal.c \
	$(LOCAL_PATH)/signal_processing/resample_fractional.c \
	$(LOCAL_PATH)/signal_processing/spl_init.c \
	$(LOCAL_PATH)/signal_processing/spl_sqrt.c \
	$(LOCAL_PATH)/signal_processing/splitting_filter.c \
	$(LOCAL_PATH)/signal_processing/sqrt_of_one_minus_x_squared.c \
	$(LOCAL_PATH)/signal_processing/vector_scaling_operations.c

ifeq ($(TARGET_ARCH_ABI),$(filter $(TARGET_ARCH_ABI), armeabi-v7a armeabi))
LOCAL_SRC_FILES += \
	$(LOCAL_PATH)/signal_processing/complex_bit_reverse_arm.S \
	$(LOCAL_PATH)/signal_processing/spl_sqrt_floor_arm.S \
	$(LOCAL_PATH)/signal_processing/filter_ar_fast_q12_armv7.S
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
LOCAL_SRC_FILES += \
	$(LOCAL_PATH)/signal_processing/complex_bit_reverse.c \
	$(LOCAL_PATH)/signal_processing/spl_sqrt_floor.c \
	$(LOCAL_PATH)/signal_processing/filter_ar_fast_q12.c
endif

LOCAL_SRC_FILES += \
	$(LOCAL_PATH)/fir_filter_neon.cc \
	$(LOCAL_PATH)/resampler/sinc_resampler_neon.cc \
	$(LOCAL_PATH)/signal_processing/cross_correlation_neon.c \
	$(LOCAL_PATH)/signal_processing/downsample_fast_neon.c \
	$(LOCAL_PATH)/signal_processing/min_max_operations_neon.c

LOCAL_WHOLE_STATIC_LIBRARIES := \
    libstlport_static

#LOCAL_LDLIBS += -llog -ldl -lc -lz -lm
LOCAL_STATIC_LIBRARIES += -llog -ldl -lc -lz -lm

include $(BUILD_STATIC_LIBRARY)
