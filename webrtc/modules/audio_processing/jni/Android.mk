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
LOCAL_MODULE := libaudio_processing
LOCAL_MODULE_TAGS := optional
LOCAL_CPP_EXTENSION := .cc .cpp

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
	-DWEBRTC_POSIX \
    -DWEBRTC_LINUX \
	-DWEBRTC_ANDROID \
	-DWEBRTC_NS_FIXED \
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
    $(LOCAL_PATH)/../../.. \
	$(LOCAL_PATH)/../../../webrtc \
	$(LOCAL_PATH)/../../../webrtc/common_audio/vad/include \
	$(LOCAL_PATH)/../../../webrtc/common_audio/signal_processing/include \
	$(LOCAL_PATH)/../../../webrtc/common_audio/resampler/include

LOCAL_SRC_FILES := \
	$(LOCAL_PATH)/aec/aec_core.c \
	$(LOCAL_PATH)/aec/aec_rdft.c \
	$(LOCAL_PATH)/aec/aec_resampler.c \
	$(LOCAL_PATH)/aec/echo_cancellation.c \
	$(LOCAL_PATH)/aecm/aecm_core.c \
	$(LOCAL_PATH)/aecm/aecm_core_c.c \
	$(LOCAL_PATH)/aecm/echo_control_mobile.c \
	$(LOCAL_PATH)/agc/agc.cc \
	$(LOCAL_PATH)/agc/agc_audio_proc.cc \
	$(LOCAL_PATH)/agc/agc_manager_direct.cc \
	$(LOCAL_PATH)/agc/circular_buffer.cc \
	$(LOCAL_PATH)/agc/gmm.cc \
	$(LOCAL_PATH)/agc/histogram.cc \
	$(LOCAL_PATH)/agc/legacy/analog_agc.c \
	$(LOCAL_PATH)/agc/legacy/digital_agc.c \
	$(LOCAL_PATH)/agc/pitch_based_vad.cc \
	$(LOCAL_PATH)/agc/pitch_internal.cc \
	$(LOCAL_PATH)/agc/pole_zero_filter.cc \
	$(LOCAL_PATH)/agc/standalone_vad.cc \
	$(LOCAL_PATH)/agc/utility.cc \
	$(LOCAL_PATH)/beamformer/covariance_matrix_generator.cc \
	$(LOCAL_PATH)/beamformer/beamformer.cc \
	$(LOCAL_PATH)/transient/moving_moments.cc \
	$(LOCAL_PATH)/transient/transient_detector.cc \
	$(LOCAL_PATH)/transient/transient_suppressor.cc \
	$(LOCAL_PATH)/transient/wpd_node.cc \
	$(LOCAL_PATH)/transient/wpd_tree.cc \
	$(LOCAL_PATH)/utility/delay_estimator.c \
	$(LOCAL_PATH)/utility/delay_estimator_wrapper.c \
	$(LOCAL_PATH)/utility/fft4g.c \
	$(LOCAL_PATH)/ns/noise_suppression_x.c \
	$(LOCAL_PATH)/ns/nsx_core.c \
	$(LOCAL_PATH)/ns/nsx_core_c.c \
	$(LOCAL_PATH)/audio_buffer.cc \
	$(LOCAL_PATH)/audio_processing_impl.cc \
	$(LOCAL_PATH)/echo_cancellation_impl.cc \
	$(LOCAL_PATH)/echo_control_mobile_impl.cc \
	$(LOCAL_PATH)/gain_control_impl.cc \
	$(LOCAL_PATH)/high_pass_filter_impl.cc \
	$(LOCAL_PATH)/level_estimator_impl.cc \
	$(LOCAL_PATH)/noise_suppression_impl.cc \
	$(LOCAL_PATH)/processing_component.cc \
	$(LOCAL_PATH)/rms_level.cc \
	$(LOCAL_PATH)/splitting_filter.cc \
	$(LOCAL_PATH)/typing_detection.cc \
	$(LOCAL_PATH)/voice_detection_impl.cc
	
### neon
LOCAL_SRC_FILES += \
	$(LOCAL_PATH)/aec/aec_core_neon.c \
	$(LOCAL_PATH)/aec/aec_rdft_neon.c \
	$(LOCAL_PATH)/aecm/aecm_core_neon.c \
	$(LOCAL_PATH)/ns/nsx_core_neon.c
	
LOCAL_WHOLE_STATIC_LIBRARIES := \
    libstlport_static

LOCAL_STATIC_LIBRARIES += -llog -ldl -lc -lz -lm

include $(BUILD_STATIC_LIBRARY)
