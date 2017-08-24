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
LOCAL_MODULE := libsystem_wrappers
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
	-DDL_ARM_NEON_OPTIONAL
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
LOCAL_CFLAGS += \
	-DWEBRTC_ARCH_ARM64_NEON \
	-DDL_ARM_NEON
endif
	
LOCAL_CXXFLAGS := \
	-std=gnu++11
	
LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/../..

LOCAL_SRC_FILES := \
	$(LOCAL_PATH)/source/aligned_malloc.cc \
	$(LOCAL_PATH)/source/cpu_features.cc \
	$(LOCAL_PATH)/source/cpu_features_android.c \
	$(LOCAL_PATH)/source/critical_section.cc \
	$(LOCAL_PATH)/source/critical_section_posix.cc \
	$(LOCAL_PATH)/source/logging.cc \
	$(LOCAL_PATH)/source/trace_impl.cc \
	$(LOCAL_PATH)/source/trace_posix.cc \
	
LOCAL_WHOLE_STATIC_LIBRARIES := \
    libstlport_static

LOCAL_STATIC_LIBRARIES := cpufeatures -llog -ldl -lc -lz -lm

include $(BUILD_STATIC_LIBRARY)
$(call import-module,android/cpufeatures)