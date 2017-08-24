# Copyright (c) 2017 The WebRTC project authors. All Rights Reserved.
#
# Use of this source code is governed by a BSD-style license
# that can be found in the LICENSE file in the root of the source
# tree. An additional intellectual property rights grant can be found
# in the file PATENTS.  All contributing project authors may
# be found in the AUTHORS file in the root of the source tree.

MY_MK_ROOT := $(call my-dir)
MY_UCS_ROOT := $(MY_MK_ROOT)/../..
MY_PROJECT_ROOT := $(MY_UCS_ROOT)/..

include $(MY_PROJECT_ROOT)/webrtc/system_wrappers/jni/Android.mk
include $(MY_PROJECT_ROOT)/webrtc/base/jni/Android.mk
include $(MY_PROJECT_ROOT)/webrtc/common_audio/jni/Android.mk
include $(MY_PROJECT_ROOT)/webrtc/modules/audio_processing/jni/Android.mk

include $(CLEAR_VARS)
LOCAL_PATH := $(MY_UCS_ROOT)
LOCAL_MODULE := libUCS_VQE
LOCAL_CPP_EXTENSION := .cc .cpp

# Flags passed to both C and C++ files.
LOCAL_CFLAGS := \
	-DWEBRTC_POSIX \
    -DWEBRTC_LINUX \
	-DWEBRTC_ANDROID \
	-D__STDC_CONSTANT_MACROS \
	-D__STDC_FORMAT_MACROS
	
LOCAL_CXXFLAGS := \
	-std=gnu++11
	
LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/.. \
    $(LOCAL_PATH)/include \
	$(LOCAL_PATH)/include/interface

LOCAL_SRC_FILES := \
	$(LOCAL_PATH)/src/ucs_trace.cpp \
	$(LOCAL_PATH)/src/ucs_vqe.cpp \
	$(LOCAL_PATH)/src/ucsvqe_jni.cpp
	
LOCAL_WHOLE_STATIC_LIBRARIES := \
    libstlport_static \
	librtc_base \
	libsystem_wrappers \
	libcommon_audio \
	libaudio_processing


LOCAL_LDLIBS += -llog -ldl -lc -lz -lm

include $(BUILD_SHARED_LIBRARY)
$(call import-module,android/cpufeatures)