# This file is generated by gyp; do not edit.

TOOLSET := target
TARGET := common_audio
DEFS_Debug := \
	'-DV8_DEPRECATION_WARNINGS' \
	'-D_FILE_OFFSET_BITS=64' \
	'-DCHROMIUM_BUILD' \
	'-DCR_CLANG_REVISION=231191-2' \
	'-DTOOLKIT_VIEWS=1' \
	'-DUI_COMPOSITOR_IMAGE_TRANSPORT' \
	'-DUSE_AURA=1' \
	'-DUSE_ASH=1' \
	'-DUSE_PANGO=1' \
	'-DUSE_CAIRO=1' \
	'-DUSE_DEFAULT_RENDER_THEME=1' \
	'-DUSE_LIBJPEG_TURBO=1' \
	'-DUSE_X11=1' \
	'-DUSE_CLIPBOARD_AURAX11=1' \
	'-DENABLE_ONE_CLICK_SIGNIN' \
	'-DENABLE_PRE_SYNC_BACKUP' \
	'-DENABLE_REMOTING=1' \
	'-DENABLE_WEBRTC=1' \
	'-DENABLE_PEPPER_CDMS' \
	'-DENABLE_CONFIGURATION_POLICY' \
	'-DENABLE_NOTIFICATIONS' \
	'-DUSE_UDEV' \
	'-DDONT_EMBED_BUILD_METADATA' \
	'-DENABLE_TASK_MANAGER=1' \
	'-DENABLE_EXTENSIONS=1' \
	'-DENABLE_PLUGINS=1' \
	'-DENABLE_SESSION_SERVICE=1' \
	'-DENABLE_THEMES=1' \
	'-DENABLE_AUTOFILL_DIALOG=1' \
	'-DENABLE_BACKGROUND=1' \
	'-DENABLE_GOOGLE_NOW=1' \
	'-DCLD_VERSION=2' \
	'-DENABLE_PRINTING=1' \
	'-DENABLE_BASIC_PRINTING=1' \
	'-DENABLE_PRINT_PREVIEW=1' \
	'-DENABLE_SPELLCHECK=1' \
	'-DENABLE_CAPTIVE_PORTAL_DETECTION=1' \
	'-DENABLE_APP_LIST=1' \
	'-DENABLE_SETTINGS_APP=1' \
	'-DENABLE_SUPERVISED_USERS=1' \
	'-DENABLE_MDNS=1' \
	'-DENABLE_SERVICE_DISCOVERY=1' \
	'-DV8_USE_EXTERNAL_STARTUP_DATA' \
	'-DEXPAT_RELATIVE_PATH' \
	'-DWEBRTC_POSIX' \
	'-DWEBRTC_LINUX' \
	'-DWEBRTC_H264_BUILD' \
	'-DUSE_LIBPCI=1' \
	'-DUSE_GLIB=1' \
	'-DUSE_NSS=1' \
	'-D__STDC_CONSTANT_MACROS' \
	'-D__STDC_FORMAT_MACROS' \
	'-DDYNAMIC_ANNOTATIONS_ENABLED=1' \
	'-DWTF_USE_DYNAMIC_ANNOTATIONS=1' \
	'-D_DEBUG' \
	'-D_GLIBCXX_DEBUG=1'

# Flags passed to all source files.
CFLAGS_Debug := \
	-fstack-protector \
	--param=ssp-buffer-size=4 \
	-Werror \
	-pthread \
	-fno-strict-aliasing \
	-Wall \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-fvisibility=hidden \
	-pipe \
	-fPIC \
	-Wno-reserved-user-defined-literal \
	-B/home/lizhiyong/linux/trunk64/webrtc/src/third_party/binutils/Linux_x64/Release/bin \
	-Wno-char-subscripts \
	-Wno-unneeded-internal-declaration \
	-Wno-covered-switch-default \
	-Wno-c++11-narrowing \
	-Wno-deprecated-register \
	-Wno-inconsistent-missing-override \
	-Wno-unused-private-field \
	-Wno-unused-variable \
	-Wno-writable-strings \
	-Wno-unsequenced \
	-Wno-unused-const-variable \
	-Wextra \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-Wno-strict-overflow \
	-m64 \
	-march=x86-64 \
	-O0 \
	-g \
	-gdwarf-4 \
	-funwind-tables \
	-gsplit-dwarf \
	-Wno-undefined-bool-conversion \
	-Wno-tautological-undefined-compare

# Flags passed to only C files.
CFLAGS_C_Debug :=

# Flags passed to only C++ files.
CFLAGS_CC_Debug := \
	-fno-exceptions \
	-fno-rtti \
	-fno-threadsafe-statics \
	-fvisibility-inlines-hidden \
	-Wsign-compare \
	-std=gnu++11 \
	-Wnon-virtual-dtor \
	-Woverloaded-virtual

INCS_Debug := \
	-I$(obj)/gen \
	-I. \
	-Iwebrtc/common_audio/resampler/include \
	-Iwebrtc/common_audio/signal_processing/include \
	-Ichromium/src/third_party/openmax_dl

DEFS_Release := \
	'-DV8_DEPRECATION_WARNINGS' \
	'-D_FILE_OFFSET_BITS=64' \
	'-DCHROMIUM_BUILD' \
	'-DCR_CLANG_REVISION=231191-2' \
	'-DTOOLKIT_VIEWS=1' \
	'-DUI_COMPOSITOR_IMAGE_TRANSPORT' \
	'-DUSE_AURA=1' \
	'-DUSE_ASH=1' \
	'-DUSE_PANGO=1' \
	'-DUSE_CAIRO=1' \
	'-DUSE_DEFAULT_RENDER_THEME=1' \
	'-DUSE_LIBJPEG_TURBO=1' \
	'-DUSE_X11=1' \
	'-DUSE_CLIPBOARD_AURAX11=1' \
	'-DENABLE_ONE_CLICK_SIGNIN' \
	'-DENABLE_PRE_SYNC_BACKUP' \
	'-DENABLE_REMOTING=1' \
	'-DENABLE_WEBRTC=1' \
	'-DENABLE_PEPPER_CDMS' \
	'-DENABLE_CONFIGURATION_POLICY' \
	'-DENABLE_NOTIFICATIONS' \
	'-DUSE_UDEV' \
	'-DDONT_EMBED_BUILD_METADATA' \
	'-DENABLE_TASK_MANAGER=1' \
	'-DENABLE_EXTENSIONS=1' \
	'-DENABLE_PLUGINS=1' \
	'-DENABLE_SESSION_SERVICE=1' \
	'-DENABLE_THEMES=1' \
	'-DENABLE_AUTOFILL_DIALOG=1' \
	'-DENABLE_BACKGROUND=1' \
	'-DENABLE_GOOGLE_NOW=1' \
	'-DCLD_VERSION=2' \
	'-DENABLE_PRINTING=1' \
	'-DENABLE_BASIC_PRINTING=1' \
	'-DENABLE_PRINT_PREVIEW=1' \
	'-DENABLE_SPELLCHECK=1' \
	'-DENABLE_CAPTIVE_PORTAL_DETECTION=1' \
	'-DENABLE_APP_LIST=1' \
	'-DENABLE_SETTINGS_APP=1' \
	'-DENABLE_SUPERVISED_USERS=1' \
	'-DENABLE_MDNS=1' \
	'-DENABLE_SERVICE_DISCOVERY=1' \
	'-DV8_USE_EXTERNAL_STARTUP_DATA' \
	'-DEXPAT_RELATIVE_PATH' \
	'-DWEBRTC_POSIX' \
	'-DWEBRTC_LINUX' \
	'-DWEBRTC_H264_BUILD' \
	'-DUSE_LIBPCI=1' \
	'-DUSE_GLIB=1' \
	'-DUSE_NSS=1' \
	'-D__STDC_CONSTANT_MACROS' \
	'-D__STDC_FORMAT_MACROS' \
	'-DNDEBUG' \
	'-DNVALGRIND' \
	'-DDYNAMIC_ANNOTATIONS_ENABLED=0'

# Flags passed to all source files.
CFLAGS_Release := \
	-fstack-protector \
	--param=ssp-buffer-size=4 \
	-Werror \
	-pthread \
	-fno-strict-aliasing \
	-Wall \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-fvisibility=hidden \
	-pipe \
	-fPIC \
	-Wno-reserved-user-defined-literal \
	-B/home/lizhiyong/linux/trunk64/webrtc/src/third_party/binutils/Linux_x64/Release/bin \
	-Wno-char-subscripts \
	-Wno-unneeded-internal-declaration \
	-Wno-covered-switch-default \
	-Wno-c++11-narrowing \
	-Wno-deprecated-register \
	-Wno-inconsistent-missing-override \
	-Wno-unused-private-field \
	-Wno-unused-variable \
	-Wno-writable-strings \
	-Wno-unsequenced \
	-Wno-unused-const-variable \
	-Wextra \
	-Wno-unused-parameter \
	-Wno-missing-field-initializers \
	-Wno-strict-overflow \
	-m64 \
	-march=x86-64 \
	-O2 \
	-fno-ident \
	-fdata-sections \
	-ffunction-sections \
	-funwind-tables

# Flags passed to only C files.
CFLAGS_C_Release :=

# Flags passed to only C++ files.
CFLAGS_CC_Release := \
	-fno-exceptions \
	-fno-rtti \
	-fno-threadsafe-statics \
	-fvisibility-inlines-hidden \
	-Wsign-compare \
	-std=gnu++11 \
	-Wnon-virtual-dtor \
	-Woverloaded-virtual

INCS_Release := \
	-I$(obj)/gen \
	-I. \
	-Iwebrtc/common_audio/resampler/include \
	-Iwebrtc/common_audio/signal_processing/include \
	-Ichromium/src/third_party/openmax_dl

OBJS := \
	$(obj).target/$(TARGET)/webrtc/common_audio/channel_buffer.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/audio_converter.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/audio_ring_buffer.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/audio_util.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/blocker.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/fir_filter.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/fir_filter_sse.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/resampler/push_resampler.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/resampler/push_sinc_resampler.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/resampler/resampler.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/resampler/sinc_resampler.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/resampler/sinc_resampler_sse.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/ring_buffer.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/auto_corr_to_refl_coef.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/auto_correlation.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/complex_fft.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/complex_bit_reverse.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/copy_set_operations.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/cross_correlation.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/division_operations.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/dot_product_with_scale.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/downsample_fast.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/energy.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/filter_ar.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/filter_ar_fast_q12.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/filter_ma_fast_q12.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/get_hanning_window.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/get_scaling_square.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/ilbc_specific_functions.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/levinson_durbin.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/lpc_to_refl_coef.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/min_max_operations.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/randomization_functions.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/refl_coef_to_lpc.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/real_fft.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/resample.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/resample_48khz.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/resample_by_2.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/resample_by_2_internal.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/resample_fractional.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/spl_init.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/spl_sqrt.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/spl_sqrt_floor.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/splitting_filter.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/sqrt_of_one_minus_x_squared.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/signal_processing/vector_scaling_operations.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/vad.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/webrtc_vad.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/vad_core.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/vad_filterbank.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/vad_gmm.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/vad/vad_sp.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/wav_header.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/wav_file.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/window_generator.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/lapped_transform.o \
	$(obj).target/$(TARGET)/webrtc/common_audio/real_fourier.o

# Add to the list of files we specially track dependencies for.
all_deps += $(OBJS)

# CFLAGS et al overrides must be target-local.
# See "Target-specific Variable Values" in the GNU Make manual.
$(OBJS): TOOLSET := $(TOOLSET)
$(OBJS): GYP_CFLAGS := $(DEFS_$(BUILDTYPE)) $(INCS_$(BUILDTYPE))  $(CFLAGS_$(BUILDTYPE)) $(CFLAGS_C_$(BUILDTYPE))
$(OBJS): GYP_CXXFLAGS := $(DEFS_$(BUILDTYPE)) $(INCS_$(BUILDTYPE))  $(CFLAGS_$(BUILDTYPE)) $(CFLAGS_CC_$(BUILDTYPE))

# Suffix rules, putting all outputs into $(obj).

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(srcdir)/%.cc FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(srcdir)/%.c FORCE_DO_CMD
	@$(call do_cmd,cc,1)

# Try building from generated source, too.

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj).$(TOOLSET)/%.cc FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj).$(TOOLSET)/%.c FORCE_DO_CMD
	@$(call do_cmd,cc,1)

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj)/%.cc FORCE_DO_CMD
	@$(call do_cmd,cxx,1)

$(obj).$(TOOLSET)/$(TARGET)/%.o: $(obj)/%.c FORCE_DO_CMD
	@$(call do_cmd,cc,1)

# End of this set of suffix rules
### Rules for final target.
LDFLAGS_Debug := \
	-Wl,-z,now \
	-Wl,-z,relro \
	-Wl,--fatal-warnings \
	-Wl,-z,defs \
	-pthread \
	-Wl,-z,noexecstack \
	-fPIC \
	-fuse-ld=gold \
	-B/home/lizhiyong/linux/trunk64/webrtc/src/third_party/binutils/Linux_x64/Release/bin \
	-Wl,--disable-new-dtags \
	-m64 \
	-Wl,--detect-odr-violations \
	-Wl,--icf=safe

LDFLAGS_Release := \
	-Wl,-z,now \
	-Wl,-z,relro \
	-Wl,--fatal-warnings \
	-Wl,-z,defs \
	-pthread \
	-Wl,-z,noexecstack \
	-fPIC \
	-fuse-ld=gold \
	-B/home/lizhiyong/linux/trunk64/webrtc/src/third_party/binutils/Linux_x64/Release/bin \
	-Wl,--disable-new-dtags \
	-m64 \
	-Wl,--detect-odr-violations \
	-Wl,--icf=safe \
	-Wl,-O1 \
	-Wl,--as-needed \
	-Wl,--gc-sections

LIBS := \
	

$(obj).target/webrtc/common_audio/libcommon_audio.a: GYP_LDFLAGS := $(LDFLAGS_$(BUILDTYPE))
$(obj).target/webrtc/common_audio/libcommon_audio.a: LIBS := $(LIBS)
$(obj).target/webrtc/common_audio/libcommon_audio.a: TOOLSET := $(TOOLSET)
$(obj).target/webrtc/common_audio/libcommon_audio.a: $(OBJS) FORCE_DO_CMD
	$(call do_cmd,alink_thin)

all_deps += $(obj).target/webrtc/common_audio/libcommon_audio.a
# Add target alias
.PHONY: common_audio
common_audio: $(obj).target/webrtc/common_audio/libcommon_audio.a

# Add target alias to "all" target.
.PHONY: all
all: common_audio

