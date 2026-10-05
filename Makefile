# Fern-Airspy, an Airspy input module for FernSDR.
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   make                        build/fern-airspy, linked against the system libusb
#   make static [ARCH=...]      build/static-ARCH/fern-airspy, fully static
#   make package [ARCH=...]     dist/airspy-VERSION-linux-ARCH.fernmod
#   make test                   unit, protocol, command line and package tests
#   make test-asan              the unit and protocol tests under ASan and UBSan
#   make test-tsan              the unit and protocol tests under TSan
#   make test-qemu ARCH=...     the unit and protocol tests built for ARCH, run under QEMU
#   make bench                  the R2/Mini sample path's CPU time per second of samples
#
# ARCH is x86_64, aarch64 or armhf (ARMv7 with hardware floating point) and
# defaults to the machine's own.

VERSION := 0.1.0
.DEFAULT_GOAL := all

HOST_ARCH := $(shell uname -m | sed -e 's/^armv7.*/armhf/' -e 's/^arm64$$/aarch64/')
ARCH ?= $(HOST_ARCH)
ifeq ($(filter $(ARCH),x86_64 aarch64 armhf),)
$(error ARCH must be x86_64, aarch64 or armhf, not "$(ARCH)")
endif

CROSS_x86_64 := x86_64-linux-gnu-
CROSS_aarch64 := aarch64-linux-gnu-
CROSS_armhf := arm-linux-gnueabihf-
ifeq ($(ARCH),$(HOST_ARCH))
CROSS ?=
else
CROSS ?= $(CROSS_$(ARCH))
endif
ARCH_FLAGS_armhf := -march=armv7-a -mfpu=vfpv3-d16 -mfloat-abi=hard
ARCH_FLAGS := $(ARCH_FLAGS_$(ARCH))

OPT ?= -O2
CXXFLAGS_BASE := -std=c++17 $(OPT) -g -pthread -Wall -Wextra -Wpedantic -Wshadow -Wformat=2 \
	-DFERN_AIRSPY_VERSION='"$(VERSION)"' -Isrc
# The vendor libraries build with these; their own warnings are theirs.
# libairspy's converter shifts negative values left, which GCC defines as
# the arithmetic shift it means; the sanitizers are told to let it.
VENDOR_CFLAGS := -std=gnu11 $(OPT) -g -pthread -fvisibility=hidden -w
LIBUSB_CFLAGS := -std=gnu11 $(OPT) -g -pthread -fvisibility=hidden -Wall -Wextra \
	-Ithird_party/libusb-config -Ithird_party/libusb/libusb
# Third-party headers are included as system headers, so that the warning
# options above apply to the module's own code only.
VENDOR_INCLUDE := -isystem third_party/libairspy -isystem third_party/libairspyhf
VENDOR_LIBUSB_INCLUDE := -isystem third_party/libusb/libusb
# Only evaluated when the native build needs it.
SYSTEM_LIBUSB_CFLAGS = $(patsubst -I%,-isystem %,$(shell pkg-config --cflags libusb-1.0))
SYSTEM_LIBUSB_LIBS = $(shell pkg-config --libs libusb-1.0)

# The licences of the module and of the libraries static builds carry, for
# fern-airspy --notices. A name with "head:" embeds only a file's leading
# comment, where the vendor libraries keep their copyright lines.
NOTICES_SRC := build/gen/notices.cpp
NOTICE_ARGS := module_licence LICENSE \
	-- libairspy_notices third_party/libairspy/LICENSE.md head:third_party/libairspy/airspy.c \
	head:third_party/libairspy/iqconverter_int16.c head:third_party/libairspy/iqconverter_float.c \
	head:third_party/libairspy/filters.h \
	-- libairspyhf_notices third_party/libairspyhf/LICENSE.BSD head:third_party/libairspyhf/airspyhf.c \
	head:third_party/libairspyhf/iqbalancer.c \
	-- libusb_notices third_party/libusb/AUTHORS third_party/libusb/COPYING
NOTICE_FILES := LICENSE third_party/libairspy/LICENSE.md third_party/libairspy/airspy.c \
	third_party/libairspy/iqconverter_int16.c third_party/libairspy/iqconverter_float.c \
	third_party/libairspy/filters.h third_party/libairspyhf/LICENSE.BSD third_party/libairspyhf/airspyhf.c \
	third_party/libairspyhf/iqbalancer.c third_party/libusb/AUTHORS third_party/libusb/COPYING

MODULE_SRCS := src/json.cpp src/io.cpp src/log.cpp src/settings.cpp src/r820t.cpp src/convert.cpp \
	src/gain_control.cpp src/stream.cpp src/receiver.cpp src/session.cpp src/listing.cpp
PROGRAM_SRCS := src/main.cpp src/airspy_backend.cpp
TEST_SRCS := tests/test_main.cpp tests/fake_backend.cpp tests/test_json.cpp tests/test_settings.cpp \
	tests/test_r820t.cpp tests/test_convert.cpp tests/test_gain_control.cpp tests/test_stream.cpp \
	tests/test_receiver.cpp tests/test_session.cpp tests/test_listing.cpp
BENCH_SRCS := tests/bench_convert.cpp src/convert.cpp src/log.cpp
# The module converts the R2's and Mini's real samples to I/Q itself, with
# libairspy's own converter, so the tests carry that file too.
CONVERTER_SRCS := iqconverter_float.c
AIRSPY_SRCS := airspy.c iqconverter_float.c iqconverter_int16.c
AIRSPYHF_SRCS := airspyhf.c iqbalancer.c
LIBUSB_SRCS := core.c descriptor.c hotplug.c io.c sync.c strerror.c os/linux_usbfs.c os/linux_netlink.c \
	os/events_posix.c os/threads_posix.c

cxx_objs = $(patsubst %.cpp,$(1)/obj/%.o,$(2))
airspy_objs = $(patsubst %.c,$(1)/obj/libairspy/%.o,$(2))
airspyhf_objs = $(patsubst %.c,$(1)/obj/libairspyhf/%.o,$(AIRSPYHF_SRCS))
libusb_objs = $(patsubst %.c,$(1)/obj/libusb/%.o,$(LIBUSB_SRCS))

# Compile rules for one build directory.
#   $(1) directory, $(2) C compiler, $(3) C++ compiler, $(4) extra flags,
#   $(5) libusb include flags for code that includes libusb.h
define compile_rules
$(1)/obj/src/%.o: src/%.cpp
	@mkdir -p $$(@D)
	$(3) $$(CXXFLAGS_BASE) $(4) $$(VENDOR_INCLUDE) $(5) -MMD -MP -c $$< -o $$@
$(1)/obj/tests/%.o: tests/%.cpp
	@mkdir -p $$(@D)
	$(3) $$(CXXFLAGS_BASE) $(4) $$(VENDOR_INCLUDE) -MMD -MP -c $$< -o $$@
$(1)/obj/gen/notices.o: $(NOTICES_SRC)
	@mkdir -p $$(@D)
	$(3) $$(CXXFLAGS_BASE) $(4) -c $$< -o $$@
$(1)/obj/libairspy/%.o: third_party/libairspy/%.c
	@mkdir -p $$(@D)
	$(2) $$(VENDOR_CFLAGS) $(4) -fno-sanitize=shift $(5) -MMD -MP -c $$< -o $$@
$(1)/obj/libairspyhf/%.o: third_party/libairspyhf/%.c
	@mkdir -p $$(@D)
	$(2) $$(VENDOR_CFLAGS) $(4) $(5) -MMD -MP -c $$< -o $$@
$(1)/obj/libusb/%.o: third_party/libusb/libusb/%.c
	@mkdir -p $$(@D)
	$(2) $$(LIBUSB_CFLAGS) $(4) -MMD -MP -c $$< -o $$@
endef

NATIVE_DIR := build/native
STATIC_DIR := build/static-$(ARCH)
TEST_DIR := build/test
ASAN_DIR := build/asan
TSAN_DIR := build/tsan
BENCH_DIR := build/bench
SANITIZE := -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined
# TSan needs the address space laid out without the randomisation newer
# kernels apply.
HOST_ARCH_UNAME := $(shell uname -m)

$(eval $(call compile_rules,$(NATIVE_DIR),$(CC),$(CXX),,$$(SYSTEM_LIBUSB_CFLAGS)))
$(eval $(call compile_rules,$(STATIC_DIR),$(CROSS)gcc,$(CROSS)g++,$(ARCH_FLAGS) -ffunction-sections -fdata-sections,$(VENDOR_LIBUSB_INCLUDE)))
$(eval $(call compile_rules,$(TEST_DIR),$(CC),$(CXX),,))
$(eval $(call compile_rules,$(ASAN_DIR),$(CC),$(CXX),$(SANITIZE),))
$(eval $(call compile_rules,$(TSAN_DIR),$(CC),$(CXX),-fsanitize=thread,))
$(eval $(call compile_rules,$(BENCH_DIR),$(CC),$(CXX),,))

NATIVE_OBJS := $(call cxx_objs,$(NATIVE_DIR),$(MODULE_SRCS) $(PROGRAM_SRCS)) \
	$(call airspy_objs,$(NATIVE_DIR),$(AIRSPY_SRCS)) $(call airspyhf_objs,$(NATIVE_DIR)) $(NATIVE_DIR)/obj/gen/notices.o
STATIC_OBJS := $(call cxx_objs,$(STATIC_DIR),$(MODULE_SRCS) $(PROGRAM_SRCS)) \
	$(call airspy_objs,$(STATIC_DIR),$(AIRSPY_SRCS)) $(call airspyhf_objs,$(STATIC_DIR)) \
	$(STATIC_DIR)/obj/gen/notices.o $(call libusb_objs,$(STATIC_DIR))
TEST_OBJS := $(call cxx_objs,$(TEST_DIR),$(MODULE_SRCS) $(TEST_SRCS)) $(call airspy_objs,$(TEST_DIR),$(CONVERTER_SRCS))
ASAN_OBJS := $(call cxx_objs,$(ASAN_DIR),$(MODULE_SRCS) $(TEST_SRCS)) $(call airspy_objs,$(ASAN_DIR),$(CONVERTER_SRCS))
TSAN_OBJS := $(call cxx_objs,$(TSAN_DIR),$(MODULE_SRCS) $(TEST_SRCS)) $(call airspy_objs,$(TSAN_DIR),$(CONVERTER_SRCS))
BENCH_OBJS := $(call cxx_objs,$(BENCH_DIR),$(BENCH_SRCS)) $(call airspy_objs,$(BENCH_DIR),$(CONVERTER_SRCS))
CROSS_TEST_OBJS := $(call cxx_objs,$(STATIC_DIR),$(MODULE_SRCS) $(TEST_SRCS)) \
	$(call airspy_objs,$(STATIC_DIR),$(CONVERTER_SRCS))
QEMU_x86_64 :=
QEMU_aarch64 := qemu-aarch64
QEMU_armhf := qemu-arm

PACKAGE := dist/airspy-$(VERSION)-linux-$(ARCH).fernmod
# The settings in every package come from --describe of a binary that runs
# here, built from the same sources, so all platforms carry the same list.
DESCRIBE_BIN := build/static-$(HOST_ARCH)/fern-airspy

.PHONY: all static package test test-asan test-tsan test-qemu bench clean print-version check-libusb check-toolchain FORCE

all: build/fern-airspy

check-libusb:
	@pkg-config --exists libusb-1.0 || { echo "The native build needs the libusb-1.0 development files" \
		"(Debian and Ubuntu: apt install libusb-1.0-0-dev pkg-config). make static needs neither." >&2; exit 1; }

$(call airspy_objs,$(NATIVE_DIR),$(AIRSPY_SRCS)) $(call airspyhf_objs,$(NATIVE_DIR)) \
	$(NATIVE_DIR)/obj/src/airspy_backend.o: | check-libusb

$(NOTICES_SRC): $(NOTICE_FILES) tools/embed_text.py
	@mkdir -p $(@D)
	python3 tools/embed_text.py $(NOTICE_ARGS) > $@.tmp
	mv $@.tmp $@

build/fern-airspy: $(NATIVE_OBJS) | check-libusb
	$(CXX) -pthread -o $@ $^ $(SYSTEM_LIBUSB_LIBS) -lm

TOOLCHAIN_PACKAGES_x86_64 := gcc-x86-64-linux-gnu g++-x86-64-linux-gnu
TOOLCHAIN_PACKAGES_aarch64 := gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
TOOLCHAIN_PACKAGES_armhf := gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

check-toolchain:
	@command -v $(CROSS)g++ >/dev/null 2>&1 || { echo "make static ARCH=$(ARCH) needs $(CROSS)gcc and $(CROSS)g++" \
		"(Debian and Ubuntu: apt install $(TOOLCHAIN_PACKAGES_$(ARCH)))." >&2; exit 1; }

$(STATIC_OBJS): | check-toolchain

static: $(STATIC_DIR)/fern-airspy

$(STATIC_DIR)/fern-airspy: $(STATIC_OBJS)
	$(CROSS)g++ $(ARCH_FLAGS) -static -pthread -s -Wl,--gc-sections -o $@ $^ -lm

ifneq ($(ARCH),$(HOST_ARCH))
$(DESCRIBE_BIN): FORCE
	$(MAKE) static ARCH=$(HOST_ARCH)
endif

package: $(PACKAGE)

$(PACKAGE): $(STATIC_DIR)/fern-airspy $(DESCRIBE_BIN) tools/mkfernmod.py tools/check_fernmod.py
	@mkdir -p dist
	python3 tools/mkfernmod.py --executable $(STATIC_DIR)/fern-airspy --describe-with $(DESCRIBE_BIN) \
		--platform linux-$(ARCH) --version $(VERSION) --output $@
	python3 tools/check_fernmod.py $@

$(TEST_DIR)/fern-airspy-tests: $(TEST_OBJS)
	$(CXX) -pthread -o $@ $^

$(ASAN_DIR)/fern-airspy-tests: $(ASAN_OBJS)
	$(CXX) $(SANITIZE) -pthread -o $@ $^

$(TSAN_DIR)/fern-airspy-tests: $(TSAN_OBJS)
	$(CXX) -fsanitize=thread -pthread -o $@ $^

$(STATIC_DIR)/fern-airspy-tests: $(CROSS_TEST_OBJS) | check-toolchain
	$(CROSS)g++ $(ARCH_FLAGS) -static -pthread -o $@ $^

$(BENCH_DIR)/fern-airspy-bench: $(BENCH_OBJS)
	$(CXX) -pthread -o $@ $^

test: $(TEST_DIR)/fern-airspy-tests build/fern-airspy
	$(TEST_DIR)/fern-airspy-tests
	sh tests/cli_test.sh build/fern-airspy
	sh tests/package_test.sh build/fern-airspy

test-asan: $(ASAN_DIR)/fern-airspy-tests
	ASAN_OPTIONS=detect_leaks=1:abort_on_error=0 UBSAN_OPTIONS=print_stacktrace=1 $(ASAN_DIR)/fern-airspy-tests

test-tsan: $(TSAN_DIR)/fern-airspy-tests
	TSAN_OPTIONS=halt_on_error=1 setarch $(HOST_ARCH_UNAME) -R $(TSAN_DIR)/fern-airspy-tests

test-qemu: $(STATIC_DIR)/fern-airspy-tests
	$(QEMU_$(ARCH)) $(STATIC_DIR)/fern-airspy-tests

bench: $(BENCH_DIR)/fern-airspy-bench
	$(BENCH_DIR)/fern-airspy-bench

print-version:
	@echo $(VERSION)

clean:
	rm -rf build dist

FORCE:

-include $(shell find build -name '*.d' 2>/dev/null)
