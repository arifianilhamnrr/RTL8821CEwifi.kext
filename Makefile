PROJ_ROOT := $(shell pwd)
SDK := $(shell xcrun --sdk macosx --show-sdk-path)
MKSDK := $(PROJ_ROOT)/MacKernelSDK
BUILD_DIR := $(PROJ_ROOT)/build/probe
OUT_KEXT := $(PROJ_ROOT)/build/out/RTL8821CEProbe.kext
OUT_BIN := $(OUT_KEXT)/Contents/MacOS/RTL8821CEProbe

CXX := xcrun clang++
CC := xcrun clang
LD := xcrun clang++

COMMON_FLAGS := -arch x86_64 -mmacosx-version-min=11.0 -isysroot $(SDK) \
	-I$(MKSDK)/Headers \
	-I$(SDK)/System/Library/Frameworks/Kernel.framework/Headers \
	-fno-stack-protector -mkernel
CXXFLAGS := $(COMMON_FLAGS) -fapple-kext -fno-exceptions -fno-rtti -std=c++17 \
	-DKERNEL -Wno-deprecated-declarations -Wno-nullability-completeness
CFLAGS := $(COMMON_FLAGS)
LDFLAGS := -arch x86_64 -static -nostdlib -Xlinker -kext \
	-Xlinker -sectcreate -Xlinker __DATA_CONST -Xlinker __firmware \
	-Xlinker $(PROJ_ROOT)/RTL8821CE/firmware/rtw8821c_fw.bin \
	$(MKSDK)/Library/x86_64/libkmod.a \
	$(MKSDK)/Library/universal/libkmodc++.a \
	-F$(SDK)/System/Library/Frameworks

OBJS := $(BUILD_DIR)/RTL8821CEProbe.o $(BUILD_DIR)/kmod_info.o

.PHONY: all probe clean

all: probe

probe: $(OUT_BIN)

$(OUT_BIN): $(OBJS) Probe/Info.plist RTL8821CE/firmware/rtw8821c_fw.bin
	mkdir -p "$(OUT_KEXT)/Contents/MacOS"
	cp Probe/Info.plist "$(OUT_KEXT)/Contents/Info.plist"
	$(LD) $(LDFLAGS) -o "$@" $(OBJS)
	codesign --force --sign - "$(OUT_KEXT)"

$(BUILD_DIR)/RTL8821CEProbe.o: Probe/RTL8821CEProbe.cpp Probe/RTL8821CEProbe.hpp
	mkdir -p "$(BUILD_DIR)"
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/kmod_info.o: Probe/kmod_info.c
	mkdir -p "$(BUILD_DIR)"
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build
