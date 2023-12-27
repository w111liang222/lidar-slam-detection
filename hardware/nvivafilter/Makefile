###############################################################################
#
# Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###############################################################################

# Location of the CUDA Toolkit
CUDA_PATH ?= /usr/local/cuda
INCLUDE_DIR = /usr/include
LIB_DIR = /usr/lib/aarch64-linux-gnu
TEGRA_LIB_DIR = /usr/lib/aarch64-linux-gnu/tegra

# For hardfp
#LIB_DIR = /usr/lib/arm-linux-gnueabihf
#TEGRA_LIB_DIR = /usr/lib/arm-linux-gnueabihf/tegra

OSUPPER = $(shell uname -s 2>/dev/null | tr "[:lower:]" "[:upper:]")
OSLOWER = $(shell uname -s 2>/dev/null | tr "[:upper:]" "[:lower:]")

OS_SIZE = $(shell uname -m | sed -e "s/i.86/32/" -e "s/x86_64/64/" -e "s/armv7l/32/")
OS_ARCH = $(shell uname -m | sed -e "s/i386/i686/")

GCC ?= g++
NVCC := $(CUDA_PATH)/bin/nvcc -ccbin $(GCC)

# internal flags
NVCCFLAGS   := --shared -std=c++11
CCFLAGS     := -fPIC  -std=c++11
CVCCFLAGS:=-I/usr/local/include/opencv4
CVLDFLAGS:=-L/usr/local/lib -lopencv_core -lopencv_calib3d -lopencv_cudawarping

LDFLAGS     :=

# Extra user flags
EXTRA_NVCCFLAGS   ?=
EXTRA_LDFLAGS     ?=
EXTRA_CCFLAGS     ?=

override abi := aarch64
LDFLAGS += --dynamic-linker=/lib/ld-linux-aarch64.so.1

# For hardfp
#override abi := gnueabihf
#LDFLAGS += --dynamic-linker=/lib/ld-linux-armhf.so.3
#CCFLAGS += -mfloat-abi=hard

ifeq ($(ARMv7),1)
NVCCFLAGS += -target-cpu-arch ARM
ifneq ($(TARGET_FS),)
CCFLAGS += --sysroot=$(TARGET_FS)
LDFLAGS += --sysroot=$(TARGET_FS)
LDFLAGS += -rpath-link=$(TARGET_FS)/lib
LDFLAGS += -rpath-link=$(TARGET_FS)/usr/lib
LDFLAGS += -rpath-link=$(TARGET_FS)/usr/lib/$(abi)-linux-gnu

# For hardfp
#LDFLAGS += -rpath-link=$(TARGET_FS)/usr/lib/arm-linux-$(abi)

endif
endif

# Debug build flags
dbg = 0
ifeq ($(dbg),1)
      NVCCFLAGS += -g -G
      TARGET := debug
else
      TARGET := release
endif

ALL_CCFLAGS :=
ALL_CCFLAGS += $(NVCCFLAGS)
ALL_CCFLAGS += $(EXTRA_NVCCFLAGS)
ALL_CCFLAGS += $(addprefix -Xcompiler ,$(CCFLAGS))
ALL_CCFLAGS += $(addprefix -Xcompiler ,$(EXTRA_CCFLAGS))

ALL_LDFLAGS :=
ALL_LDFLAGS += $(ALL_CCFLAGS)
ALL_LDFLAGS += $(addprefix -Xlinker ,$(LDFLAGS))
ALL_LDFLAGS += $(addprefix -Xlinker ,$(EXTRA_LDFLAGS))

# Common includes and paths for CUDA
INCLUDES  := -I./
LIBRARIES := -L$(LIB_DIR) -lEGL -lGLESv2
LIBRARIES += -L$(TEGRA_LIB_DIR) -lcuda -lrt

################################################################################

# CUDA code generation flags
GENCODE_SM87    := -gencode arch=compute_87,code=compute_87  # for Orin
GENCODE_FLAGS   ?= $(GENCODE_SM87)

# Target rules
all: build

build: lib-gst-custom-opencv_cudaprocess.so

gst-custom-opencv_cudaprocess.o : gst-custom-opencv_cudaprocess.cu
	$(NVCC) $(INCLUDES) $(ALL_CCFLAGS) $(CVCCFLAGS) $(GENCODE_FLAGS) -o $@ -c $<

lib-gst-custom-opencv_cudaprocess.so : gst-custom-opencv_cudaprocess.o
	$(NVCC) $(ALL_LDFLAGS) $(CVLDFLAGS) $(GENCODE_FLAGS) -o $@ $^ $(LIBRARIES)

clean:
	rm lib-gst-custom-opencv_cudaprocess.so gst-custom-opencv_cudaprocess.o

clobber: clean