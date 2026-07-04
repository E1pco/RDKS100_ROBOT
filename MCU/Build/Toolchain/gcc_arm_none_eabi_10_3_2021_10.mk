# RDK S100 MCU toolchain metadata.
# Include this from the vendor build only after the actual toolchain path is
# known.

RDKS100_MCU_TOOLCHAIN_VERSION := gcc-arm-none-eabi-10.3~2021.10
CROSS_COMPILE ?= arm-none-eabi-

CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar
OBJCOPY := $(CROSS_COMPILE)objcopy
OBJDUMP := $(CROSS_COMPILE)objdump
SIZE := $(CROSS_COMPILE)size

# The exact R52+/FPU/cache flags should come from the vendor MCU package.
RDKS100_MCU_COMMON_CFLAGS ?= \
  -ffreestanding \
  -fno-common \
  -ffunction-sections \
  -fdata-sections \
  -Wall \
  -Wextra
