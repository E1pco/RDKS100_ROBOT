# Make fragment for the RDK S100 hard-sync framework.
# Include this from the vendor MCU FreeRTOS build after defining MCU_ROOT.

MCU_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)

HARD_SYNC_INCLUDES := \
  -I$(MCU_ROOT)/Common/HardSync \
  -I$(MCU_ROOT)/Config \
  -I$(MCU_ROOT)/McalCdd/HardSync \
  -I$(MCU_ROOT)/Platform/Compiler \
  -I$(MCU_ROOT)/Platform/Schm \
  -I$(MCU_ROOT)/Service/HardSync \
  -I$(MCU_ROOT)/Target/rdks100

HARD_SYNC_SOURCES := \
  $(MCU_ROOT)/Service/HardSync/hard_sync.c \
  $(MCU_ROOT)/Config/rdks100_hard_sync_config.c

# For Linux-side syntax checks only. Replace with Target/rdks100 port code in
# the real MCU build.
HARD_SYNC_HOST_STUB_SOURCE := \
  $(MCU_ROOT)/McalCdd/HardSync/hard_sync_port_stub.c
