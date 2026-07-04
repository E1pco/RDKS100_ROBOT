# RDK S100 MCU Hard Sync Framework

This directory is a minimal MCU1-side framework for one camera trigger output
and one LiDAR sync output on RDK S100.

RDK S100 has two MCU domains:

- MCU0 starts Acore/Linux and MCU1, and handles power management. Do not modify
  MCU0 for this hard-sync work.
- MCU1 runs customer FreeRTOS business logic. This framework targets MCU1 only.

The framework intentionally separates policy from board-specific driver calls:

- `Common/HardSync`: common types shared by service, config, and port code.
- `Config`: product-level hard-sync configuration for one camera and one LiDAR.
- `Service/HardSync`: trigger orchestration and config validation.
- `McalCdd/HardSync`: MCAL/CDD-facing port API plus a host-build stub.
- `Target/rdks100`: RDK S100 pin/channel mapping and port implementation template.
- `samples/hard_sync`: a small bring-up sample.
- `Build`: placeholders for the vendor FreeRTOS build integration.

See `docs/rdks100_mcu1_quick_start.md` for the MCU1 development assumptions
used by this framework.

See `docs/development_environment.md` for Windows + VSCode Remote SSH and
Ubuntu 22.04 cross-build environment setup notes.

## Current Signal Plan

Default config in `Config/rdks100_hard_sync_config.c` defines:

- Camera trigger: 10 Hz, 1 ms active pulse, active high.
- LiDAR sync pulse: 1 Hz PPS-style pulse, 10 ms active pulse, active high.

Adjust the LiDAR output to `HARD_SYNC_SIGNAL_LIDAR_FRAME_TRIGGER` if the target
LiDAR supports an external frame trigger instead of PPS/time-sync input.

## Integration Points

The only board-specific functions that must be replaced with real RDK S100 MCU
driver calls are declared in:

```text
McalCdd/HardSync/hard_sync_port.h
```

The template file is:

```text
Target/rdks100/rdks100_hard_sync_port.c
```

That file should eventually call the S100 MCU LPWM/trigger-bus APIs to configure
period, duty, polarity, phase, and start/stop behavior.

## Host Syntax Check

The current stub can be checked on the Linux side with:

```bash
gcc -std=c11 -Wall -Wextra \
  -I MCU/Common/HardSync \
  -I MCU/Config \
  -I MCU/McalCdd/HardSync \
  -I MCU/Platform/Compiler \
  -I MCU/Platform/Schm \
  -I MCU/Service/HardSync \
  -I MCU/Target/rdks100 \
  MCU/Service/HardSync/hard_sync.c \
  MCU/McalCdd/HardSync/hard_sync_port_stub.c \
  MCU/Config/rdks100_hard_sync_config.c \
  MCU/samples/hard_sync/main.c \
  -o /tmp/rdks100_hard_sync_sample
```

The generated Linux binary only validates the framework wiring. It does not
toggle real MCU pins.
