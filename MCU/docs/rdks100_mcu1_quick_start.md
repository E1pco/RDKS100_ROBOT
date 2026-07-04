# RDK S100 MCU1 Quick Start Notes

These notes capture the MCU assumptions relevant to the hard-sync framework.

## Scope

This framework is for MCU1 development only.

MCU0 starts Acore/Linux and MCU1, and handles power management. MCU0 is not
expected to be modified by customers and is not part of this hard-sync
implementation.

## Baseline

- MCU compiler: `gcc-arm-none-eabi-10.3~2021.10`
- MCU core: ARM R52+
- RTOS: FreeRTOS Kernel V10.0.1
- Customer development target: MCU1
- Startup/power-management target: MCU0 vendor binary

If the editor is running on Windows through VSCode Remote SSH, treat the remote
RDK S100 Ubuntu shell as the active development environment. Windows only needs
VSCode, Remote SSH, Git credentials, and SSH access.

Use the ARM R52 technical reference manual for low-level core behavior, but use
the vendor MCU package for actual board startup, memory map, interrupt, and
driver integration.

## System Model

RDK S100 MCU development is split into:

- MCU0: board boot entry, Acore/Linux startup, MCU1 startup, power management.
- MCU1: customer FreeRTOS business tasks, including deterministic trigger
  generation.
- Acore/Linux: ROS2, camera driver, LiDAR driver, remoteproc control path.

Acore/Linux controls MCU1 start/stop through the remoteproc path, with MCU0 as
the control point. The hard-sync firmware must therefore be robust to MCU1 being
restarted independently from Linux userspace.

## Hard-Sync Development Rules

1. Do not modify MCU0 for camera/LiDAR hard sync.
2. Keep all hard-sync application code under the MCU1 build.
3. Assign LPWM, timer, interrupt, and trigger-bus resources only after checking
   they do not conflict with MCU0 startup or power-management resources.
4. Drive camera and LiDAR outputs inactive during init failure and stop paths.
5. Keep board-specific LPWM calls inside `Target/rdks100`.

## Current Framework Mapping

The current one-camera/one-LiDAR framework maps to MCU1 as follows:

| Layer | Path | Purpose |
| --- | --- | --- |
| Service | `Service/HardSync` | Validate config and start/stop outputs |
| Config | `Config/rdks100_hard_sync_config.c` | 10 Hz camera trigger and 1 Hz LiDAR PPS defaults |
| Port API | `McalCdd/HardSync/hard_sync_port.h` | Driver abstraction used by service |
| Target | `Target/rdks100` | Real S100 LPWM/trigger-bus implementation |
| Sample | `samples/hard_sync` | Minimal MCU1 bring-up sequence |

## Next Hardware Inputs Needed

- Exact LPWM channel and pin for camera trigger.
- Exact LiDAR sync input type: PPS/time sync or external frame trigger.
- Required electrical polarity and minimum pulse width for both devices.
- Vendor LPWM/trigger-bus API headers and sample code.

For build dependencies and cross-compilation setup, see
`development_environment.md`.
