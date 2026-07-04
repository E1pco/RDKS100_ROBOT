# RDK S100 Target Layer

This directory holds board-specific mapping for the S100 MCU hard-sync service.

This target layer is for MCU1. Keep MCU0-owned startup and power-management
resources out of this mapping.

Files:

- `rdks100_pinmap.h`: logical hard-sync output IDs and wiring notes.
- `rdks100_hard_sync_port.c`: template for replacing the Linux stub with real
  S100 MCU LPWM/trigger-bus calls.

Before enabling on hardware, fill in:

1. LPWM channel used for camera `LINE0` trigger.
2. Output channel used for LiDAR PPS or frame-trigger input.
3. Electrical polarity and pulse width required by the camera and LiDAR.
4. Whether both outputs must be phase-aligned to an external PPS epoch.

See `resource_ownership.md` before assigning LPWM, timer, interrupt, or trigger
bus resources.
