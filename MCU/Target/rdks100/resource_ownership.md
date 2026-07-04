# RDK S100 MCU Resource Ownership Notes

This framework targets MCU1 only.

## MCU0 Boundary

MCU0 owns board startup, Acore/Linux startup, MCU1 startup, and power management.
Do not modify MCU0 or reuse MCU0-owned timer, interrupt, LPWM, trigger-bus, or
power-management resources for hard-sync output.

MCU0 is expected to run the vendor-verified binary. Treat MCU0 as an integration
boundary, not as an application development target.

## MCU1 Boundary

MCU1 runs customer FreeRTOS business logic and is the correct place for this
hard-sync service.

The hard-sync service should own only the resources explicitly assigned to:

- `camera0_trigger`: LPWM/GPIO output wired to camera `LINE0` or `FSYNC`.
- `lidar0_pps`: PPS or sync output wired to the LiDAR sync input.

## Remoteproc Implication

Acore/Linux controls MCU1 start and stop through remoteproc via MCU0. The MCU1
image must therefore be restart-safe:

- Initialize outputs in the inactive state.
- Stop outputs on service failure.
- Avoid assuming that Linux userspace has already started.
- Avoid sharing resources with MCU0 power-management paths.

## Open Items

Fill these once the vendor MCU package and board pinout are available:

| Item | Owner | Value |
| --- | --- | --- |
| Camera trigger LPWM channel | MCU1 | TBD |
| Camera trigger pin / pad mux | MCU1 | TBD |
| LiDAR sync channel | MCU1 | TBD |
| LiDAR sync pin / pad mux | MCU1 | TBD |
| Shared timebase source | MCU1 / platform | TBD |
| External PPS/global-time source | platform | TBD |
