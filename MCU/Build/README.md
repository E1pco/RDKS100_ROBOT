# Build Integration

This directory mirrors the vendor MCU package layout and is reserved for the
real FreeRTOS/MCU1 build scripts.

The target build assumptions are:

- Core: ARM R52+
- OS: FreeRTOS Kernel V10.0.1
- Toolchain: gcc-arm-none-eabi-10.3~2021.10
- Development target: MCU1 only
- Official build Python: 3.8.10

For dependency setup, use `../docs/development_environment.md`.

Current contents are framework placeholders:

- `FreeRtos/hard_sync_app.mk`: source/include list for the hard-sync service.
- `FreeRtos_mcu1/`: target-specific build hook placeholder.
- `ToolChain/Gcc/`: official toolchain placement path.
- `Toolchain/`: local framework metadata placeholder.
- `Tools/`: helper-script placeholder.

Keep generated binaries under `../output`.

Official MCU1 build commands, once the vendor MCU package is present:

```bash
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py lite matrix B s100 mcu1 gcc debug
python build_freertos.py lite matrix B s100 mcu1 gcc release
```
