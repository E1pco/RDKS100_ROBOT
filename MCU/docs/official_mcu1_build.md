# Official MCU1 Build Flow

This note records the vendor build flow for RDK S100 MCU1.

## Python Version

The official documentation states that RDK S100/S600 MCU development uses:

```text
Python 3.8.10
```

The current RDK S100 Ubuntu 22.04 userland may provide a newer default
`python3`. If the vendor build script fails under the default Python, create a
Python 3.8 environment for the MCU package instead of changing the system
Python globally.

## Toolchain

The official build may download the toolchain from Arm during the first build,
which can take about 10 minutes and can fail on unstable networks.

Preferred offline flow:

1. Download the official toolchain from the D-Robotics tool download page.
2. Move it into the vendor MCU source tree:

```bash
mv <toolchain_path>/<toolchain_file> <new_code>/Build/ToolChain/Gcc/
```

When the build detects the toolchain in `Build/ToolChain/Gcc/`, it should not
download it again.

## Build MCU1 Debug

```bash
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py lite matrix B s100 mcu1 gcc debug
```

The debug image contains debug information.

## Build MCU1 Release

```bash
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py lite matrix B s100 mcu1 gcc release
```

The release image does not contain debug information.

## Current Repository Note

`MCU/` in this workspace is currently the hard-sync framework scaffold. The
official vendor MCU package still needs to be added or mounted before the
commands above can run unchanged.
