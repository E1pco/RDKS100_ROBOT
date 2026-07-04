# Development Environment

This note is for the current workflow: Windows runs VSCode, and VSCode connects
to the RDK S100 board through Remote SSH.

## Practical Conclusion

Windows does not need the Ubuntu build packages listed by the vendor. In this
workflow Windows is only the editor and SSH client.

Install build dependencies in the environment where commands actually run:

- If VSCode terminal is connected to RDK S100, install dependencies on the RDK
  S100 Ubuntu system.
- If using a separate cross-build host, install dependencies on that Ubuntu
  22.04 host.
- If using Windows-only hardware, use WSL2 Ubuntu 22.04 as the Linux host and
  install dependencies inside WSL2.

## Recommended Modes

### Mode A: Board-Side Development Through VSCode Remote SSH

Use this mode for early bring-up, driver integration, and checking real LPWM,
pinmux, remoteproc, and board resources.

Windows requirements:

- VSCode
- Remote SSH extension
- SSH key or password access to RDK S100
- Git credentials if pushing from the remote board

RDK S100 requirements:

- Vendor MCU package present on the board or mounted into the workspace
- `gcc-arm-none-eabi-10.3~2021.10` available under the vendor MCU build tree
- Basic Linux build tools installed
- Python 3.8.10 for the official MCU build scripts, according to the vendor
  documentation

This is the simplest mode for the current project because the source tree is
already on the RDK S100 side.

### Mode B: Separate Ubuntu 22.04 Cross-Build Host

Use this mode when full MCU images build slowly on the board or when the vendor
build system expects a clean host machine.

Host requirements:

- Ubuntu 22.04, preferably matching the RDK S100 OS version
- Vendor MCU package and toolchain installed locally
- Deployment path back to the board, such as `rsync`, `scp`, or vendor flashing
  tools

### Mode C: Windows + WSL2 Ubuntu 22.04

Use this mode if a separate Linux PC is not available.

Install Ubuntu 22.04 in WSL2, install the same packages there, and build from
the WSL2 filesystem. Avoid building large firmware trees from `/mnt/c/...`
because file I/O can be slow.

## Ubuntu 22.04 Package Setup

Run this in the Linux environment that will execute the build:

```bash
sudo apt-get update
sudo apt-get install -y build-essential make cmake libpcre3 libpcre3-dev bc bison \
  flex python3-numpy mtd-utils zlib1g-dev debootstrap \
  libdata-hexdumper-perl libncurses5-dev zip qemu-user-static \
  curl repo git liblz4-tool apt-cacher-ng libssl-dev checkpolicy autoconf \
  android-sdk-libsparse-utils mtools parted dosfstools udev rsync python3-pip scons
```

Python dependencies:

```bash
python3 -m pip install --user "scons>=4.0.0"
python3 -m pip install --user ecdsa
python3 -m pip install --user tqdm
```

## MCU Toolchain

The MCU compiler expected by the vendor documentation is:

```text
gcc-arm-none-eabi-10.3~2021.10
```

The official build can download and unpack the toolchain during the first build,
but the download may fail or be incomplete on an unstable network. Prefer the
offline flow:

1. Download the toolchain from the official D-Robotics tool download page.
2. Put the toolchain archive or extracted toolchain under the vendor MCU tree:

```text
Build/ToolChain/Gcc/
```

The path is case-sensitive on Linux. Use `ToolChain/Gcc` when following the
vendor build scripts.

After placing the toolchain, verify from the vendor MCU build environment:

```bash
arm-none-eabi-gcc --version
```

The first line should report the 10.3 2021.10 generation. If it reports a
different system package version, prefer the vendor-provided toolchain path in
the MCU build environment.

## Official MCU1 Build Commands

The official MCU1 build entry point is:

```bash
cd mcu/Build/FreeRtos_mcu1
```

Build debug image:

```bash
python build_freertos.py lite matrix B s100 mcu1 gcc debug
```

Build release image:

```bash
python build_freertos.py lite matrix B s100 mcu1 gcc release
```

Debug images contain debug information. Release images do not include debug
information.

In this repository, `MCU/` is currently a framework scaffold. Once the official
MCU package is added, align the scaffold with the vendor package root before
running these commands.

## VSCode Remote SSH Notes

When VSCode is connected to RDK S100:

- The integrated terminal runs on RDK S100, not on Windows.
- Extensions that compile or index C/C++ should be installed on the remote side.
- Configure C/C++ include paths against the MCU package on the remote filesystem.
- Git operations run against the remote checkout, which is currently the
  `Alan` branch in this workspace.

Suggested workspace tasks after the vendor MCU package is added:

- Host syntax check using the stub port.
- MCU1 firmware build using the vendor FreeRTOS build.
- Deploy or copy the generated MCU1 image to the vendor-defined output path.
- Start/stop MCU1 through the documented remoteproc flow.

## Current Framework Check

The current framework can be syntax-checked without the vendor toolchain:

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

This check validates framework wiring only. It does not build MCU firmware and
does not drive LPWM hardware.
