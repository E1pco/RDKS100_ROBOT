# GCC Toolchain Placement

The official RDK S100 MCU build checks this directory before downloading the
toolchain from the network.

Place the official MCU toolchain here:

```text
Build/ToolChain/Gcc/
```

Expected compiler:

```text
gcc-arm-none-eabi-10.3~2021.10
```

If this directory already contains the required toolchain, the vendor build
should skip the first-build download step.
