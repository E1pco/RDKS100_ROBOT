# Toolchain

RDK S100 MCU firmware uses the GCC toolchain:

```text
gcc-arm-none-eabi-10.3~2021.10
```

The official vendor toolchain placement path is:

```text
Build/ToolChain/Gcc/
```

This `Build/Toolchain/` directory is kept only for local framework metadata.
Use `Build/ToolChain/Gcc/` when copying the official toolchain into the vendor
MCU package.

For the framework sources, the expected compiler mode is freestanding C with
ARM R52+ target flags supplied by the vendor build.
