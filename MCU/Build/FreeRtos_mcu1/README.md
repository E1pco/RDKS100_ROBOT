# FreeRTOS MCU1 Build Hook

Place MCU1-specific build rules here when the vendor build package is added.

Do not link this framework into MCU0. MCU0 is responsible for Acore/Linux
startup, MCU1 startup, and power management, and is expected to use the
vendor-verified binary.

Expected integration:

1. Include `../FreeRtos/hard_sync_app.mk`.
2. Compile `Service/HardSync` and `Config`.
3. Compile the real RDK S100 port implementation from `Target/rdks100`.
4. Link the hard-sync sample or production task into the MCU1 image.

Official vendor build commands:

```bash
cd mcu/Build/FreeRtos_mcu1
python build_freertos.py lite matrix B s100 mcu1 gcc debug
python build_freertos.py lite matrix B s100 mcu1 gcc release
```

Use `debug` while validating LPWM timing and `release` for the final deployed
image.

MCU1 is managed from Acore/Linux through the remoteproc path controlled by MCU0.
The production image should therefore remain restart-safe: outputs must be
driven inactive during init failure and stop paths.
