# Hard Sync MCAL/CDD Port

The service layer calls only the functions declared in `hard_sync_port.h`.

`hard_sync_port_stub.c` is for Linux-side syntax checks. The real RDK S100 MCU
implementation should live under `Target/rdks100` and be selected by the vendor
build system.
