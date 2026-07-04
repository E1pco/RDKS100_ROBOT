# Hard Sync Bring-Up Sample

This sample initializes and starts the default RDK S100 hard-sync configuration.

For real MCU firmware, call the same sequence from the board startup task:

1. `HardSync_Init(&g_rdks100_hard_sync_config)`
2. `HardSync_Start()`
3. Keep the service alive while LPWM hardware runs autonomously.
