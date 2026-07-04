#ifndef HARD_SYNC_PORT_H_
#define HARD_SYNC_PORT_H_

#include "hard_sync_types.h"

#ifdef __cplusplus
extern "C" {
#endif

HardSyncResult HardSyncPort_InitTimebase(uint32_t timebase_hz);
HardSyncResult HardSyncPort_ConfigurePeriodicOutput(
    const HardSyncOutputConfig *output);
HardSyncResult HardSyncPort_StartOutput(uint8_t output_id);
HardSyncResult HardSyncPort_StopOutput(uint8_t output_id);
uint64_t HardSyncPort_GetTimeNs(void);
void HardSyncPort_Log(const char *message);

#ifdef __cplusplus
}
#endif

#endif  // HARD_SYNC_PORT_H_
