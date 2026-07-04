#ifndef HARD_SYNC_H_
#define HARD_SYNC_H_

#include "hard_sync_types.h"

#ifdef __cplusplus
extern "C" {
#endif

HardSyncResult HardSync_ValidateConfig(const HardSyncConfig *config);
HardSyncResult HardSync_Init(const HardSyncConfig *config);
HardSyncResult HardSync_Start(void);
HardSyncResult HardSync_Stop(void);
const HardSyncConfig *HardSync_GetConfig(void);

#ifdef __cplusplus
}
#endif

#endif  // HARD_SYNC_H_
