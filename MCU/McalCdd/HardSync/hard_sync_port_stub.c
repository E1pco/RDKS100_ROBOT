#include "hard_sync_port.h"

#if defined(HARD_SYNC_PORT_ENABLE_STDIO)
#include <stdio.h>
#endif

HardSyncResult HardSyncPort_InitTimebase(uint32_t timebase_hz) {
  if (timebase_hz == 0u) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  return HARD_SYNC_OK;
}

HardSyncResult HardSyncPort_ConfigurePeriodicOutput(
    const HardSyncOutputConfig *output) {
  if (output == 0) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  if (output->period_us == 0u || output->pulse_width_us == 0u ||
      output->pulse_width_us >= output->period_us) {
    return HARD_SYNC_INVALID_CONFIG;
  }
  return HARD_SYNC_OK;
}

HardSyncResult HardSyncPort_StartOutput(uint8_t output_id) {
  (void)output_id;
  return HARD_SYNC_OK;
}

HardSyncResult HardSyncPort_StopOutput(uint8_t output_id) {
  (void)output_id;
  return HARD_SYNC_OK;
}

uint64_t HardSyncPort_GetTimeNs(void) {
  return 0u;
}

void HardSyncPort_Log(const char *message) {
#if defined(HARD_SYNC_PORT_ENABLE_STDIO)
  if (message != 0) {
    (void)printf("%s\n", message);
  }
#else
  (void)message;
#endif
}
