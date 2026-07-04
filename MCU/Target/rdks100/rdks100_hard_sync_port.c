#include "hard_sync_port.h"

#if defined(HARD_SYNC_USE_RDKS100_PORT)

#include "rdks100_pinmap.h"

/*
 * Replace each TODO block with the vendor S100 MCU API calls.
 *
 * Required hardware behavior:
 * - Configure a shared timebase for all hard-sync outputs.
 * - Program period, active pulse width, phase offset, and polarity per output.
 * - Start enabled outputs from the same epoch when possible.
 */

HardSyncResult HardSyncPort_InitTimebase(uint32_t timebase_hz) {
  if (timebase_hz == 0u) {
    return HARD_SYNC_INVALID_CONFIG;
  }

  /*
   * TODO:
   * - Initialize the MCU timer/LPWM clock domain.
   * - If available, lock the timer to PPS/global time before enabling outputs.
   */
  return HARD_SYNC_PORT_ERROR;
}

HardSyncResult HardSyncPort_ConfigurePeriodicOutput(
    const HardSyncOutputConfig *output) {
  if (output == 0) {
    return HARD_SYNC_INVALID_CONFIG;
  }

  /*
   * TODO:
   * - Translate output->output_id to the real LPWM/GPIO resource.
   * - Configure period_us, pulse_width_us, phase_offset_us, and polarity.
   * - Keep the channel disabled until HardSyncPort_StartOutput().
   */
  return HARD_SYNC_PORT_ERROR;
}

HardSyncResult HardSyncPort_StartOutput(uint8_t output_id) {
  /*
   * TODO:
   * - Enable the selected LPWM/GPIO output.
   * - If the hardware supports grouped start, start all channels atomically.
   */
  (void)output_id;
  return HARD_SYNC_PORT_ERROR;
}

HardSyncResult HardSyncPort_StopOutput(uint8_t output_id) {
  /*
   * TODO:
   * - Disable the selected output and drive it to inactive state.
   */
  (void)output_id;
  return HARD_SYNC_PORT_ERROR;
}

uint64_t HardSyncPort_GetTimeNs(void) {
  /*
   * TODO:
   * - Return MCU global time in nanoseconds if available.
   * - This will be useful when exporting trigger timestamps to Linux.
   */
  return 0u;
}

void HardSyncPort_Log(const char *message) {
  /*
   * TODO:
   * - Route to the vendor MCU log service.
   */
  (void)message;
}

#endif  // HARD_SYNC_USE_RDKS100_PORT
