#ifndef HARD_SYNC_TYPES_H_
#define HARD_SYNC_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HARD_SYNC_MAX_OUTPUTS (4u)

typedef enum {
  HARD_SYNC_OK = 0,
  HARD_SYNC_ERROR = -1,
  HARD_SYNC_INVALID_CONFIG = -2,
  HARD_SYNC_PORT_ERROR = -3,
  HARD_SYNC_NOT_INITIALIZED = -4
} HardSyncResult;

typedef enum {
  HARD_SYNC_SIGNAL_CAMERA_TRIGGER = 0,
  HARD_SYNC_SIGNAL_LIDAR_PPS = 1,
  HARD_SYNC_SIGNAL_LIDAR_FRAME_TRIGGER = 2
} HardSyncSignalKind;

typedef enum {
  HARD_SYNC_ACTIVE_HIGH = 0,
  HARD_SYNC_ACTIVE_LOW = 1
} HardSyncPolarity;

typedef struct {
  const char *name;
  uint8_t output_id;
  HardSyncSignalKind signal_kind;
  uint32_t period_us;
  uint32_t pulse_width_us;
  uint32_t phase_offset_us;
  HardSyncPolarity polarity;
  bool enabled;
} HardSyncOutputConfig;

typedef struct {
  const HardSyncOutputConfig *outputs;
  size_t output_count;
  uint32_t timebase_hz;
  bool start_aligned_to_next_second;
} HardSyncConfig;

#ifdef __cplusplus
}
#endif

#endif  // HARD_SYNC_TYPES_H_
