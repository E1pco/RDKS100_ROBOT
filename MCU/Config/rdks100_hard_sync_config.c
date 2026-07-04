#include "rdks100_hard_sync_config.h"

#include "rdks100_pinmap.h"

static const HardSyncOutputConfig kHardSyncOutputs[] = {
    {
        .name = "camera0_trigger",
        .output_id = RDKS100_HARDSYNC_CAMERA0_LPWM_CHANNEL,
        .signal_kind = HARD_SYNC_SIGNAL_CAMERA_TRIGGER,
        .period_us = 100000u,
        .pulse_width_us = 1000u,
        .phase_offset_us = 0u,
        .polarity = HARD_SYNC_ACTIVE_HIGH,
        .enabled = true,
    },
    {
        .name = "lidar0_pps",
        .output_id = RDKS100_HARDSYNC_LIDAR0_SYNC_CHANNEL,
        .signal_kind = HARD_SYNC_SIGNAL_LIDAR_PPS,
        .period_us = 1000000u,
        .pulse_width_us = 10000u,
        .phase_offset_us = 0u,
        .polarity = HARD_SYNC_ACTIVE_HIGH,
        .enabled = true,
    },
};

const HardSyncConfig g_rdks100_hard_sync_config = {
    .outputs = kHardSyncOutputs,
    .output_count = sizeof(kHardSyncOutputs) / sizeof(kHardSyncOutputs[0]),
    /*
     * MCU1 framework timebase. The real RDK S100 port should bind this to the
     * MCU timer/LPWM clock domain and avoid resources reserved by MCU0.
     */
    .timebase_hz = 1000000u,
    .start_aligned_to_next_second = true,
};
