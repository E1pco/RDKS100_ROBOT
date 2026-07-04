#ifndef RDKS100_PINMAP_H_
#define RDKS100_PINMAP_H_

/*
 * Logical output IDs. Map these IDs to real S100 MCU LPWM/GPIO/trigger-bus
 * resources in Target/rdks100/rdks100_hard_sync_port.c.
 */
#define RDKS100_HARDSYNC_CAMERA0_LPWM_CHANNEL (0u)
#define RDKS100_HARDSYNC_LIDAR0_SYNC_CHANNEL (1u)

/*
 * Wiring notes:
 * - camera0_trigger -> camera LINE0 / Trigger / FSYNC input
 * - lidar0_pps      -> LiDAR PPS/time-sync input, or frame trigger if supported
 */

#endif  // RDKS100_PINMAP_H_
