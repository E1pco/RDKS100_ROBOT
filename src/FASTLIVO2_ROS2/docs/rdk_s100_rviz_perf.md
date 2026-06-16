# RDK S100 FAST-LIVO2 RViz Performance Checklist

## Current Findings

- `rviz_cfg/fast_livo2.rviz` has `mapping/surround` enabled on `/cloud_registered` with `Decay Time: 10000`, `Style: Points`, `Size (Pixels): 1`, and `Selectable: false`.
- `config/mid360_mvs.yaml` previously had `publish.dense_map_en: true`, which makes `/cloud_registered` publish `feats_undistort` instead of the downsampled `feats_down_body` in LIO publishing.
- `publish.pub_scan_num` is not a reliable `/cloud_registered` rate limiter in LIVO mode. The RGB point cloud was accumulated every N frames, but the publisher path could still emit an empty PointCloud2 message on skipped frames.

## RViz Configs

| File | Purpose | surround topic | Decay Time |
| --- | --- | --- | ---: |
| `rviz_cfg/fast_livo2_latest_scan.rviz` | Latest scan only, P0 test | `/cloud_registered` | 0 |
| `rviz_cfg/fast_livo2_local_surround.rviz` | Short local surround | `/cloud_registered` | 5 |
| `rviz_cfg/fast_livo2_mapping.rviz` | Long-term map display target | `/cloud_map_visualization` | 0 |

`Decay Time=0` only displays the newest cloud message. Larger values retain historical scans in RViz, which is useful for a visual local map but can grow GPU/CPU/RSS cost over time.

## Measurement

Start the system with one RViz config at a time:

```bash
ros2 launch fast_livo mapping_avia.launch.py \
  avia_params_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/config/mid360_mvs.yaml \
  rviz_config_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/rviz_cfg/fast_livo2_latest_scan.rviz
```

In another terminal:

```bash
/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/scripts/measure_rviz_perf.sh
```

Fill the `manual_rviz_fps` column from RViz lower-left FPS at each sample time. Run the same bag or fixed scene for each Decay Time:

| Decay Time | Initial FPS | 1 min FPS | 3 min FPS | RViz RSS | Retained cloud estimate |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | | | | | |
| 2 | | | | | |
| 5 | | | | | |
| 10 | | | | | |
| 30 | | | | | |
| 10000 | | | | | |

## CPU Frequency

Use the official RDK S100 frequency management document for any setting command:

```text
https://developer.d-robotics.cc/rdk_s_doc/System_configuration/frequency_management
```

The official RDK S100 page describes S100 temperatures under `/sys/class/hwmon/hwmon0/temp*_input`, cpufreq policy files under `/sys/devices/system/cpu/cpufreq` or `/sys/devices/system/cpu/cpu0/cpufreq`, and `performance` as the high-performance governor. It also notes these runtime settings need to be applied again after reboot.

Before changing the mode, record the current value so it can be restored:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq
sudo hrut_somstatus
```

Official S100 performance-mode example:

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

Official fixed-frequency pattern, only after switching to `userspace`:

```bash
echo userspace | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1500000 | sudo tee /sys/devices/system/cpu/cpufreq/policy0/scaling_setspeed
```

The script records read-only `/sys/devices/system/cpu/cpufreq/policy*`, `/sys/class/hwmon/hwmon*/temp*_input`, and thermal-zone values. For the A/B test, record actual frequency under FAST-LIVO2 plus RViz load, not only configured values:

| Frequency mode | RViz FPS | RViz CPU | FAST-LIVO2 CPU | Actual CPU frequency | Temperature | Throttled |
| --- | ---: | ---: | ---: | --- | --- | --- |
| Default | | | | | | |
| Official high performance | | | | | | |

## OpenGL

Check `system_snapshot.txt` from the measurement output. If `OpenGL renderer` contains `llvmpipe`, `softpipe`, or `Software Rasterizer`, treat software rendering as a primary bottleneck. Also record whether RViz is running on a local display, VNC, XRDP, SSH X11 forwarding, NoMachine, or an external x86 PC.

## Visualization Parameters

These parameters affect only the point cloud published for RViz:

```yaml
visualization:
  publish_every_n_frames: 1
  voxel_size: 0.20
  max_points: 150000
  publish_rate: 5.0
```

Test `voxel_size` at `0.10`, `0.20`, `0.30`, and `0.50`; test `publish_rate` at original, `10 Hz`, `5 Hz`, and `2 Hz`.

## Final Classification

Use the collected data to distinguish:

| Bottleneck | Evidence pattern |
| --- | --- |
| Historical point cloud accumulation | FPS/RSS worsens over time while `/cloud_registered` points per frame stay stable; Decay 0 recovers FPS |
| Single-frame point count | FPS low immediately; lowering `dense_map_en`, voxel size, or max points improves FPS |
| CPU frequency | High-performance mode improves both RViz and FAST-LIVO2 timing without changing point counts |
| OpenGL software rendering | `glxinfo -B` shows llvmpipe/softpipe/software renderer |
| DDS transport | Topic bandwidth is high; RViz CPU/RSS may be acceptable but subscribers lag or drop |
| FAST-LIVO2 algorithm | `/aft_mapped_to_init` frequency is low even with RViz closed and point visualization minimized |
