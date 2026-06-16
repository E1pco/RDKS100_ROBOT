# FAST Nav2 Bringup

This package starts Nav2 with an existing 2D occupancy map and the local
Ackermann serial bridge.

## Build

```bash
cd /home/sunrise/fast_ws
colcon build --symlink-install --packages-select fast_nav2_bringup
source install/setup.zsh
```

## Required runtime topics and TF

- `/map`: published by Nav2 `map_server` from a `.yaml/.pgm` map.
- `/scan`: `sensor_msgs/msg/LaserScan` for AMCL and obstacle layers.
- `/odom`: wheel odometry from `ackermann_serial_bridge`.
- `/cmd_vel`: Nav2 command output consumed by `ackermann_serial_bridge`.
- TF chain: `map -> odom -> base_link -> livox`.

Set `publish_tf: true` in
`ackermann_serial_bridge/config/ackermann_serial_bridge.yaml`, or provide
`odom -> base_link` from another localization/robot state source.

## Start Navigation

```bash
ros2 launch fast_nav2_bringup nav2.launch.py \
  map:=/home/sunrise/fast_ws/src/fast_nav2_bringup/maps/fastlivo_2d.yaml
```

This also starts RViz by default. Disable it with `use_rviz:=false`.

## Start Navigation With FAST-LIVO Localization

Use this mode when FAST-LIVO2 is running and publishing
`camera_init -> aft_mapped`. It does not start AMCL and does not require an
initial pose in RViz.

```bash
ros2 launch fast_nav2_bringup fastlivo_nav2.launch.py
```

This mode uses `camera_init` as the Nav2 global frame, publishes an identity
`aft_mapped -> base_link` transform, uses Smac Hybrid-A* for global planning,
and uses Regulated Pure Pursuit for path tracking.

## Convert FAST-LIVO2 PCD

The generated map in `maps/fastlivo_2d.yaml` was created without rotating the
point cloud. It treats the LiDAR/IMU origin as `base_link`.

```bash
src/fast_nav2_bringup/scripts/pcd_to_occupancy_map.py \
  --pcd src/FASTLIVO2_ROS2/Log/PCD/all_downsampled_points.pcd \
  --output src/fast_nav2_bringup/maps/fastlivo_2d.yaml \
  --resolution 0.05 \
  --crop-percentile 1 \
  --ground-percentile 25 \
  --free-min -0.25 \
  --free-max 0.25 \
  --occupied-min 0.30 \
  --occupied-max 2.00 \
  --occupied-dilation 0
```

If using MID360 point clouds as AMCL input, install `pointcloud_to_laserscan` and
start:

```bash
ros2 launch fast_nav2_bringup pointcloud_to_scan.launch.py
```

The launch publishes an identity static transform from `base_link` to
`livox_frame` because `base_link` is the LiDAR/IMU frame in this setup. If the
actual cloud frame is `livox`, use:

```bash
ros2 launch fast_nav2_bringup pointcloud_to_scan.launch.py \
  lidar_frame:=livox
```

If the Livox driver is already publishing `sensor_msgs/msg/PointCloud2`, skip the
custom-message converter:

```bash
ros2 launch fast_nav2_bringup pointcloud_to_scan.launch.py \
  convert_custom:=false \
  cloud_topic:=/livox/lidar
```
