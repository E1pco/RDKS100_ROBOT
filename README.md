# RDKS100_ROBOT

本仓库是 RDK S100 机器人本体侧工作空间，面向移动底盘、机械臂、视觉感知、MID360 激光雷达、MVS 工业相机、FAST-LIVO2 建图定位与 Nav2 自主导航的一体化运行。工程以 ROS 2 Humble / TROS Humble 为基础，强调硬件同步、节点解耦、按需启动和可维护的导航数据流。

## 1. 系统定位

系统以 RDK S100 作为机器人本体计算节点，负责传感器采集、底盘串口通信、机械臂透传、视觉推理、点云处理和 FAST-LIVO2 状态估计。Web 中控台或上位机通过 ROS 2 topic 与本体交互，不直接持有底层硬件资源。

核心设计原则：

- 常驻节点只保留状态上报、功能管理和必要的数据转发。
- 视频、分割、机械臂透传、雷达显示、MVS 相机、FAST-LIVO2 等高负载功能按需启动。
- FAST-LIVO2 使用 MID360 + MVS 硬同步数据进行建图/定位，Web 显示只使用轻量压缩流或原始雷达点云，不抢占建图链路。
- 导航链路以 FAST-LIVO2 位姿和地图数据为主，Nav2 负责路径规划与运动控制。

## 2. 目录结构

| 路径 | 说明 |
| --- | --- |
| `MCU/` | 硬同步 MCU 工程，输出雷达与相机触发信号。 |
| `src/arm/` | S100 本体核心管理节点、USB 视频流、MVS JPEG 转发、机械臂透传与功能开关管理。 |
| `src/ackermann_serial_bridge/` | 阿克曼底盘串口协议、`/cmd_vel` 下发、底盘状态与 `/odom` 发布。 |
| `src/livox_ros_driver2/` | MID360 驱动及 Web/FAST-LIVO 两类点云启动入口。 |
| `src/mvs_ros_driver/` | MVS 工业相机触发采集节点。 |
| `src/FASTLIVO2_ROS2/` | FAST-LIVO2 建图、定位、MID360+MVS 配置和 RViz 配置。 |
| `src/fast_nav2_bringup/` | Nav2、点云转换、重定位、导航参数和机器人外参配置。 |
| `src/mono_edgesam/` | DOSOD + EdgeSAM 实时检测/分割链路。 |
| `src/hobot_clip/` | D-Robotics CLIP 相关模型与节点。 |
| `src/FAST-Calib2/` | 激光雷达与相机外参标定工具。 |
| `tools/` | 辅助工具。 |

## 3. 环境约定

目标设备：

- RDK S100：机器人本体侧，默认工作空间 `~/fast_ws`。
- ROS 2 / TROS：Humble。
- 224 机器默认 shell 为 `zsh`，工程中多数启动命令假定已自动 source ROS/TROS 环境。

如终端没有自动加载环境，手动执行：

```bash
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh
cd ~/fast_ws
source install/setup.zsh
```

若使用 `bash`：

```bash
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash
cd ~/fast_ws
source install/setup.bash
```

FAST-LIVO2 相关 launch 已设置 TROS Humble zero-copy / Fast DDS SHM 环境变量：

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
RMW_FASTRTPS_USE_QOS_FROM_XML=1
ROS_DISABLE_LOANED_MESSAGES=0
```

注意：zero-copy 只适合同机进程间大数据通信，不用于跨板传输。

## 4. 构建

完整构建：

```bash
cd ~/fast_ws
colcon build --symlink-install
source install/setup.zsh
```

仅构建当前核心链路：

```bash
cd ~/fast_ws
colcon build --symlink-install --packages-up-to \
  arm \
  ackermann_serial_bridge \
  livox_ros_driver2 \
  mvs_ros_driver \
  fast_livo \
  fast_nav2_bringup \
  mono_edgesam
source install/setup.zsh
```

## 5. 设备命名

仓库根目录提供 `99-arm-chassis.rules`，用于固定机械臂和底盘串口名称：

- 机械臂：`/dev/arm`
- 底盘：`/dev/chassis`

安装规则：

```bash
sudo cp 99-arm-chassis.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

底盘串口配置位于：

```text
src/ackermann_serial_bridge/config/ackermann_serial_bridge.yaml
```

关键参数：

- `port: /dev/chassis`
- `baudrate: 115200`
- `cmd_send_rate: 5.0`
- `read_rate: 100.0`
- `command_timeout: 5.0`
- `publish_odom: true`
- `publish_tf: false`

## 6. 本体核心启动

推荐使用 S100 本体核心启动文件：

```bash
ros2 launch arm s100_robot_core.launch.py
```

兼容旧入口：

```bash
ros2 launch arm arm_bringup.launch.py
```

默认启动内容：

- `s100_feature_manager`：监听 `/control_console/feature_command`，按需启动高负载功能。
- `rgb_img_republisher`：将 `/left_camera/image` 转发为 `/rgb_img`。
- `ackermann_serial_bridge`：默认随核心启动，可通过 `enable_chassis:=false` 禁用。

暂不启动底盘控制时：

```bash
ros2 launch arm s100_robot_core.launch.py enable_chassis:=false
```

核心 launch 启动前会清理历史残留的相关进程，避免重复持有相机、雷达、串口或 Web 端口。

## 7. Web 功能管理接口

中控台通过 `/control_console/feature_command` 控制本体高负载功能，消息类型为 `std_msgs/msg/String`，内容为 JSON：

```json
{"feature":"video_stream","action":"start"}
```

支持的 `feature`：

| feature | 作用 |
| --- | --- |
| `video_stream` | 启动 USB camera 原始视频流与 JPEG/WebSocket 输出。 |
| `segmentation` | 启动 DOSOD + EdgeSAM 分割链路。会自动拉起 `video_stream`。 |
| `arm_passthrough` | 启动机械臂从臂透传节点。 |
| `lidar` | 启动 Web 显示用 MID360 原始 PointCloud2 输出。 |
| `mvs_camera` | 启动 MVS 相机触发采集与 JPEG 轻量转发。 |
| `fast_livo2` | 启动 FAST-LIVO2 依赖链路，包括 MID360 raw driver、MVS camera 和 FAST-LIVO2 mapping。 |

关闭功能：

```json
{"feature":"video_stream","action":"stop"}
```

功能状态发布到：

```text
/control_console/feature_status
```

FAST-LIVO2 启动时会独占建图所需的 MID360 与 MVS 相机链路；Web 雷达显示和建图雷达驱动不要同时抢占同一数据源。

## 8. 视频、分割与 MVS 显示

USB camera 原始视频流：

```bash
ros2 launch arm video_stream.launch.py device:=/dev/video0
```

分割链路：

```bash
ros2 launch arm segmentation_stream.launch.py
```

分割链路订阅 `/camera/image_raw`，并输出 Web 可显示的 `/segmentation/image_jpeg`。

MVS 相机触发采集：

```bash
ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py
```

Web 轻量 MVS 图像转发：

```bash
ros2 run arm mvs_jpeg_republisher_node --ros-args \
  -p input_topic:=/left_camera/image \
  -p output_topic:=/mvs/image_jpeg \
  -p max_width:=640 \
  -p jpeg_quality:=45 \
  -p fps:=5.0
```

MVS 视频用于状态观察时应使用 JPEG 降帧链路，避免在 Web 端直接传输高分辨率 raw image 导致堆积延迟。

## 9. MID360 点云

Web 原始点云显示使用 PointCloud2 模式：

```bash
ros2 launch livox_ros_driver2 web_pointcloud2_MID360_launch.py
```

FAST-LIVO2 使用 Livox CustomMsg 模式：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

两者用途不同：

- Web 原始点云：用于中控台 3D 显示，可按距离伪彩、限点数和点径显示。
- FAST-LIVO2 点云：用于建图和定位，不应被 Web 显示链路改写 QoS、frame 或数据格式。

## 10. FAST-LIVO2

推荐 MID360 + MVS 建图入口：

```bash
ros2 launch fast_livo mapping_mid360.launch.py
```

该 launch 默认使用：

```text
src/FASTLIVO2_ROS2/config/mid360_mvs.yaml
src/FASTLIVO2_ROS2/config/camera_mvs.yaml
```

导航定位轻量入口：

```bash
ros2 launch fast_livo mapping_mid360_mvs_nav.launch.py
```

该 launch 默认使用：

```text
src/FASTLIVO2_ROS2/config/mid360_mvs_nav.yaml
src/FASTLIVO2_ROS2/config/camera_mvs.yaml
```

历史兼容入口仍可显式传参：

```bash
ros2 launch fast_livo mapping_avia.launch.py \
  avia_params_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/config/mid360_mvs.yaml \
  camera_params_file:=/home/sunrise/fast_ws/src/FASTLIVO2_ROS2/config/camera_mvs.yaml \
  use_rviz:=0
```

硬同步约定：

- MCU 输出两路同相位触发信号。
- MID360 上位机应显示 GPS sync。
- 不建议在 FAST-LIVO2 内人为给 IMU 或 LiDAR 添加时间 offset；此前测试表明 offset 会破坏建图效果。

FAST-LIVO2 建图结果默认应保存可用于重定位和导航的点云数据。高密度地图用于离线保存和重定位，在线 Web 可视化不直接订阅 FAST-LIVO2 稠密建图点云。

## 11. 自主导航

导航相关包：

```text
src/fast_nav2_bringup
```

FAST-LIVO2 定位启动：

```bash
ros2 launch fast_livo mapping_mid360_mvs_nav.launch.py
```

Nav2 启动：

```bash
ros2 launch fast_nav2_bringup fastlivo_nav2.launch.py
```

点云转导航输入：

```bash
ros2 launch fast_nav2_bringup pointcloud_to_scan.launch.py
```

PCD 重定位节点：

```bash
ros2 launch fast_nav2_bringup pcd_relocalization.launch.py
```

关键配置：

| 文件 | 说明 |
| --- | --- |
| `config/nav2_fastlivo_params.yaml` | Nav2 参数，面向 FAST-LIVO2 全局坐标系。 |
| `config/pointcloud_to_scan.yaml` | 点云转 2D 障碍输入参数。 |
| `config/pcd_relocalization.yaml` | PCD 重定位配置。 |
| `config/robot_extrinsics.yaml` | 小车、雷达、机械臂几何关系。 |

几何关系以小车几何中心为原点建立右手系：

- 雷达几何中心：`(189.5, 0.0, 162.0) mm`
- 机械臂几何中心：`(-66.5, -64.0, 120.0) mm`
- 雷达相对车体绕 X 轴约 `-22 deg`
- 机械臂相对车体绕 Z 轴约 `-43.7 deg`

底盘 `/odom` 由 `ackermann_serial_bridge` 发布；FAST-LIVO2 输出定位链路。导航使用时需要确保 TF 树中 `map/camera_init -> odom/base_link` 的职责清晰，避免底盘 odom 与 FAST-LIVO2 位姿重复发布同一段 TF。

## 12. 常用检查

查看节点：

```bash
ros2 node list
```

查看 topic：

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic hz /left_camera/image
ros2 topic hz /odom
```

查看 TF：

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link livox_frame
```

查看 feature manager 状态：

```bash
ros2 topic echo /control_console/feature_status
```

查看功能日志：

```bash
ls /tmp/s100_*.log
tail -f /tmp/s100_fast_livo2.log
```

## 13. 维护规则

- 不提交 `build/`、`install/`、`log/`、bag、PCD 大数据、临时调试日志和 `.bak/.codex` 文件。
- 模型权重体积较大，后续建议迁移到 Git LFS 或发布包管理。
- Web 跨板传输只使用 JPEG/MJPEG/WebSocket 等压缩数据；raw image 和 FAST-LIVO2 稠密点云只在本机内部链路使用。
- FAST-LIVO2 的 QoS、队列深度、时间戳和同步策略会直接影响建图质量，修改前应记录测试条件并保留可回退提交。
- MCU 硬同步代码位于 `MCU/`，如需修改应单独评审，避免与 ROS 应用层改动混杂。

## 14. 推荐启动顺序

基础遥控/中控：

```bash
ros2 launch arm s100_robot_core.launch.py
```

建图：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py
ros2 launch fast_livo mapping_mid360.launch.py
```

导航：

```bash
ros2 launch arm s100_robot_core.launch.py
ros2 launch fast_livo mapping_mid360_mvs_nav.launch.py
ros2 launch fast_nav2_bringup fastlivo_nav2.launch.py
```

Web 显示原始雷达和 MVS：

```bash
ros2 launch arm s100_robot_core.launch.py
```

然后由中控台按钮按需启动 `lidar` 和 `mvs_camera`。
