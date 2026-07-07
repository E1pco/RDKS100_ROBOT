# Tools 工具集

本目录包含用于传感器同步检查和数据分析的工具脚本。

## 工具列表

### 1. check_sync.py - 实时同步检查

实时检查相机、雷达、IMU的时间同步情况。

**用法:**
```bash
python3 tools/check_sync.py
```

**输出示例:**
```
[ 1] cam-lidar:    0.239ms | cam-imu:    0.827ms | lidar-imu:  150.000ms | time_base: SAME
[ 2] cam-lidar:    0.238ms | cam-imu:    0.826ms | lidar-imu:  149.500ms | time_base: SAME
...
```

### 2. analyze_bag.py - Rosbag 分析

分析rosbag文件中各话题的频率和时间同步情况。

**用法:**
```bash
python3 tools/analyze_bag.py <bag目录>
```

**示例:**
```bash
python3 tools/analyze_bag.py ~/nas/bag/202606242
```

**输出内容:**
- 📊 话题频率统计
- ⏱️ 时间同步分析
- 📈 同步统计信息
- ✅ 同步质量评估
- 🕐 时间基准检查

## 同步质量标准

| 传感器对 | 优秀 | 良好 | 可接受 | 较差 |
|---------|------|------|--------|------|
| 相机-雷达 | < 1ms | < 5ms | < 10ms | > 10ms |
| 相机-IMU | < 1ms | < 5ms | < 10ms | > 10ms |
| 雷达-IMU | < 5ms | < 10ms | < 20ms | > 20ms |

## 注意事项

1. 实时检查需要所有传感器节点正在运行
2. Rosbag分析需要sqlite3支持
3. 时间基准检查可以判断是否使用了硬件同步

### 3. calibrate_mvs_charuco.py - MVS 相机内参标定

使用 ChArUco 棋盘格标定 MVS 相机内参。默认棋盘参数：`8x11`，Checker Size `23 mm`，Marker Size `17 mm`，AruCo 字典 `DICT_4X4_50`。

**实时从 MVS 图像话题采集并标定:**
```bash
zsh -lic 'cd ~/fast_ws && python3 tools/calibrate_mvs_charuco.py --ros --topic /left_camera/image --samples 40 --save-dir /tmp/mvs_charuco_samples --annotated-dir /tmp/mvs_charuco_annotated --output tools/mvs_camera_intrinsics.yaml'
```

**使用已保存图片离线标定:**
```bash
python3 tools/calibrate_mvs_charuco.py   --images /tmp/mvs_charuco_samples/*.png   --annotated-dir /tmp/mvs_charuco_annotated   --output tools/mvs_camera_intrinsics.yaml
```

**输出:**
- `tools/mvs_camera_intrinsics.yaml`: ROS camera_info 风格内参文件
- `tools/mvs_camera_intrinsics_report.json`: 检测角点、重投影误差等详细报告

采集时会打开 OpenCV 窗口。按 `Space` 或 `s` 保存当前有效画面，按 `a` 切换自动采样，按 `q` 或 `Esc` 结束并开始标定。让棋盘格覆盖画面不同位置和角度，建议至少 30-40 张有效样本，边缘和中心都要覆盖。

