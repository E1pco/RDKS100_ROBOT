# FAST-Calib2 ROS2 Humble 校准指南

## 概述

FAST-Calib2 是一个用于大光斑 LiDAR 与相机外参标定的工具。本指南将帮助您在 ROS2 Humble 环境下使用该工具进行传感器标定。

**重要说明**: FAST-Calib2 仅用于 **LiDAR-相机外参标定**，不包含相机内参标定功能。您需要预先标定好相机内参。

## 目录

1. [环境准备](#1-环境准备)
2. [编译安装](#2-编译安装)
3. [相机内参标定](#3-相机内参标定)
4. [标定板准备](#4-标定板准备)
5. [数据采集](#5-数据采集)
6. [LiDAR-相机外参标定](#6-lidar-相机外参标定)
7. [多场景联合标定](#7-多场景联合标定)
8. [结果验证](#8-结果验证)
9. [常见问题](#9-常见问题)

---

## 1. 环境准备

### 1.1 系统要求

- Ubuntu 22.04
- ROS2 Humble
- PCL >= 1.10
- OpenCV >= 4.0
- Eigen3

### 1.2 安装依赖

```bash
# 安装 ROS2 依赖
sudo apt update
sudo apt install -y \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-rosbag2 \
    ros-humble-rviz2

# 安装系统依赖
sudo apt install -y \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev
```

### 1.3 克隆代码

```bash
cd ~/fast_ws/src
git clone https://github.com/xuankuzcr/FAST-Calib2.git
```

---

## 2. 编译安装

### 2.1 编译

```bash
cd ~/fast_ws
colcon build --packages-select fast_calib --symlink-install
source install/setup.bash
```

### 2.2 验证安装

```bash
# 检查可执行文件
ros2 pkg executables fast_calib
```

应该看到：
```
fast_calib fast_calib
fast_calib lidar_center_test
fast_calib multi_fast_calib
```

---

## 3. 相机内参标定

**FAST-Calib2 需要预先标定好的相机内参**。请使用以下工具之一进行相机内参标定：

### 3.1 使用 ROS2 camera_calibration 包

```bash
# 安装
sudo apt install -y ros-humble-camera-calibration

# 运行标定（单目相机）
ros2 run camera_calibration cameracalibrator \
    --size 8x5 \
    --square 0.03 \
    image:=/left_camera/image \
    camera:=/left_camera
```

参数说明：
- `--size 8x5`: 棋盘格内部角点数（列x行）
- `--square 0.0 3`: 每个方格的边长（米）
- `image:=/left_camera/image`: 相机图像话题（根据您的相机配置调整）

### 3.2 使用 OpenCV 标定脚本

```python
#!/usr/bin/env python3
"""
OpenCV 相机内参标定脚本
"""
import cv2
import numpy as np
import glob

# 棋盘格参数
CHECKERBOARD = (8, 6)  # 内部角点数
square_size = 0.0108   # 方格尺寸（米）

# 准备棋盘格角点坐标
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

# 存储角点
obj_points = []
img_points = []

# 读取标定图像
images = glob.glob('calib_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        # 亚像素精度优化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        obj_points.append(objp)
        img_points.append(corners2)

        # 绘制角点
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Calibration', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# 标定
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

print("Camera Matrix (fx, fy, cx, cy):")
print(f"fx: {camera_matrix[0, 0]}")
print(f"fy: {camera_matrix[1, 1]}")
print(f"cx: {camera_matrix[0, 2]}")
print(f"cy: {camera_matrix[1, 2]}")

print("\nDistortion Coefficients (k1, k2, p1, p2):")
print(f"k1: {dist_coeffs[0, 0]}")
print(f"k2: {dist_coeffs[0, 1]}")
print(f"p1: {dist_coeffs[0, 2]}")
print(f"p2: {dist_coeffs[0, 3]}")

# 计算重投影误差
total_error = 0
for i in range(len(obj_points)):
    imgpoints2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i],
                                       camera_matrix, dist_coeffs)
    error = cv2.norm(img_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

print(f"\nMean Reprojection Error: {total_error / len(obj_points)} pixels")
```

### 3.3 标定结果

标定完成后，您将获得以下参数：
- **fx, fy**: 焦距（像素单位）
- **cx, cy**: 主点坐标（像素单位）
- **k1, k2**: 径向畸变系数
- **p1, p2**: 切向畸变系数

---

## 4. 标定板准备

### 4.1 标定板规格

FAST-Calib2 使用定制的反射环形标定板：

- **材质**: PVC 板（厚度至少 1 cm）
- **反射环**: 3M 工程级反光膜
- **标记**: 4 个 ArUco 标记（用于相机检测）
- **反射环**: 4 个环形反射目标（用于 LiDAR 检测）

### 4.2 标定板尺寸参数

在 `config/qr_params.yaml` 中配置：

```yaml
# 标定板参数
marker_size: 0.20           # ArUco 标记尺寸（米）
delta_width_qr_center: 0.55 # 标记中心水平距离的一半
delta_height_qr_center: 0.35 # 标记中心垂直距离的一半
delta_width_circles: 0.5    # 反射环中心水平距离
delta_height_circles: 0.4   # 反射环中心垂直距离
circle_radius: 0.12         # 反射环中线半径
annulus_half_width: 0.025   # 环带半宽：(外径 - 内径) / 2
board_width: 1.4            # 标定板宽度
board_height: 1.0           # 标定板高度
```

### 4.3 ArUco 标记

使用 DICT_6X6_250 字典，标记 ID 为 1, 2, 3, 4。

```python
# 生成 ArUco 标记
import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

for marker_id in [1, 2, 3, 4]:
    marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
    cv2.imwrite(f'aruco_marker_{marker_id}.png', marker_image)
```

---

## 5. 数据采集

### 5.1 采集要求

1. **环境**: 室内或室外，避免强光直射
2. **距离**: LiDAR 距标定板 2-5 米
3. **角度**: 标定板尽量正对相机，倾斜角 < 30°
4. **清晰度**: 图像清晰，无运动模糊
5. **点云密度**: 确保标定板上有足够的 LiDAR 点

### 5.2 采集步骤

```bash
# 1. 启动 LiDAR 和相机驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py

# 2. 录制 rosbag（包含 LiDAR 点云和相机图像）
ros2 bag record \
    /livox/lidar \
    /left_camera/image \
    --output calibration_data

# 3. 从 rosbag 中提取图像（使用 Python 脚本）
# 示例代码：extract_image.py
```

**提取图像的 Python 脚本示例**：

```python
#!/usr/bin/env python3
"""从 rosbag 中提取图像"""
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

def extract_image_from_bag(bag_path, image_topic, output_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == image_topic:
            msg = deserialize_message(data, Image())
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(output_path, cv_image)
            print(f"Saved image to {output_path}")
            break

if __name__ == '__main__':
    rclpy.init()
    extract_image_from_bag(
        bag_path='/path/to/calibration_data',
        image_topic='/left_camera/image',
        output_path='/path/to/image.png'
    )
    rclpy.shutdown()
```

### 5.3 数据格式

- **bag_path**: 包含 LiDAR 点云的 rosbag 文件（`.db3` 格式）
  - LiDAR topic: 支持 `sensor_msgs/PointCloud2` 或 `livox_ros_driver2/CustomMsg`（默认: `/livox/lidar`）
- **image_path**: 标定板图像文件（PNG/BMP/JPG），需要从 rosbag 中提取或单独保存

---

## 6. LiDAR-相机外参标定

### 6.1 单场景标定

#### 6.1.1 配置参数

编辑 `config/qr_params.yaml`：

```yaml
# 相机内参（替换为您的标定结果）
fx: 1215.31801774424
fy: 1214.72961288138
cx: 1047.86571859677
cy: 745.068353101898
k1: -0.33574781188503
k2: 0.10996870793601
p1: 0.000157303079833973
p2: 0.000544930726278493

# LiDAR topic
lidar_topic: "/livox/lidar"

# 数据路径
# bag_path: 包含 LiDAR 点云的 rosbag 文件路径
bag_path: "/path/to/your/calibration_data"
# image_path: 标定板图像文件路径（PNG/BMP/JPG），从 rosbag 中提取或单独保存
image_path: "/path/to/your/image.png"

# 输出路径
output_path: "/path/to/output"
```

#### 6.1.2 运行标定

```bash
# 方法 1: 使用 launch 文件
ros2 launch fast_calib calib_launch.py

# 方法 2: 直接运行
ros2 run fast_calib fast_calib --ros-args --params-file config/qr_params.yaml
```

#### 6.1.3 输出结果

标定结果保存在 `output_path` 目录下：

- `single_calib_result.txt`: 标定结果（FAST-LIVO2 格式）
- `colored_cloud.pcd`: 彩色点云（LiDAR 点投影到图像）
- `qr_detect.png`: ArUco 标记检测结果
- `circle_center_record.txt`: 圆心坐标记录

结果文件示例：
```
# FAST-LIVO2 calibration format
cam_model: Pinhole
cam_width: 1920
cam_height: 1080
scale: 1.0
cam_fx: 1215.31801774424
cam_fy: 1214.72961288138
cam_cx: 1047.86571859677
cam_cy: 745.068353101898
cam_d0: -0.33574781188503
cam_d1: 0.10996870793601
cam_d2: 0.000157303079833973
cam_d3: 0.000544930726278493

Rcl: [  0.999876,  0.015234, -0.004567,
        -0.015198,  0.999852,  0.008234,
         0.004692, -0.008167,  0.999956]
Pcl: [ -0.012345,  0.023456,  0.034567]
```

### 6.2 使用 LiDAR 圆心测试工具

```bash
# 测试单个 bag 文件
ros2 run fast_calib lidar_center_test /path/to/bag /livox/lidar solid

# 参数说明：
# - bag_path: rosbag 文件路径
# - lidar_topic: LiDAR 点云话题
# - LiDAR 类型: solid (固态) 或 mech (机械式)
```

---

## 7. 多场景联合标定

### 7.1 采集多场景数据

建议采集 3 组以上不同角度/距离的数据：

```bash
# 场景 1
ros2 bag record /livox/lidar /left_camera/image --output scene_1

# 场景 2（不同角度）
ros2 bag record /livox/lidar /left_camera/image --output scene_2

# 场景 3（不同距离）
ros2 bag record /livox/lidar /left_camera/image --output scene_3
```

### 7.2 运行单场景标定

对每个场景分别运行单场景标定：

```bash
# 场景 1
ros2 run fast_calib fast_calib --ros-args \
    -p bag_path:=/path/to/scene_1 \
    -p image_path:=/path/to/scene_1/image.png \
    -p output_path:=/path/to/output/scene_1

# 场景 2
ros2 run fast_calib fast_calib --ros-args \
    -p bag_path:=/path/to/scene_2 \
    -p image_path:=/path/to/scene_2/image.png \
    -p output_path:=/path/to/output/scene_2

# 场景 3
ros2 run fast_calib fast_calib --ros-args \
    -p bag_path:=/path/to/scene_3 \
    -p image_path:=/path/to/scene_3/image.png \
    -p output_path:=/path/to/output/scene_3
```

### 7.3 合并结果

将所有场景的 `circle_center_record.txt` 合并到一个文件：

```bash
cat /path/to/output/scene_1/circle_center_record.txt \
    /path/to/output/scene_2/circle_center_record.txt \
    /path/to/output/scene_3/circle_center_record.txt \
    > /path/to/output/circle_center_record.txt
```

### 7.4 运行多场景标定

```bash
ros2 launch fast_calib multi_calib_launch.py
```

或直接运行：

```bash
ros2 run fast_calib multi_fast_calib --ros-args \
    -p output_path:=/path/to/output
```

---

## 8. 结果验证

### 8.1 RMSE 检查

标定结果中的 RMSE 值应该尽量小：
- **优秀**: RMSE < 0.01 m (1 cm)
- **良好**: RMSE < 0.02 m (2 cm)
- **可接受**: RMSE < 0.05 m (5 cm)

### 8.2 可视化验证

```bash
# 查看彩色点云
pcl_viewer /path/to/output/colored_cloud.pcd

# 或使用 RViz2
ros2 launch fast_calib calib_launch.py rviz:=true
```

### 8.3 几何一致性检查

输出结果中的几何误差应该满足：
- 最大误差 < 10 mm
- RMSE < 5 mm

### 8.4 重投影验证

使用标定结果将 LiDAR 点投影到图像上，检查对齐精度：

```python
#!/usr/bin/env python3
"""
验证 LiDAR-相机外参标定结果
"""
import cv2
import numpy as np

# 读取标定结果
# Rcl: 旋转矩阵
# Pcl: 平移向量
Rcl = np.array([
    [0.999876, 0.015234, -0.004567],
    [-0.015198, 0.999852, 0.008234],
    [0.004692, -0.008167, 0.999956]
])
Pcl = np.array([-0.012345, 0.023456, 0.034567])

# 相机内参
K = np.array([
    [1215.318, 0, 1047.865],
    [0, 1214.730, 745.068],
    [0, 0, 1]
])

# 畸变系数
dist = np.array([-0.3357, 0.1099, 0.000157, 0.000544, 0])

# 读取 LiDAR 点云和图像
# ... 实现省略

# 投影 LiDAR 点到图像
# u = (fx * X' + cx * Z') / Z'
# v = (fy * Y' + cy * Z') / Z'
# 其中 [X', Y', Z'] = Rcl * [X, Y, Z] + Pcl
```

---

## 9. 常见问题

### 9.1 编译错误

**问题**: `Could not find a package configuration file provided by "livox_ros_driver2"`

**解决**:
```bash
cd ~/fast_ws
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

### 9.2 运行时错误

**问题**: `Loading the rosbag failed`

**解决**:
1. 检查 bag 文件路径是否正确
2. 确认 bag 文件格式为 ROS2 (sqlite3)
3. 如果是 ROS1 bag，需要转换：
   ```bash
   ros2 bag convert input.bag --output-format sqlite3 --output output
   ```

### 9.3 检测失败

**问题**: `Unable to find a candidate set that matches target's geometry`

**解决**:
1. 检查标定板参数是否正确
2. 确保 ArUco 标记清晰可见
3. 调整 `delta_width_circles` 和 `delta_height_circles` 参数
4. 确保 LiDAR 点云质量良好

### 9.4 精度问题

**问题**: RMSE 过大

**解决**:
1. 采集更多场景数据（至少 3 组）
2. 确保相机内参标定准确
3. 检查标定板是否平整
4. 避免标定板反光不均匀

### 9.5 点云质量问题

**问题**: LiDAR 点云噪声大

**解决**:
1. 调整 `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max` 参数
2. 设置 `use_auto_lidar_roi: true` 启用自动 ROI
3. 使用反射率更高的标定板

---

## 附录 A: 配置文件完整示例

```yaml
# Camera intrinsics
fx: 1215.31801774424
fy: 1214.72961288138
cx: 1047.86571859677
cy: 745.068353101898
k1: -0.33574781188503
k2: 0.10996870793601
p1: 0.000157303079833973
p2: 0.000544930726278493

# Calibration target parameters
marker_size: 0.20
delta_width_qr_center: 0.55
delta_height_qr_center: 0.35
delta_width_circles: 0.5
delta_height_circles: 0.4
circle_radius: 0.12
annulus_half_width: 0.025
board_width: 1.4
board_height: 1.0
board_roi_margin: 0.08
board_roi_depth: 0.12
auto_roi_voxel_leaf: 0.01
annulus_voxel_leaf: 0.005

# Distance filter (adjust based on your setup)
use_auto_lidar_roi: true
x_min: 2.0
x_max: 5.0
y_min: -1.0
y_max: 1.0
z_min: -0.5
z_max: 2.0

# Input
lidar_topic: "/livox/lidar"
bag_path: "/path/to/your/calibration_data"
image_path: "/path/to/your/image.png"

# Output
output_path: "/path/to/output"
```

---

## 附录 B: 支持的 LiDAR 类型

| LiDAR 型号 | 类型 | topic 示例 |
|-----------|------|-----------|
| Livox Mid-360 | 固态 | /livox/lidar |
| Livox Avia | 固态 | /livox/lidar |
| Ouster OS1 | 机械式 | /ouster/points |
| Velodyne VLP-16 | 机械式 | /velodyne_points |
| Hesai XT32 | 机械式 | /hesai/pandar |

---

## 附录 C: 参考资料

- [FAST-Calib2 GitHub](https://github.com/xuankuzcr/FAST-Calib2)
- [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)
- [ROS2 camera_calibration](http://wiki.ros.org/camera_calibration)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

