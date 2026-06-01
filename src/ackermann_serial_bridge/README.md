# ackermann_serial_bridge — WHEELTEC 阿克曼底盘 ROS2 串口桥接包

## 目录

- [1. 概述](#1-概述)
- [2. 功能特性](#2-功能特性)
- [3. 工程结构](#3-工程结构)
- [4. 环境准备与安装](#4-环境准备与安装)
- [5. 编译与启动](#5-编译与启动)
- [6. 参数说明](#6-参数说明)
- [7. 话题与消息类型](#7-话题与消息类型)
- [8. 串口协议详解](#8-串口协议详解)
- [9. 里程计算法说明](#9-里程计算法说明)
- [10. yaw_sign 角速度方向约定](#10-yaw_sign-角速度方向约定)
- [11. 安全机制](#11-安全机制)
- [12. 诊断信息](#12-诊断信息)
- [13. 使用示例](#13-使用示例)
- [14. 常见问题排查](#14-常见问题排查)
- [15. 单元测试](#15-单元测试)
- [16. 代码模块说明](#16-代码模块说明)
- [17. 许可证](#17-许可证)

---

## 1. 概述

本包是一个 ROS2 Humble Python 节点，用于将 **WHEELTEC（轮趣）阿克曼底盘** 通过串口（UART3）连接到 ROS2 上位机。

底盘侧由 STM32 控制，负责电机驱动和阿克曼前轮转角计算；上位机侧运行本节点，负责：

- **上行**（底盘 → 上位机）：读取 STM32 发送的 24 字节状态反馈帧，发布里程计、电池状态和诊断信息。
- **下行**（上位机 → 底盘）：接收 ROS2 `/cmd_vel` 速度指令，打包为 11 字节控制帧发送给 STM32。

> **当前版本说明**：上行帧中的 IMU 加速度和角速度字段仅做解析和校验，**不发布 `sensor_msgs/msg/Imu` 话题**。

---

## 2. 功能特性

| 功能 | 说明 |
|------|------|
| 上行帧解析 | 24 字节状态帧，含帧头/帧尾/BCC 校验 |
| 下行帧构建 | 11 字节速度指令帧，自动计算 BCC |
| 流式解析器 | 处理粘包、半包、错包，自动重新同步 |
| 里程计发布 | 基于 vx/wz 的二维积分，支持四元数和协方差 |
| TF 广播 | 可选发布 odom → base_link 变换 |
| 电池状态 | 发布电压信息 |
| 诊断信息 | 串口状态、帧计数、错误计数、电机状态等 |
| 超时保护 | 无指令超时自动发送零速 |
| 关闭保护 | 节点退出时发送 3 次零速帧确保底盘停止 |
| 自动重连 | 串口断开后自动尝试重新打开 |
| 参数可配置 | 串口路径、波特率、速度限幅、角速度方向等 |

---

## 3. 工程结构

```
ackermann_serial_bridge/
├── package.xml                              # ROS2 包清单
├── setup.py                                 # Python 包构建脚本
├── setup.cfg                                # 安装/测试配置
├── pytest.ini                               # pytest 测试发现配置
├── resource/
│   └── ackermann_serial_bridge              # ament 索引标记文件（空文件）
├── config/
│   └── ackermann_serial_bridge.yaml         # 默认参数配置文件
├── launch/
│   └── serial_bridge.launch.py              # ROS2 launch 启动文件
├── ackermann_serial_bridge/                 # Python 包源码
│   ├── __init__.py
│   ├── protocol.py                          # 纯协议层（无 ROS 依赖）
│   └── serial_bridge_node.py                # ROS2 节点主逻辑
├── test/
│   ├── __init__.py
│   └── test_protocol.py                     # 31 项单元测试
└── README.md                                # 本文档
```

---

## 4. 环境准备与安装

### 4.1 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### 4.2 安装依赖

```bash
# 方式一：apt 安装（推荐）
sudo apt install python3-serial

# 方式二：pip 安装
pip install pyserial
```

### 4.3 串口权限配置

普通用户默认无法访问串口设备，需要将当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
```

> **重要**：执行上述命令后，必须**注销并重新登录**才能生效。可通过以下命令确认：
>
> ```bash
> groups $USER
> ```
>
> 输出中应包含 `dialout`。

### 4.4 确认串口设备

连接底盘 USB 线后，确认设备存在：

```bash
ls -la /dev/ttyACM*
```

默认设备为 `/dev/ttyACM0`。如果系统中有多个 ACM 设备，可通过 `dmesg | tail` 查看最近插入的设备路径。

---

## 5. 编译与启动

### 5.1 编译

```bash
cd ~/fast_ws
colcon build --packages-select ackermann_serial_bridge
source install/setup.bash
```

### 5.2 启动节点

**方式一：使用 launch 文件（推荐）**

```bash
ros2 launch ackermann_serial_bridge serial_bridge.launch.py
```

launch 文件会自动加载 `config/ackermann_serial_bridge.yaml` 中的默认参数。

**方式二：直接运行节点**

```bash
ros2 run ackermann_serial_bridge serial_bridge_node
```

**方式三：运行时覆盖参数**

```bash
ros2 run ackermann_serial_bridge serial_bridge_node --ros-args \
  -p port:=/dev/ttyACM1 \
  -p baudrate:=115200 \
  -p yaw_sign:=-1.0
```

**方式四：使用自定义配置文件**

```bash
ros2 launch ackermann_serial_bridge serial_bridge.launch.py \
  config:=/path/to/my_config.yaml
```

---

## 6. 参数说明

所有参数均在节点初始化时声明，可通过 YAML 文件或命令行动态设置。

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | int | `115200` | 波特率 |
| `cmd_send_rate` | double | `20.0` | 下行发送频率（Hz） |
| `read_rate` | double | `100.0` | 上行读取频率（Hz） |
| `command_timeout` | double | `0.5` | 指令超时时间（秒），超时后自动发送零速 |
| `max_linear_speed` | double | `1.0` | 最大线速度限幅（m/s） |
| `max_angular_speed` | double | `2.0` | 最大角速度限幅（rad/s） |
| `yaw_sign` | double | `-1.0` | 角速度方向修正系数（详见[第 10 节](#10-yaw_sign-角速度方向约定)） |
| `publish_odom` | bool | `true` | 是否发布 `/odom` 里程计话题 |
| `publish_tf` | bool | `false` | 是否发布 odom → base_link TF 变换 |
| `frame_id` | string | `odom` | 里程计父坐标系名称 |
| `base_frame_id` | string | `base_link` | 里程计子坐标系名称 |

### 参数配置文件示例

```yaml
ackermann_serial_bridge:
  ros__parameters:
    port: "/dev/ttyACM0"
    baudrate: 115200
    cmd_send_rate: 20.0
    read_rate: 100.0
    command_timeout: 0.5
    max_linear_speed: 1.0
    max_angular_speed: 2.0
    yaw_sign: -1.0
    publish_odom: true
    publish_tf: false
    frame_id: "odom"
    base_frame_id: "base_link"
```

---

## 7. 话题与消息类型

### 7.1 发布的话题

| 话题名 | 消息类型 | QoS | 说明 |
|--------|----------|-----|------|
| `/odom` | `nav_msgs/msg/Odometry` | Best-Effort | 轮式里程计（位置、姿态、速度） |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Best-Effort | 电池电压 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Reliable | 节点诊断信息 |

### 7.2 订阅的话题

| 话题名 | 消息类型 | QoS | 说明 |
|--------|----------|-----|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable | 速度控制指令 |

### 7.3 TF 变换

当 `publish_tf` 设为 `true` 时，发布：

```
odom → base_link
```

### 7.4 /cmd_vel 使用说明

对于阿克曼车型，`/cmd_vel` 的各字段含义如下：

| 字段 | 含义 | 说明 |
|------|------|------|
| `linear.x` | 前进速度（m/s） | 正值前进，负值后退 |
| `linear.y` | 横向速度 | **忽略**，阿克曼不支持横移 |
| `linear.z` | 垂直速度 | **忽略** |
| `angular.x` | 绕 X 轴角速度 | **忽略** |
| `angular.y` | 绕 Y 轴角速度 | **忽略** |
| `angular.z` | 绕 Z 轴角速度（rad/s） | 正值逆时针旋转（ROS 标准约定） |

> **注意**：底盘 STM32 会根据 `linear.x` 和 `angular.z` 自动计算阿克曼前轮转角，上位机**不需要**直接发送舵机角度。

---

## 8. 串口协议详解

### 8.1 串口参数

| 参数 | 值 |
|------|-----|
| 数据位 | 8 |
| 校验位 | None |
| 停止位 | 1 |
| 流控 | None |
| 读取超时 | 非阻塞（timeout=0） |
| 写入超时 | 0.5 秒 |

### 8.2 上行帧格式（底盘 → 上位机，24 字节）

```
┌──────┬───────────┬────────┬────────┬────────┬─────────┬─────────┬─────────┬─────────┬─────────┬─────────┬──────────┬──────┬──────┐
│ 字节 │     0     │   1    │  2-3   │  4-5   │   6-7   │  8-13   │ 14-19   │  20-21  │   22    │   23    │          │      │      │
├──────┼───────────┼────────┼────────┼────────┼─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤          │      │      │
│ 内容 │  帧头     │flag    │ X 轴   │ Y 轴   │  Z 轴   │ 加速度  │ 角速度  │ 电池    │  BCC    │  帧尾   │          │      │      │
│      │  0x7B     │_stop   │ 速度   │ 速度   │  速度   │ (IMU)   │ (IMU)   │ 电压    │  校验   │  0x7D   │          │      │      │
│ 类型 │  uint8    │ uint8  │ int16  │ int16  │  int16  │ 3×int16 │ 3×int16 │  int16  │  uint8  │  uint8  │          │      │      │
│ 单位 │   —       │   —    │ mm/s   │ mm/s   │ mrad/s  │  raw    │  raw    │   mV    │   —     │   —     │          │      │      │
└──────┴───────────┴────────┴────────┴────────┴─────────┴─────────┴─────────┴─────────┴─────────┴─────────┴──────────┴──────┴──────┘
```

**字段详解：**

| 字节索引 | 字段名 | 类型 | 字节序 | 说明 |
|----------|--------|------|--------|------|
| 0 | 帧头 | uint8 | — | 固定 `0x7B` |
| 1 | flag_stop | uint8 | — | 电机使能标志：`0x00` = 电机使能，其他值 = 电机失能 |
| 2-3 | x_mm_s | int16 | Big-Endian | X 轴线速度，单位 mm/s |
| 4-5 | y_mm_s | int16 | Big-Endian | Y 轴线速度，单位 mm/s |
| 6-7 | z_mrad_s | int16 | Big-Endian | Z 轴角速度，单位 mrad/s（rad/s × 1000） |
| 8-9 | acc_x_raw | int16 | Big-Endian | X 轴加速度原始值（IMU） |
| 10-11 | acc_y_raw | int16 | Big-Endian | Y 轴加速度原始值（IMU） |
| 12-13 | acc_z_raw | int16 | Big-Endian | Z 轴加速度原始值（IMU） |
| 14-15 | gyro_x_raw | int16 | Big-Endian | X 轴角速度原始值（IMU） |
| 16-17 | gyro_y_raw | int16 | Big-Endian | Y 轴角速度原始值（IMU） |
| 18-19 | gyro_z_raw | int16 | Big-Endian | Z 轴角速度原始值（IMU） |
| 20-21 | battery_mv | int16 | Big-Endian | 电池电压，单位 mV |
| 22 | BCC | uint8 | — | 前 22 字节逐字节异或校验 |
| 23 | 帧尾 | uint8 | — | 固定 `0x7D` |

**单位转换：**

```python
vx_mps        = x_mm_s / 1000.0        # mm/s → m/s
vy_mps        = y_mm_s / 1000.0        # mm/s → m/s
serial_wz     = z_mrad_s / 1000.0      # mrad/s → rad/s
voltage       = battery_mv / 1000.0    # mV → V
```

**BCC 校验算法：**

```python
bcc = byte[0] ^ byte[1] ^ byte[2] ^ ... ^ byte[21]
```

### 8.3 下行帧格式（上位机 → 底盘，11 字节）

```
┌──────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬──────┬──────┐
│ 字节 │   0    │   1    │   2    │  3-4   │  5-6   │  7-8   │   9    │  10  │      │
├──────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼──────┤      │
│ 内容 │ 帧头   │ 预留   │ 预留   │ X 轴   │ Y 轴   │ Z 轴   │  BCC   │ 帧尾 │      │
│      │ 0x7B   │ 0x00   │ 0x00   │ 目标速度│ 目标速度│ 目标速度│  校验  │ 0x7D │      │
│ 类型 │ uint8  │ uint8  │ uint8  │ int16  │ int16  │ int16  │ uint8  │uint8 │      │
│ 单位 │   —    │   —    │   —    │ mm/s   │ mm/s   │ mrad/s │   —    │  —   │      │
└──────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴──────┴──────┘
```

**字段详解：**

| 字节索引 | 字段名 | 类型 | 字节序 | 说明 |
|----------|--------|------|--------|------|
| 0 | 帧头 | uint8 | — | 固定 `0x7B` |
| 1 | 预留 | uint8 | — | 固定 `0x00` |
| 2 | 预留 | uint8 | — | 固定 `0x00` |
| 3-4 | x_mm_s | int16 | Big-Endian | X 轴目标速度，单位 mm/s |
| 5-6 | y_mm_s | int16 | Big-Endian | Y 轴目标速度，**阿克曼车型固定为 0** |
| 7-8 | z_mrad_s | int16 | Big-Endian | Z 轴目标角速度，单位 mrad/s（rad/s × 1000） |
| 9 | BCC | uint8 | — | 前 9 字节逐字节异或校验 |
| 10 | 帧尾 | uint8 | — | 固定 `0x7D` |

**ROS → 协议单位转换：**

```python
x_mm_s   = int(linear.x * 1000)                  # m/s → mm/s
y_mm_s   = 0                                      # 阿克曼固定为 0
z_mrad_s = int(yaw_sign * angular.z * 1000)       # rad/s → mrad/s，带方向修正
```

所有 int16 值均限幅到 `[-32768, 32767]`。

### 8.4 流式解析器工作原理

`UplinkFrameParser` 是一个有状态的流式解析器，内部维护一个 `bytearray` 缓冲区。其工作流程如下：

```
┌─────────────────────────────────────────────────────────┐
│  串口读取原始字节 → feed(data)                            │
│                                                         │
│  ┌─────────────────────────────────────────────────────┐│
│  │  1. 将新数据追加到缓冲区尾部                          ││
│  │  2. 在缓冲区中搜索帧头 0x7B                          ││
│  │  3. 若无帧头 → 清空缓冲区，退出                       ││
│  │  4. 丢弃帧头之前的垃圾字节                            ││
│  │  5. 检查缓冲区是否 ≥ 24 字节（一帧长度）               ││
│  │  6. 检查第 23 字节是否为帧尾 0x7D                      ││
│  │     - 否 → 跳过当前帧头字节，回到步骤 2                ││
│  │  7. 提取 24 字节候选帧，验证 BCC 校验                  ││
│  │     - 通过 → 返回解析结果，从缓冲区移除该帧             ││
│  │     - 失败 → 跳过当前帧头字节，回到步骤 2                ││
│  └─────────────────────────────────────────────────────┘│
│                                                         │
│  返回: list[dict]  # 0 个或多个有效帧                     │
└─────────────────────────────────────────────────────────┘
```

该设计能正确处理以下场景：

- **粘包**：一次 `feed()` 包含多个完整帧 → 返回多个解析结果
- **半包**：一次 `feed()` 只有部分帧 → 缓存等待下次数据
- **帧前垃圾**：有效帧前有随机字节 → 自动丢弃并重新同步
- **错帧**：BCC 校验失败 → 跳过该帧头，继续搜索下一个 0x7B
- **逐字节**：每次只喂 1 字节 → 最终仍能拼出完整帧

---

## 9. 里程计算法说明

### 9.1 运动模型

本节点使用简单的二维阿克曼里程计积分模型：

```
状态: (x, y, yaw)
输入: (vx, wz)  — 来自上行帧反馈

更新公式:
    yaw  = yaw + wz × dt
    x    = x + vx × cos(yaw) × dt
    y    = y + vx × sin(yaw) × dt
```

其中：
- `vx` = 上行帧反馈的 X 轴线速度（已转换为 m/s）
- `wz` = 上行帧反馈的 Z 轴角速度（已应用 `yaw_sign` 修正）
- `dt` = 两次积分之间的时间间隔

### 9.2 坐标系约定

```
         y (前进方向初始时刻)
         ↑
         │
         │
         └────────→ x
         车体中心 (base_link)
```

- `vx`：车体坐标系下的前进速度
- `wz`：绕 Z 轴的角速度（ROS 约定：逆时针为正）
- 阿克曼车型 `vy ≈ 0`，忽略侧向速度

### 9.3 四元数转换

仅使用 yaw 角度，pitch 和 roll 默认为 0：

```python
qx = 0.0
qy = 0.0
qz = sin(yaw / 2)
qw = cos(yaw / 2)
```

### 9.4 协方差矩阵

里程计消息中设置了对角协方差（行优先 6×6 矩阵）：

- **Pose 协方差**：`covariance[0]=0.01 (x)`, `covariance[7]=0.01 (y)`, `covariance[35]=0.01 (yaw)`
- **Twist 协方差**：`covariance[0]=0.01 (vx)`, `covariance[35]=0.01 (wz)`

这些值为初始估计，实际应用中可根据需要调整。

---

## 10. yaw_sign 角速度方向约定

### 10.1 问题背景

ROS2 中 `angular.z` 遵循**逆时针为正**的右手定则。但底盘 STM32 协议中 Z 轴角速度的正方向可能与 ROS 约定**相反**（取决于底盘硬件安装方向和 STM32 固件定义）。

### 10.2 解决方案

引入 `yaw_sign` 参数（默认值 `-1.0`），用于在 ROS 约定和底盘串口约定之间进行转换：

**上行（读取时）**：
```python
ros_wz = yaw_sign × serial_wz
# 示例: serial_wz = +0.033 rad/s (底盘报告)
#       ros_wz    = -1.0 × 0.033 = -0.033 rad/s (ROS 中为顺时针)
```

**下行（发送时）**：
```python
serial_wz = yaw_sign × ros_wz
# 示例: ros_wz    = +0.5 rad/s (用户希望逆时针旋转)
#       serial_wz = -1.0 × 0.5 = -0.5 rad/s (发送给底盘)
```

### 10.3 如何确定正确的 yaw_sign 值

1. 启动节点，发布一个正的角速度指令：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
   ```
2. 观察车辆实际旋转方向：
   - 如果车辆**逆时针**旋转（与 ROS 约定一致） → `yaw_sign = -1.0` 正确
   - 如果车辆**顺时针**旋转（与 ROS 约定相反） → 尝试将 `yaw_sign` 改为 `1.0`
3. 同时检查里程计 `/odom` 中的 `angular.z` 是否与实际旋转方向一致。

---

## 11. 安全机制

### 11.1 指令超时保护

参数 `command_timeout`（默认 0.5 秒）：

- 如果超过该时间没有收到新的 `/cmd_vel` 消息，节点会自动发送**零速度帧**
- 这防止了上位机程序崩溃或通信中断后底盘继续运动

### 11.2 速度限幅

| 参数 | 作用 |
|------|------|
| `max_linear_speed` | 对 `/cmd_vel` 的 `linear.x` 进行 `[-max, +max]` 限幅 |
| `max_angular_speed` | 对 `/cmd_vel` 的 `angular.z` 进行 `[-max, +max]` 限幅 |

限幅在 ROS 单位（m/s、rad/s）下进行，然后再转换为协议单位（mm/s、mrad/s）。

int16 值也会被 `clamp_int16()` 限制到 `[-32768, 32767]`。

### 11.3 关闭保护

当节点收到 `Ctrl+C` 或被要求关闭时：

1. 连续发送 **3 次零速度帧**（间隔 50ms）
2. 关闭串口连接
3. 销毁 ROS2 节点

这确保即使在最坏情况下底盘也能收到停止指令。

### 11.4 串口异常处理

| 异常场景 | 处理方式 |
|----------|----------|
| 串口打开失败 | 记录警告日志，定时器周期性重试 |
| 读取异常 | 记录警告，尝试重新打开串口 |
| 写入异常 | 记录警告，不崩溃 |
| 写入不完整 | 记录警告，不增加发送计数 |

---

## 12. 诊断信息

节点以 1 Hz 频率发布 `/diagnostics` 话题，包含以下信息：

### 12.1 状态级别

| 条件 | 级别 | 消息 |
|------|------|------|
| 串口已打开且收到过数据 | `OK` | "OK" |
| 串口已打开但未收到数据 | `WARN` | "Connected, no data received yet" |
| 串口未打开 | `ERROR` | "Serial port not open" |

### 12.2 诊断键值对

| 键名 | 说明 |
|------|------|
| `serial_connected` | 串口是否已连接 |
| `port` | 串口设备路径 |
| `rx_frames` | 已接收的有效帧总数 |
| `tx_frames` | 已发送的帧总数 |
| `checksum_errors` | BCC 校验错误计数 |
| `parse_errors` | 串口读取异常计数 |
| `motor_enabled` | 电机是否使能 |
| `battery_voltage_V` | 最近一次电池电压（V） |
| `last_frame_age` | 距最后一次成功收帧的时间 |

### 12.3 查看诊断信息

```bash
# 实时查看
ros2 topic echo /diagnostics

# 使用 rqt_runtime_monitor
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 13. 使用示例

### 13.1 基本启动

```bash
# 终端 1：启动节点
cd ~/fast_ws
source install/setup.bash
ros2 launch ackermann_serial_bridge serial_bridge.launch.py

# 终端 2：发布速度指令
source install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 13.2 前进

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

### 13.3 后退

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.2}, angular: {z: 0.0}}"
```

### 13.4 左转（逆时针）

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

### 13.5 右转（顺时针）

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: -0.5}}"
```

### 13.6 原地旋转

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.8}}"
```

### 13.7 停车

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 13.8 监控里程计

```bash
ros2 topic echo /odom
```

### 13.9 监控电池

```bash
ros2 topic echo /battery_state
```

### 13.10 查看话题列表

```bash
ros2 topic list
# 预期输出：
# /battery_state
# /cmd_vel
# /diagnostics
# /odom
# /parameter_events
# /rosout
```

### 13.11 使用键盘遥控（需额外安装）

```bash
# 安装 teleop_twist_keyboard
sudo apt install ros-humble-teleop-twist-keyboard

# 启动（注意 remap 话题名）
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel
```

---

## 14. 常见问题排查

### 14.1 串口打开失败

**现象**：日志输出 `Cannot open serial port /dev/ttyACM0: [Errno 13]`

**排查步骤**：

```bash
# 1. 检查设备是否存在
ls -la /dev/ttyACM*

# 2. 检查用户是否在 dialout 组
groups $USER

# 3. 如果不在 dialout 组，添加并重新登录
sudo usermod -aG dialout $USER
# 注销并重新登录！

# 4. 临时方案（不推荐长期使用）
sudo chmod 666 /dev/ttyACM0
```

### 14.2 串口打开成功但收不到数据

**现象**：日志输出 `Connected, no data received yet`

**排查步骤**：

1. 确认底盘已上电且 STM32 正常运行
2. 确认串口线连接正确（USB 转 TTL 连接到 UART3）
3. 确认波特率设置正确（默认 115200）
4. 使用 `screen` 或 `minicom` 手动测试串口：
   ```bash
   sudo apt install screen
   screen /dev/ttyACM0 115200
   # 应能看到乱码数据流，按 Ctrl+A 然后 K 退出
   ```
5. 检查是否有其他程序占用了该串口：
   ```bash
   sudo fuser /dev/ttyACM0
   ```

### 14.3 收到数据但校验错误计数不断增加

**现象**：`checksum_errors` 持续增加

**可能原因**：
- 波特率不匹配
- 串口线质量差或接触不良
- 电平不匹配（3.3V vs 5V）

**排查**：
1. 尝试更换串口线
2. 确认 STM32 固件版本与协议一致
3. 检查 `dmesg` 是否有 USB 断连重连记录

### 14.4 里程计漂移严重

**现象**：`/odom` 显示的位置与实际运动差距很大

**原因**：轮式里程计本身存在累积误差，属于正常现象。

**缓解方法**：
1. 结合激光雷达/视觉进行传感器融合（如 robot_localization 包）
2. 调整里程计协方差以反映实际不确定性
3. 确保轮胎没有打滑

### 14.5 车辆运动方向与指令相反

**现象**：发布正的 `linear.x`，车辆却后退

**解决方案**：检查 `yaw_sign` 参数。如果需要反转前进方向，可能需要修改 STM32 固件中的编码器方向定义。

### 14.6 车辆旋转方向与 ROS 约定不一致

**现象**：发布正的 `angular.z`，车辆顺时针旋转（ROS 约定应为逆时针）

**解决方案**：将 `yaw_sign` 从 `-1.0` 改为 `1.0`（或反之）。详见[第 10 节](#10-yaw_sign-角速度方向约定)。

---

## 15. 单元测试

### 15.1 运行测试

```bash
# 方式一：直接 pytest
cd ~/fast_ws
python3 -m pytest src/ackermann_serial_bridge/test/test_protocol.py -v

# 方式二：colcon test（推荐，会自动发现测试）
colcon build --packages-select ackermann_serial_bridge
colcon test --packages-select ackermann_serial_bridge --event-handlers console_direct+
```

### 15.2 测试用例覆盖

共 31 项测试，覆盖以下内容：

| 测试类 | 测试项数 | 覆盖内容 |
|--------|----------|----------|
| `TestCalcBCC` | 3 | BCC 校验计算（已知值、空数据、单字节） |
| `TestClampInt16` | 4 | int16 限幅（范围内、上溢、下溢、零） |
| `TestInt16` | 5 | int16 大端序打包/解包（正数、负数、零、往返、已知值） |
| `TestBuildDownlinkFrame` | 6 | 下行帧构建（精确字节匹配、长度、帧头帧尾、BCC、零速、Y 轴为零） |
| `TestParseUplinkFrame` | 5 | 上行帧解析（规格帧全字段、错误帧头、错误帧尾、错误 BCC、错误长度） |
| `TestUplinkFrameParser` | 8 | 流式解析器（单帧、双帧拼接、半包、帧前垃圾、损坏帧恢复、空数据、重置、逐字节） |

### 15.3 关键测试帧

**下行 100 mm/s 前进帧**：
```
原始: 7B 00 00 00 64 00 00 00 00 1F 7D
      │  │  │  │  │  │  │  │  │  │  └─ 帧尾 0x7D
      │  │  │  │  │  │  │  │  │  └──── BCC = 0x1F
      │  │  │  │  │  │  │  │  └─────── Z 轴 = 0 mrad/s
      │  │  │  │  │  │  │  └────────── Y 轴 = 0 mm/s
      │  │  │  │  │  │  └───────────── X 轴 = 100 mm/s (0x0064)
      │  │  │  │  │  └──────────────── 预留 0x00
      │  │  │  │  └─────────────────── 预留 0x00
      │  │  │  └────────────────────── 预留 0x00
      │  │  └───────────────────────── 预留 0x00
      │  └──────────────────────────── 预留 0x00
      └─────────────────────────────── 帧头 0x7B
```

**上行测试帧**：
```
原始: 7B 00 00 9B 00 00 FF DF 00 60 00 0C 40 A8 FF FD 00 06 00 1E 5B 87 82 7D

解析结果:
  flag_stop   = 0        (电机使能)
  x_mm_s      = 155      (0x009B)
  y_mm_s      = 0        (0x0000)
  z_mrad_s    = -33      (0xFFDF, 有符号)
  acc_z_raw   = 16552    (0x40A8)
  battery_mv  = 23431    (0x5B87)
  voltage     = 23.431 V
  BCC         = 0x82     ✓
```

---

## 16. 代码模块说明

### 16.1 protocol.py — 纯协议层

**无 ROS2 依赖**，可独立进行单元测试。

#### 常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `HEADER` | `0x7B` | 帧头字节 |
| `TAIL` | `0x7D` | 帧尾字节 |
| `UPLINK_FRAME_LEN` | `24` | 上行帧长度 |
| `DOWNLINK_FRAME_LEN` | `11` | 下行帧长度 |
| `INT16_MAX` | `32767` | int16 最大值 |
| `INT16_MIN` | `-32768` | int16 最小值 |

#### 函数

| 函数 | 签名 | 说明 |
|------|------|------|
| `calc_bcc` | `(data: bytes) -> int` | 计算 XOR 校验和 |
| `clamp_int16` | `(value: int) -> int` | 将整数限幅到 int16 范围 |
| `pack_int16_be` | `(value: int) -> bytes` | 将 int16 打包为大端序 2 字节 |
| `unpack_int16_be` | `(high: int, low: int) -> int` | 将 2 字节解包为 int16 |
| `build_downlink_frame` | `(x_mm_s, y_mm_s, z_mrad_s) -> bytes` | 构建 11 字节下行帧 |
| `parse_uplink_frame` | `(frame: bytes) -> dict` | 解析 24 字节上行帧 |

#### 类

| 类 | 说明 |
|----|------|
| `UplinkFrameParser` | 流式上行帧解析器，处理粘包/半包/错包 |

`UplinkFrameParser` 方法：
- `feed(data: bytes) -> List[dict]`：喂入原始字节，返回有效帧列表
- `reset()`：清空内部缓冲区

### 16.2 serial_bridge_node.py — ROS2 节点

#### 类：`AckermannSerialBridgeNode`

继承自 `rclpy.node.Node`，实现以下功能：

| 方法 | 说明 |
|------|------|
| `__init__` | 声明参数、创建发布者/订阅者、打开串口、启动定时器 |
| `_open_serial` | 打开或重新打开串口（异常安全） |
| `_read_serial_cb` | 读取定时器回调：读串口 → 解析 → 发布 |
| `_publish_odom_from_frame` | 积分里程计并发布 Odometry + TF |
| `_publish_battery` | 发布 BatteryState |
| `_cmd_vel_cb` | /cmd_vel 订阅回调 |
| `_send_cmd_cb` | 发送定时器回调：构建并发送下行帧 |
| `_publish_diagnostics_cb` | 发布诊断信息 |
| `destroy_node` | 关闭前发送 3 次零速帧 |

#### 定时器

| 定时器 | 频率 | 回调 |
|--------|------|------|
| 读取定时器 | `read_rate` Hz（默认 100） | `_read_serial_cb` |
| 发送定时器 | `cmd_send_rate` Hz（默认 20） | `_send_cmd_cb` |
| 诊断定时器 | 1 Hz | `_publish_diagnostics_cb` |

---

## 17. 许可证

MIT License
