# grab_trigger.cpp 源码解析文档

## 一、文件概述

`grab_trigger.cpp` 是海康威视 MVS（Machine Vision SDK）工业相机的 ROS2 驱动节点。它完成了从相机初始化、参数配置、图像采集到 ROS2 话题发布的完整流程，并支持硬件触发模式和运行时动态参数修改。

### 1.1 功能清单

| 功能 | 说明 |
|------|------|
| GigE / USB3 相机自动发现 | 通过 `MV_CC_EnumDevices` 枚举所有在线设备 |
| 序列号匹配 | 多相机场景下通过序列号精确选定目标相机 |
| 相机参数配置 | 曝光、增益、Gamma、像素格式、触发模式 |
| 连续采集 / 硬件触发 | 通过配置文件切换两种采集模式 |
| 像素格式转换 | SDK 内部将 Bayer/Mono 等格式统一转为 RGB8 |
| ROS2 图像发布 | 通过 `image_transport` 发布 `sensor_msgs/Image` |
| 硬件触发时间戳注入 | 从共享内存读取外部触发时间戳，赋给图像帧头 |
| 运行时参数修改 | `ros2 param set` 实时修改曝光/增益/Gamma |
| 安全退出 | Ctrl+C 时自动停止采集、关闭设备、销毁句柄 |

### 1.2 头文件依赖

```cpp
#include "MvCameraControl.h"          // 海康 MVS SDK C API（核心）
#include <rclcpp/rclcpp.hpp>          // ROS2 C++ 客户端库
#include <sensor_msgs/msg/image.hpp>  // ROS2 图像消息
#include <std_msgs/msg/header.hpp>    // ROS2 标准消息头
#include <cv_bridge/cv_bridge.h>      // OpenCV ↔ ROS2 图像转换
#include <image_transport/image_transport.hpp>  // 图像传输（支持压缩等插件）
#include <opencv2/opencv.hpp>         // OpenCV（图像缩放）
#include <fcntl.h> / <sys/mman.h>     // POSIX 共享内存（触发时间戳）
```

---

## 二、整体架构与执行流程

```
main()
  │
  ├─ rclcpp::init()                         // 初始化 ROS2
  ├─ node = make_shared<MvsCameraNode>()    // 创建节点对象
  │
  ├─ node->loadConfig(yaml_path)            // ① 从 YAML 读取相机参数
  ├─ node->declareDynamicParams()           // ② 声明 ROS2 动态参数 + 注册回调
  ├─ node->initCamera()                     // ③ 枚举→选设备→打开→配参数→开始采集
  ├─ node->startPublishing()                // ④ 创建 image_transport 发布者 + 启动采集线程
  │
  ├─ rclcpp::spin(node)                     // ⑤ 阻塞，处理 ROS2 事件（含参数修改回调）
  │
  └─ node->requestStop() + join()           // ⑥ 退出时：停采集→关设备→销毁句柄
```

### 线程模型

```
主线程 (rclcpp::spin)              采集线程 (grabLoop)
─────────────────────              ─────────────────────
处理 ROS2 回调                      循环调用 GetOneFrameTimeout
处理参数修改回调                     像素格式转换
                                   图像缩放
                                   发布 ROS2 Image 消息
```

两个线程通过以下共享状态通信：
- `running_`（`std::atomic<bool>`）：主线程设为 false 通知采集线程退出
- `handle_`（`void*`）：MVS 设备句柄，两个线程都会使用（MVS SDK 内部有线程安全保护）
- `exposure_time_` / `gain_` / `gamma_`：参数回调在主线程写入，`applyParams` 在采集前读取

---

## 三、数据结构定义

### 3.1 共享内存时间戳

```cpp
struct TimeStamp {
  int64_t high;
  int64_t low;
};
```

用于接收外部硬件触发系统注入的高精度时间戳。`low` 字段存储纳秒级时间戳，`high` 保留未用。当触发模式启用且共享内存可用时，图像帧的 `header.stamp` 使用此时间戳而非 ROS 系统时间，以实现多传感器精确时间同步。

### 3.2 像素格式枚举

```cpp
enum PixelFormat : unsigned int {
  RGB8            = 0x02180014,   // 24-bit RGB，每通道 8 位
  BayerRG8        = 0x01080009,   // Bayer RG 格式，8 位
  BayerRG12Packed = 0x010C002B,   // Bayer RG 格式，12 位压缩
  BayerGB12Packed = 0x010C002C,   // Bayer GB 格式，12 位压缩
  BayerGB8        = 0x0108000A    // Bayer GB 格式，8 位
};
```

这些值来自 MVS SDK 的 `MvCameraControl.h` 中定义的像素格式常量。配置文件中 `PixelFormat: 0` 对应 `RGB8`，`PixelFormat: 1` 对应 `BayerRG8`，以此类推。数组 `PIXEL_FORMATS` 将索引映射为实际的 SDK 常量。

---

## 四、MvsCameraNode 类详解

节点类继承自 `rclcpp::Node`，将所有 ROS2 和相机逻辑封装在一个类中。

### 4.1 成员变量

#### 相机参数（来自 YAML，部分可通过 ROS2 参数运行时修改）

| 变量 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_number_` | string | — | 目标相机序列号，多相机时用于匹配 |
| `topic_name_` | string | — | 发布的 ROS2 图像话题名 |
| `trigger_enable_` | int | 0 | 0=连续采集，1=硬件触发 |
| `pixel_format_idx_` | int | 0 | 像素格式索引，对应 PIXEL_FORMATS 数组 |
| `exposure_auto_mode_` | int | 0 | 0=手动，1=单次自动，2=连续自动 |
| `exposure_time_` | int | 5000 | 曝光时间（微秒），仅手动模式有效 |
| `gain_auto_` | int | 0 | 增益自动模式，0=手动 |
| `gain_` | float | 1.0 | 增益值，仅手动模式有效 |
| `gamma_` | float | 1.0 | Gamma 校正值 |
| `gamma_selector_` | int | 0 | 0=User，1=sRGB，2=Off |
| `image_scale_` | float | 1.0 | 图像缩放比例 |

#### 运行时状态

| 变量 | 类型 | 说明 |
|------|------|------|
| `handle_` | void* | MVS SDK 设备句柄，贯穿整个生命周期 |
| `pub_` | image_transport::Publisher | ROS2 图像发布者 |
| `worker_` | std::thread | 采集线程 |
| `running_` | atomic<bool> | 采集线程运行标志 |
| `param_cb_handle_` | OnSetParametersCallbackHandle::SharedPtr | 参数回调句柄 |
| `shm_` / `shm_fd_` | TimeStamp* / int | 共享内存指针和文件描述符 |

---

## 五、核心函数逐行解析

### 5.1 loadConfig — 加载 YAML 配置文件

```cpp
bool loadConfig(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
```

使用 OpenCV 的 `FileStorage` 读取 YAML 文件。选择这个库是因为它在 ROS2 环境中已经安装，且支持 YAML 格式。文件格式示例：

```yaml
%YAML:1.0
SerialNumber: "DA2099368"
TopicName: "left_camera/image"
TriggerEnable: 0
ExposureAutoMode: 0
ExposureTime: 5000
GainAuto: 0
Gain: 15
Gamma: 0.7
GammaSelector: 1
PixelFormat: 0
image_scale: 0.5
```

每个字段通过 `fs["key"] >> variable` 读取。`release()` 关闭文件句柄。

**安全检查**：
- `image_scale < 0.1` 时强制设为 1.0（防止无效缩放）
- `topic_name` 为空时默认为 `"camera/image"`

### 5.2 declareDynamicParams — 声明 ROS2 运行时参数

```cpp
void declareDynamicParams() {
    this->declare_parameter("exposure_time", exposure_time_);
    this->declare_parameter("gain", gain_);
    this->declare_parameter("gamma", gamma_);
```

将三个相机参数注册为 ROS2 参数。`declare_parameter` 的第二个参数是默认值（来自 YAML 配置）。用户可通过 launch 文件的 `parameters` 覆盖初始值。

```cpp
    exposure_time_ = this->get_parameter("exposure_time").as_int();
```

`get_parameter` 读取最终值（可能被 launch 参数覆盖），写回成员变量，确保后续 `initCamera()` 使用正确的值。

```cpp
    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params)
        -> rcl_interfaces::msg::SetParametersResult
      {
        return onParamChange(params);
      });
```

注册参数修改回调。当用户执行 `ros2 param set /left_camera exposure_time 8000` 时，ROS2 框架会调用此回调。回调函数遍历被修改的参数，逐个调用 MVS SDK API 将新值写入相机硬件。

**关键逻辑**：曝光时间和增益的设置受自动模式限制——如果 `exposure_auto_mode_ != 0`（自动曝光），则忽略手动曝光值并输出警告。

### 5.3 initCamera — 相机初始化全流程

这是最核心的函数，按照 MVS SDK 标准流程执行：

#### 步骤 1：枚举设备

```cpp
MV_CC_DEVICE_INFO_LIST dev_list{};
nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
```

`MV_CC_EnumDevices` 扫描所有已连接的 GigE 和 USB3 相机。`dev_list.pDeviceInfo` 是 SDK 内部分配的设备信息数组，`dev_list.nDeviceNum` 是找到的设备数量。

> **SDK 注意事项**：此函数的设备列表内存由 SDK 内部管理，多线程调用时可能被释放重分配，因此应避免并发枚举。

#### 步骤 2：按序列号选择设备

```cpp
if (dev_list.nDeviceNum > 1) {
    // 多相机：遍历设备列表，匹配 serial_number_
    for (unsigned i = 0; i < dev_list.nDeviceNum; ++i) {
        // 根据传输层类型读取不同的序列号字段
        if (info->nTLayerType == MV_USB_DEVICE)
            sn = info->SpecialInfo.stUsb3VInfo.chSerialNumber;
        else if (info->nTLayerType == MV_GIGE_DEVICE)
            sn = info->SpecialInfo.stGigEInfo.chSerialNumber;
    }
}
```

GigE 和 USB3 的设备信息结构不同，需要分别处理。单相机时直接选 index 0，跳过序列号匹配。

#### 步骤 3：创建句柄并打开设备

```cpp
nRet = MV_CC_CreateHandle(&handle_, dev_list.pDeviceInfo[sel]);
nRet = MV_CC_OpenDevice(handle_);
```

- `MV_CC_CreateHandle`：根据设备信息创建 SDK 句柄，后续所有 API 调用都通过此句柄操作
- `MV_CC_OpenDevice`：以独占模式连接相机。默认 `MV_ACCESS_Exclusive`，其他进程无法同时访问

**错误码 `0x80000203`**：表示设备被占用（MVS GUI 或其他进程已打开该相机）。

#### 步骤 4：配置相机参数

```cpp
// 关闭自动帧率限制（允许外部触发频率决定帧率）
MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);

// 设置像素格式
MV_CC_SetEnumValue(handle_, "PixelFormat", PIXEL_FORMATS[pixel_format_idx_]);

// 应用曝光/增益/Gamma 参数
applyParams();

// 设置触发模式
MV_CC_SetEnumValue(handle_, "TriggerMode", trigger_enable_);
if (trigger_enable_) {
    MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
}
```

**参数设置 API 对照表**：

| API | 用途 | 本代码中的使用 |
|-----|------|---------------|
| `MV_CC_SetBoolValue` | 设置布尔型参数 | 关闭自动帧率 |
| `MV_CC_SetEnumValue` | 设置枚举型参数（按数值） | 像素格式、触发模式、触发源、增益自动模式 |
| `MV_CC_SetExposureAutoMode` | 设置曝光自动模式 | applyParams 中 |
| `MV_CC_SetExposureTime` | 设置曝光时间（微秒） | applyParams 中 |
| `MV_CC_SetGain` | 设置增益值 | applyParams 中 |
| `MV_CC_SetGammaSelector` | 设置 Gamma 选择器 | applyParams 中 |
| `MV_CC_SetGamma` | 设置 Gamma 值 | applyParams 中 |

#### 步骤 5：开始采集

```cpp
nRet = MV_CC_StartGrabbing(handle_);
```

调用后相机开始出图（连续模式）或等待触发信号（触发模式）。此 API 不支持 CameraLink 设备。

### 5.4 applyParams — 批量应用相机参数

```cpp
void applyParams() {
    MV_CC_SetExposureAutoMode(handle_, exposure_auto_mode_);
    if (exposure_auto_mode_ == 2) {  // 连续自动曝光
        MV_CC_SetAutoExposureTimeLower(handle_, 100);
        MV_CC_SetAutoExposureTimeUpper(handle_, 20000);
    }
    if (exposure_auto_mode_ == 0) {  // 手动曝光
        MV_CC_SetExposureTime(handle_, exposure_time_);
    }
    // ... 增益、Gamma 类似
}
```

**曝光模式逻辑**：

| exposure_auto_mode_ | 含义 | 行为 |
|---------------------|------|------|
| 0 (Off) | 手动曝光 | 使用 `exposure_time_` 值 |
| 1 (Once) | 单次自动 | 相机自动调整一次 |
| 2 (Continues) | 连续自动 | 相机持续自动调整，设置上下限 |

### 5.5 onParamChange — 运行时参数修改回调

```cpp
rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> &params) {
```

当用户执行 `ros2 param set` 时触发。回调遍历所有被修改的参数：

- `exposure_time` → 调用 `MV_CC_SetExposureTime`，仅在手动曝光模式下生效
- `gain` → 调用 `MV_CC_SetGain`，仅在手动增益模式下生效
- `gamma` → 调用 `MV_CC_SetGamma`，任何时候都可修改

返回 `SetParametersResult`，`successful=false` 时 ROS2 会报告错误。

### 5.6 grabLoop — 采集线程主循环

这是运行在独立线程中的核心采集循环：

#### 缓冲区分配

```cpp
MVCC_INTVALUE param{};
MV_CC_GetIntValue(handle_, "PayloadSize", &param);
const unsigned buf_size = param.nCurValue * 3;
auto *raw_data = static_cast<unsigned char *>(std::malloc(buf_size));
auto *bgr_data = static_cast<unsigned char *>(std::malloc(buf_size));
```

- `PayloadSize` 是相机单帧图像的原始数据大小（字节）
- 乘以 3 是因为 RGB8 格式最大需要原始数据 3 倍的空间（Bayer → RGB 转换）
- `raw_data`：接收相机原始图像数据
- `bgr_data`：存放像素格式转换后的 RGB8 数据

#### 取图循环

```cpp
while (running_ && rclcpp::ok()) {
    nRet = MV_CC_GetOneFrameTimeout(handle_, raw_data, buf_size, &frame_info, 1000);
    if (nRet != MV_OK) continue;  // 超时或错误，重试
```

`MV_CC_GetOneFrameTimeout` 是主动取图 API（与回调方式互斥）：
- 从 SDK 内部缓冲区取出一帧图像数据到用户提供的 `raw_data`
- 超时时间 1000ms，超时返回 `0x80000007`（MV_E_NODATA）
- 必须在 `StartGrabbing` 之后调用

**错误码 `0x80000007`（MV_E_NODATA）**：SDK 缓冲区中没有可用图像。在触发模式下，这意味着没有收到触发信号。

#### 时间戳处理

```cpp
if (trigger_enable_ && shm_ && shm_->low != 0) {
    double sec = shm_->low / 1e9;
    stamp = rclcpp::Time(sec_integer, sec_fraction);
} else {
    stamp = this->now();  // 使用 ROS2 系统时间
}
```

两种时间戳来源：
1. **共享内存**：外部触发硬件写入的高精度时间戳，用于多传感器时间同步
2. **ROS2 系统时间**：`this->now()` 获取节点时钟时间

#### 像素格式转换

```cpp
conv_param.enSrcPixelType = frame_info.enPixelType;  // 相机原始格式
conv_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;  // 目标：RGB8
MV_CC_ConvertPixelType(handle_, &conv_param);
```

`MV_CC_ConvertPixelType` 将相机输出的原始像素格式（可能是 BayerRG8、BayerRG12Packed 等）转换为统一的 RGB8 格式。这样下游 ROS2 节点不需要关心相机的具体像素格式。

#### 图像缩放与发布

```cpp
cv::Mat img(frame_info.nHeight, frame_info.nWidth, CV_8UC3, bgr_data);
if (image_scale_ != 1.0f) {
    cv::resize(img, img, cv::Size(cols * scale, rows * scale));
}
auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img).toImageMsg();
msg->header.stamp = stamp;
msg->header.frame_id = "camera";
pub_.publish(msg);
```

- `cv::Mat` 零拷贝包装 `bgr_data`（不复制像素数据）
- `cv::resize` 按 `image_scale_` 缩放（如 0.5 则分辨率减半）
- `cv_bridge::CvImage` 将 OpenCV Mat 转换为 ROS2 `sensor_msgs/Image`
- 通过 `image_transport::Publisher` 发布，自动支持压缩传输插件

### 5.7 openShm / closeShm — 共享内存管理

```cpp
void openShm() {
    std::string path = "/home/" + std::string(user) + "/timeshare";
    shm_fd_ = open(path.c_str(), O_RDWR);
    shm_ = static_cast<TimeStamp *>(mmap(nullptr, sizeof(TimeStamp),
         PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));
}
```

使用 POSIX 共享内存（`mmap` + `MAP_SHARED`）读取外部触发系统写入的时间戳。文件路径为 `~/timeshare`，由外部触发硬件的进程创建和写入。

- `O_RDWR`：读写模式打开
- `MAP_SHARED`：映射为共享内存，多个进程可访问
- `PROT_READ | PROT_WRITE`：允许读写

如果文件不存在或 mmap 失败，`shm_` 设为 nullptr，采集线程会回退到使用 ROS2 系统时间。

### 5.8 cleanup — 资源清理

```cpp
void cleanup() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (handle_) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }
}
```

按照 MVS SDK 要求的**逆序**释放资源：
1. 停止采集 → `MV_CC_StopGrabbing`
2. 关闭设备 → `MV_CC_CloseDevice`
3. 销毁句柄 → `MV_CC_DestroyHandle`

析构函数 `~MvsCameraNode()` 调用 `cleanup()`，确保即使异常退出也能正确释放。

### 5.9 main 函数

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MvsCameraNode>(rclcpp::NodeOptions());
    if (!node->loadConfig(argv[1])) return 1;
    node->declareDynamicParams();
    if (!node->initCamera()) return 1;
    node->startPublishing();
    rclcpp::spin(node);           // 阻塞，直到 Ctrl+C
    node->requestStop();
    node->join();
    rclcpp::shutdown();
}
```

启动顺序严格遵循 MVS SDK 要求：配置 → 参数 → 初始化 → 开始采集。`rclcpp::spin` 阻塞主线程处理所有 ROS2 回调（包括参数修改回调）。收到 Ctrl+C 后 `spin` 返回，执行清理。

---

## 六、MVS SDK API 使用对照表

| 代码中的调用 | SDK API | SDK 功能 | 调用时机 |
|-------------|---------|----------|----------|
| 枚举设备 | `MV_CC_EnumDevices` | 扫描 GigE/USB3 设备 | initCamera |
| 创建句柄 | `MV_CC_CreateHandle` | 创建设备操作句柄 | initCamera |
| 打开设备 | `MV_CC_OpenDevice` | 以独占模式连接相机 | initCamera |
| 关闭帧率限制 | `MV_CC_SetBoolValue` | 设置布尔型 GenICam 节点 | initCamera |
| 设置像素格式 | `MV_CC_SetEnumValue` | 设置枚举型 GenICam 节点 | initCamera |
| 设置曝光模式 | `MV_CC_SetExposureAutoMode` | 设置曝光自动模式 | applyParams |
| 设置曝光时间 | `MV_CC_SetExposureTime` | 设置固定曝光时间 | applyParams / onParamChange |
| 设置增益 | `MV_CC_SetGain` | 设置增益值 | applyParams / onParamChange |
| 设置 Gamma | `MV_CC_SetGamma` | 设置 Gamma 校正值 | applyParams / onParamChange |
| 设置触发模式 | `MV_CC_SetEnumValue("TriggerMode")` | 开关触发模式 | initCamera |
| 设置触发源 | `MV_CC_SetEnumValue("TriggerSource")` | 选择 LINE0/软触发等 | initCamera |
| 开始采集 | `MV_CC_StartGrabbing` | 启动图像流 | initCamera |
| 获取 PayloadSize | `MV_CC_GetIntValue` | 获取单帧数据大小 | grabLoop |
| 取一帧图像 | `MV_CC_GetOneFrameTimeout` | 超时方式取帧 | grabLoop |
| 像素格式转换 | `MV_CC_ConvertPixelType` | Bayer/Mono → RGB8 | grabLoop |
| 停止采集 | `MV_CC_StopGrabbing` | 停止图像流 | cleanup |
| 关闭设备 | `MV_CC_CloseDevice` | 断开相机连接 | cleanup |
| 销毁句柄 | `MV_CC_DestroyHandle` | 释放 SDK 句柄资源 | cleanup |

---

## 七、配置文件参数说明

| 参数 | 类型 | 值域 | 说明 |
|------|------|------|------|
| `SerialNumber` | string | — | 相机序列号，多相机时必填 |
| `TopicName` | string | — | ROS2 发布话题名 |
| `TriggerEnable` | int | 0/1 | 0=连续采集，1=硬件触发 |
| `PixelFormat` | int | 0-4 | 0=RGB8, 1=BayerRG8, 2=BayerRG12Packed, 3=BayerGB12Packed, 4=BayerGB8 |
| `ExposureAutoMode` | int | 0/1/2 | 0=手动, 1=单次自动, 2=连续自动 |
| `ExposureTime` | int | μs | 手动模式下的曝光时间 |
| `GainAuto` | int | 0/1/2 | 0=手动, 1=单次自动, 2=连续自动 |
| `Gain` | float | 相机范围 | 手动模式下的增益值 |
| `Gamma` | float | 0-17 | Gamma 校正值 |
| `GammaSelector` | int | 0/1/2 | 0=User, 1=sRGB, 2=Off |
| `image_scale` | float | 0.1-1.0 | 图像缩放比例，1.0 不缩放 |

---

## 八、运行时参数修改

以下三个参数可通过 `ros2 param set` 在运行时修改，无需重启节点：

```bash
# 修改曝光时间为 8000 微秒（仅手动曝光模式有效）
ros2 param set /left_camera exposure_time 8000

# 修改增益为 10.0（仅手动增益模式有效）
ros2 param set /left_camera gain 10.0

# 修改 Gamma 为 1.2
ros2 param set /left_camera gamma 1.2
```

参数修改通过 `add_on_set_parameters_callback` 注册的回调函数实时生效，回调内部直接调用 MVS SDK API 写入相机寄存器。

---

## 九、错误处理策略

| 场景 | 错误码 | 处理方式 |
|------|--------|----------|
| 配置文件打不开 | — | `RCLCPP_ERROR` + 返回 false，节点退出 |
| 枚举设备失败 | 非 MV_OK | `RCLCPP_ERROR` + 返回 false |
| 无设备在线 | nDeviceNum=0 | `RCLCPP_ERROR` + 返回 false |
| 序列号不匹配 | — | `RCLCPP_ERROR` + 返回 false |
| 创建句柄失败 | 非 MV_OK | `RCLCPP_ERROR` + 返回 false |
| 打开设备失败 | 0x80000203 | `RCLCPP_ERROR` + 返回 false（设备被占用） |
| 设置参数失败 | 非 MV_OK | `RCLCPP_WARN`（警告但继续） |
| 取帧超时 | 0x80000007 | 重试，前 5 次和每 100 次输出一次警告 |
| 像素转换失败 | 非 MV_OK | 跳过该帧，继续下一帧 |
| 共享内存打开失败 | — | `RCLCPP_WARN`，回退到 ROS2 系统时间 |

取帧失败的节流日志策略：前 5 次每次都输出警告（方便调试），之后每 100 次输出一次（防止日志刷屏）。恢复正常后输出恢复信息。

---

## 十、资源生命周期

```
构造函数
  └─ handle_ = nullptr, running_ = true

loadConfig()
  └─ 读取 YAML，填充成员变量

declareDynamicParams()
  └─ 注册 ROS2 参数，注册参数修改回调

initCamera()
  ├─ MV_CC_CreateHandle    → handle_ 有效
  ├─ MV_CC_OpenDevice      → 相机连接
  ├─ 设置参数              → 相机配置完成
  └─ MV_CC_StartGrabbing   → 相机出图

startPublishing()
  ├─ 创建 image_transport 发布者
  └─ 启动采集线程 (grabLoop)
       ├─ openShm           → 共享内存映射
       ├─ 分配 raw_data/bgr_data
       └─ 取图循环
            ├─ GetOneFrameTimeout
            ├─ ConvertPixelType
            ├─ cv::resize
            └─ pub_.publish

cleanup() [析构时自动调用]
  ├─ running_ = false       → 通知采集线程退出
  ├─ worker_.join()          → 等待线程结束
  │    └─ grabLoop 退出
  │         ├─ free(raw_data)
  │         ├─ free(bgr_data)
  │         └─ closeShm     → munmap + close
  ├─ MV_CC_StopGrabbing
  ├─ MV_CC_CloseDevice
  └─ MV_CC_DestroyHandle    → handle_ = nullptr
```
