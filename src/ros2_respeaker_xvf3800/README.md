# reSpeaker XVF3800 ROS 2 节点

这是一个独立的 ROS 2 `ament_python` 包，包名为 `respeaker_xvf3800`。它通过 USB control transfer 读取 reSpeaker XVF3800 的 `DOA_VALUE`，并发布 ROS 2 话题。

## 发布话题

- `/doa`：`std_msgs/msg/UInt16`，声源方向角，范围为 0 到 359 度。
- `/speech_detected`：`std_msgs/msg/Bool`，是否检测到语音。

## 依赖安装

在已安装 ROS 2 的 Ubuntu 系统中执行：

```bash
sudo apt update
sudo apt install python3-usb python3-pip
pip3 install pyusb
```

如果普通用户无法访问 USB 设备，需要添加 udev 规则：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="001a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-respeaker-xvf3800.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后拔插一次 reSpeaker 设备。

## 放入 ROS 2 工作空间

把本文件夹复制到 ROS 2 工作空间的 `src` 目录下。目录名可以和包名一致：

```bash
mkdir -p ~/ros2_ws/src
cp -r ros2_respeaker_xvf3800 ~/ros2_ws/src/respeaker_xvf3800
```

如果你是在 Linux 上直接克隆了完整仓库，也可以只复制这个子目录：

```bash
cp -r reSpeaker_XVF3800_USB_4MIC_ARRAY/ros2_respeaker_xvf3800 ~/ros2_ws/src/respeaker_xvf3800
```

## 编译

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select respeaker_xvf3800
source install/setup.bash
```

## 启动节点

```bash
ros2 launch respeaker_xvf3800 doa.launch.py
```

启动后查看话题：

```bash
ros2 topic echo /doa
ros2 topic echo /speech_detected
```

## 可选参数

默认 USB VID/PID 为 `0x2886:0x001a`，默认读取频率为 `10 Hz`。

```bash
ros2 launch respeaker_xvf3800 doa.launch.py poll_rate_hz:=20.0 vid:=10374 pid:=26
```

参数说明：

- `poll_rate_hz`：读取并发布 DOA 的频率。
- `vid`：USB vendor ID，十进制格式，默认 `10374`，对应 `0x2886`。
- `pid`：USB product ID，十进制格式，默认 `26`，对应 `0x001a`。

## 常见问题

如果启动时报 `reSpeaker XVF3800 not found`：

1. 确认设备已插入 Linux 主机。
2. 执行 `lsusb`，确认能看到 `2886:001a`。
3. 确认 udev 规则已生效，必要时拔插设备。
4. 确认没有其他程序正在占用设备控制接口。
