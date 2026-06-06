# mono_edgesam RDK S100 编译与本地图片回灌说明

本文档面向 RDK S100 Ubuntu 系统，说明如何在板端编译 `mono_edgesam`，以及如何使用本地 JPG 图片进行 EdgeSAM 回灌预测。运行部分只覆盖“本地 jpg 格式图片进行回灌预测”。

## 编译环境

确认板端已经具备以下环境：

- 已安装 RDK S100 Ubuntu 系统。
- 已安装 TogetherROS / ROS2 Humble 运行环境。
- 已安装 `colcon`。
- 当前环境能找到 `dnn_node`、`hobot_cv`、`ai_msgs`、`hbm_img_msgs`、`cv_bridge`、`sensor_msgs` 等依赖包。
- 本包位于 colcon 工作区，例如 `/home/sunrise/fast_ws/src/mono_edgesam`。

每个新终端建议先 source 环境：

```bash
cd /home/sunrise/fast_ws
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh 2>/dev/null || true
source install/setup.zsh 2>/dev/null || true
```

如果系统没有 `colcon`：

```bash
pip install -U colcon-common-extensions
```

## S100 编译

`mono_edgesam` 支持 X5/S100/S600，S100 编译必须显式传入 `-DPLATFORM_S100=ON`，否则 CMake 可能落到默认平台。

```bash
cd /home/sunrise/fast_ws
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh 2>/dev/null || true
source install/setup.zsh 2>/dev/null || true

colcon build --packages-select mono_edgesam \
  --allow-overriding mono_edgesam \
  --cmake-args -DPLATFORM_S100=ON
```

如果你的工作区没有覆盖 `/opt/tros/humble` 中的同名包，`--allow-overriding mono_edgesam` 可以省略。本工作区通常会覆盖系统包，所以建议保留。

编译成功后检查：

```bash
source install/setup.zsh
ros2 pkg prefix mono_edgesam
ros2 pkg executables mono_edgesam
```

期望能看到工作区安装路径和可执行文件：

```text
/home/sunrise/fast_ws/install/mono_edgesam
mono_edgesam mono_edgesam
```

S100 编译产物应包含：

```text
install/mono_edgesam/lib/mono_edgesam/mono_edgesam
install/mono_edgesam/lib/mono_edgesam/config/edgesam_encoder_1024.hbm
install/mono_edgesam/lib/mono_edgesam/config/edgesam_decoder_1024.hbm
install/mono_edgesam/lib/mono_edgesam/config/edgesam_encoder_512.hbm
install/mono_edgesam/lib/mono_edgesam/config/edgesam_decoder_512.hbm
```

## 准备测试图片

把要测试的 JPG 图片放到源码目录的 `config` 下，例如：

```text
src/mono_edgesam/config/desk1.jpg
```

渲染输出建议放到：

```text
src/mono_edgesam/output
```

开启 `dump_render_img:=1` 后，渲染图默认会写到当前运行命令所在目录。当前版本也支持通过 `dump_render_path` 参数指定输出目录。

## 手动画框

EdgeSAM/SAM 依赖输入框提示。测试自定义图片时，必须先给主体画一个大致 box，box 要完整包住主体，并比主体略大一点。

本包提供辅助脚本：

```text
src/mono_edgesam/test/select_box.py
```

在有图形界面的终端中运行：

```bash
cd /home/sunrise/fast_ws
src/mono_edgesam/test/select_box.py src/mono_edgesam/config/desk1.jpg --model-size 1024
```

操作方式：

- 鼠标拖拽框选主体。
- 按 `Enter` 或 `Space` 确认。
- 按 `Esc` 取消。

脚本会打印原图坐标和模型输入坐标，并给出可直接复制到 `ros2 run` 的参数，例如：

```text
ros2 params for 1024 model:
-p box_x1:=120.00 -p box_y1:=180.00 -p box_x2:=820.00 -p box_y2:=760.00
```

注意：

- 使用 1024 模型时，画框脚本必须传 `--model-size 1024`。
- 使用 512 模型时，画框脚本必须传 `--model-size 512`。
- box 坐标不是简单的原图像素坐标，而是按照本包本地回灌前处理缩放规则换算后的模型输入坐标。

## 运行本地 JPG 回灌预测

每个新终端先 source 环境：

```bash
cd /home/sunrise/fast_ws
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh 2>/dev/null || true
source install/setup.zsh
```

进入输出目录：

```bash
mkdir -p /home/sunrise/fast_ws/src/mono_edgesam/output
cd /home/sunrise/fast_ws/src/mono_edgesam/output
```

使用 1024 S100 模型运行本地 JPG 回灌预测。把最后一行的 `box_x1/box_y1/box_x2/box_y2` 替换为 `select_box.py` 输出的值：

```bash
ros2 run mono_edgesam mono_edgesam --ros-args \
  -p feed_type:=0 \
  -p image:=/home/sunrise/fast_ws/src/mono_edgesam/config/desk1.jpg \
  -p dump_render_img:=1 \
  -p encoder_model_file_name:=/home/sunrise/fast_ws/install/mono_edgesam/lib/mono_edgesam/config/edgesam_encoder_1024.hbm \
  -p decoder_model_file_name:=/home/sunrise/fast_ws/install/mono_edgesam/lib/mono_edgesam/config/edgesam_decoder_1024.hbm \
  -p box_x1:=120.00 -p box_y1:=180.00 -p box_x2:=820.00 -p box_y2:=760.00
```

如果要使用 512 模型，先用 `--model-size 512` 重新画框，再运行：

```bash
ros2 run mono_edgesam mono_edgesam --ros-args \
  -p feed_type:=0 \
  -p image:=/home/sunrise/fast_ws/src/mono_edgesam/config/desk1.jpg \
  -p dump_render_img:=1 \
  -p encoder_model_file_name:=/home/sunrise/fast_ws/install/mono_edgesam/lib/mono_edgesam/config/edgesam_encoder_512.hbm \
  -p decoder_model_file_name:=/home/sunrise/fast_ws/install/mono_edgesam/lib/mono_edgesam/config/edgesam_decoder_512.hbm \
  -p box_x1:=60.00 -p box_y1:=90.00 -p box_x2:=410.00 -p box_y2:=380.00
```

## 查看结果

运行成功后，日志会出现类似信息：

```text
Use SAM box: [120.00, 180.00, 820.00, 760.00]
Draw result to file: render_sam_feedback_0_0.jpeg
```

查看输出：

```bash
ls -lh /home/sunrise/fast_ws/src/mono_edgesam/output/render_sam_*.jpeg
```

渲染图会保存在运行命令所在目录，也就是：

```text
src/mono_edgesam/output
```

## 联动 DOSOD 自动画框并分割

如果希望输入一张图片和目标类别名，先由 `hobot_dosod` 检测目标框，再把检测框送入 `mono_edgesam` 分割，可以使用 `sam_with_dosod.launch.py`。

该流程适合示例：

```text
图片: /home/sunrise/fast_ws/src/mono_edgesam/config/2.jpg
目标: cup
输出: /home/sunrise/fast_ws/src/mono_edgesam/output/render_sam_*.jpeg
```

运行前先 source 环境：

```bash
cd /home/sunrise/fast_ws
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh 2>/dev/null || true
source install/setup.zsh
```

启动本地图片回灌、DOSOD 检测和 EdgeSAM 分割：

```bash
CAM_TYPE=fb ros2 launch mono_edgesam sam_with_dosod.launch.py \
  publish_image_source:=/home/sunrise/fast_ws/src/mono_edgesam/config/2.jpg \
  sam_image_width:=1696 \
  sam_image_height:=1280 \
  dosod_target_classes:=cup \
  dosod_score_threshold:=0.2 \
  sam_encoder_model_file_name:=edgesam_encoder_1024.hbm \
  sam_decoder_model_file_name:=edgesam_decoder_1024.hbm \
  sam_dump_render_img:=1 \
  sam_dump_render_path:=/home/sunrise/fast_ws/src/mono_edgesam/output
```

说明：

- `CAM_TYPE=fb` 表示使用本地图片回灌；如果不设置，会默认尝试启动 MIPI 相机。
- `dosod_target_classes:=cup` 表示 DOSOD 只保留 `cup` 检测框；如果图片里有多个 `cup`，多个检测框会一起送入 EdgeSAM。
- `sam_dump_render_img:=1` 表示保存与 mono_edgesam 示例一致的上色叠加图。
- `sam_dump_render_path` 指定渲染图输出目录。

运行时终端可能会一直打印类似：

```text
Smart fps: 17.00, pre process time ms: 5, infer time ms: 55, post process time ms: 16
```

这是正常现象，因为本地图片发布节点在循环发布同一张图片，DOSOD 和 EdgeSAM 会持续处理同一帧内容。看到 `output` 目录里已经生成 `render_sam_*.jpeg` 后，可以按 `Ctrl+C` 停止。

查看输出：

```bash
ls -lh /home/sunrise/fast_ws/src/mono_edgesam/output/render_sam_*.jpeg
```

## 浏览器实时分割显示

`mono_edgesam` 的实时浏览器显示复用 `websocket`，不需要单独写前端。链路是：

```text
相机或图片发布 -> /hbmem_img -> hobot_dosod检测目标框 -> /hobot_dnn_detection
              -> mono_edgesam按框分割 -> /perception/segmentation/edgesam
              -> websocket在浏览器叠加显示分割结果
```

`sam_with_dosod.launch.py` 已经同时启动：

- 图像发布或相机节点。
- `hobot_dosod` 检测节点。
- `mono_edgesam` 分割节点。
- `websocket` 浏览器显示节点。


### 外部 ROS Image 相机实时分割

如果相机不是由 `sam_with_dosod.launch.py` 启动，而是已经由其他节点发布普通 ROS Image，例如本工作区的 MVS 相机发布 `/left_camera/image`，使用 `CAM_TYPE=ros`：

```bash
ros2 launch mvs_ros_driver mvs_camera_trigger_launch.py
```

另开一个终端：

```bash
cd /home/sunrise/fast_ws
source /opt/ros/humble/setup.zsh
source /opt/tros/humble/setup.zsh 2>/dev/null || true
source install/setup.zsh

CAM_TYPE=ros ros2 launch mono_edgesam sam_with_dosod.launch.py
```

当前 `CAM_TYPE=ros` 模式的默认配置等价于：

```bash
CAM_TYPE=ros ros2 launch mono_edgesam sam_with_dosod.launch.py \
  sam_ros_img_sub_topic_name:=/left_camera/image \
  sam_codec_in_format:=bgr8 \
  sam_websocket_image_topic:=/image_mjpeg/image \
  dosod_target_classes:=cup \
  dosod_score_threshold:=0.3 \
  sam_encoder_model_file_name:=edgesam_encoder_512.hbm \
  sam_decoder_model_file_name:=edgesam_decoder_512.hbm \
  sam_cache_len_limit:=1 \
  sam_max_rois:=0 \
  sam_dump_render_img:=0
```

该模式下：

- `hobot_dosod` 和 `mono_edgesam` 都直接订阅 `sam_ros_img_sub_topic_name`。
- `hobot_codec` 将 `sam_ros_img_sub_topic_name` 编码成 `/image_mjpeg/image` 给浏览器显示。
- `websocket` 叠加 `/perception/segmentation/edgesam` 的分割结果。

如果相机实际发布 `rgb8`，把 `sam_codec_in_format:=bgr8` 改成：

```bash
sam_codec_in_format:=rgb8
```

参数含义：

- `CAM_TYPE=ros`：使用外部已经存在的 ROS Image topic 作为图像输入。这个环境变量必须保留，否则 launch 会按 MIPI/USB/本地回灌等其他模式启动。
- `sam_ros_img_sub_topic_name`：`hobot_dosod` 和 `mono_edgesam` 共同订阅的原始图像 topic，默认 `/left_camera/image`。
- `sam_codec_in_format`：原始图像编码格式，默认 `bgr8`；如果相机发布 `rgb8`，需要改成 `rgb8`。
- `sam_websocket_image_topic`：给浏览器显示用的 MJPEG 图像 topic，`CAM_TYPE=ros` 默认 `/image_mjpeg/image`。
- `dosod_target_classes`：DOSOD 要检测的目标类别，默认 `cup`。
- `dosod_score_threshold`：DOSOD 检测置信度阈值，默认 `0.3`；调高可减少误检，调低可减少漏检。
- `sam_encoder_model_file_name` 和 `sam_decoder_model_file_name`：EdgeSAM 模型文件，默认使用 512 模型以降低实时延迟。
- `sam_cache_len_limit`：`mono_edgesam` 图像缓存长度，默认 `1`，表示只保留最新帧，减少处理旧帧造成的显示滞后。
- `sam_max_rois`：每帧最多分割的检测框数量，默认 `0` 表示不限制；如果只想降低延迟，可以设成 `1` 或 `3`。
- `sam_dump_render_img`：是否额外保存本地上色渲染图，实时浏览器演示默认 `0` 关闭。

### 本地图片回灌的“准实时”浏览器显示

本地图片回灌会循环发布同一张图，所以浏览器里会持续看到同一张图的分割效果，终端也会持续打印 `Smart fps`。

```bash
CAM_TYPE=fb ros2 launch mono_edgesam sam_with_dosod.launch.py \
  publish_image_source:=/home/sunrise/fast_ws/src/mono_edgesam/config/2.jpg \
  sam_image_width:=1696 \
  sam_image_height:=1280 \
  dosod_target_classes:=cup \
  dosod_score_threshold:=0.35 \
  sam_encoder_model_file_name:=edgesam_encoder_512.hbm \
  sam_decoder_model_file_name:=edgesam_decoder_512.hbm \
  sam_cache_len_limit:=1 \
  sam_max_rois:=3 \
  sam_dump_render_img:=0
```

### 浏览器显示小提示

实时分割节点启动后，浏览器打开：

```text
http://<S100的IP地址>:8000/?camera=image
```

如果在 S100 本机桌面查看：

```text
http://127.0.0.1:8000/?camera=image
```

页面右上角或左侧 `Perception Information Display` 区域里，`Detection frames` 通常只显示检测框。要显示分割上色层，需要勾选：

```text
Full Image Segmentation
```

调试时注意：

- 如果网页只有检测框、没有分割上色，先确认 `Full Image Segmentation` 是否已勾选。
- 如果使用外部 ROS Image，例如 `/left_camera/image`，启动命令前必须带 `CAM_TYPE=ros`；否则 launch 会默认走 MIPI 或其他图像输入路径。
- `CAM_TYPE=ros` 模式下，浏览器图像 topic 是 `/image_mjpeg/image`，不是 `/image`。此时 `ros2 topic info /image` 显示 `Unknown topic` 是正常现象。
- `sam_max_rois:=1` 只会分割置信度最高的一个目标，多杯子场景中可能因为置信度波动出现分割目标来回跳动。多个杯子建议用 `sam_max_rois:=3` 或 `sam_max_rois:=0`。
- `ros2 topic echo /perception/segmentation/edgesam --once | grep features` 只能看到 `features:` 字段名，不能证明分割 mask 为空。更可靠的判断是看 `mono_edgesam` 日志里是否有 `features size: <非0>`。

### 实时低延迟参数

实时演示优先使用 512 EdgeSAM 模型：

- `edgesam_encoder_512.hbm` + `edgesam_decoder_512.hbm`：延迟低，适合浏览器实时显示。
- `edgesam_encoder_1024.hbm` + `edgesam_decoder_1024.hbm`：分割更精细，但 encoder/decoder 计算量更大，实时延迟会明显升高。

`mono_edgesam` 会先对整帧跑一次 encoder，再对每个检测框各跑一次 decoder。因此同一帧检测到多个 `cup` 时，分割耗时会随框数量增加。低延迟演示建议：

- `sam_max_rois:=3`：每帧最多分割 3 个检测框，适合同时演示多个杯子。如果希望所有 `cup` 都分割，改成 `0`；如果只追求最低延迟，改成 `1`。
- `sam_cache_len_limit:=1`：只保留最新图像，避免算法忙不过来时继续处理旧帧。
- `dosod_score_threshold:=0.35` 或更高：减少低置信度误检框，降低 decoder 次数。
- `sam_dump_render_img:=0`：浏览器实时显示时不要同时保存本地渲染图，避免额外的图像转换和写盘开销。

### 常用检查

确认浏览器叠加用的分割结果 topic 存在：

```bash
ros2 topic info /perception/segmentation/edgesam
```

确认浏览器图像 topic 存在：

```bash
ros2 topic info /image
```

如果使用 `CAM_TYPE=ros` 外部 ROS Image 模式，浏览器图像 topic 是：

```bash
ros2 topic info /image_mjpeg/image
```

如果网页只有图像没有分割结果，优先看 `mono_edgesam` 日志里是否有：

```text
out box size: <非0>
features size: <非0>
```

如果 `out box size: 0`，说明 `hobot_dosod` 没检测到目标，先降低 `dosod_score_threshold` 或确认 `dosod_target_classes` 是否和词表类别一致。

## 常用排查

确认运行的是工作区内重新编译出的包：

```bash
ros2 pkg prefix mono_edgesam
```

期望输出：

```text
/home/sunrise/fast_ws/install/mono_edgesam
```

如果模型加载失败，优先检查模型路径是否存在：

```bash
ls -lh /home/sunrise/fast_ws/install/mono_edgesam/lib/mono_edgesam/config/*.hbm
```

如果渲染图没有生成，确认运行命令里有：

```bash
-p dump_render_img:=1
```

如果自定义图片分割有明显整体偏移，优先检查两点：

- 是否重新编译并 source 了当前工作区的 `install/setup.zsh`。
- 是否使用 `select_box.py` 按当前模型尺寸重新生成了 box 参数。

如果边缘仍有轻微锯齿，通常来自 decoder 输出 mask 分辨率和量化模型特性。优先使用 1024 模型，并让 box 完整包住主体且略微留边。
