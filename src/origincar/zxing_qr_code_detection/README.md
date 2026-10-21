# zxing_qr_code_detection

基于 zxing-c++ 的 ROS 2 二维码识别节点。默认同时监听：

- `/aurora/rgb/image_raw`（`sensor_msgs/msg/Image`）
- `/image`（`sensor_msgs/msg/CompressedImage`）

输出与 `qr_code_detection` 保持一致，并使用 depth 1、Transient Local、Reliable QoS：

- `/qr_code_result`（`std_msgs/msg/String`）：二维码原始文本
- `/sign_switch`（`origincar_msg/msg/Sign`）：尾号为奇数时 `sign_data=3`，偶数时
  `sign_data=4`；同时兼容 `ClockWise`/`AntiClockWise`

默认首次成功识别后停止处理图片，与现有 `qr_code_detection` 的抢占式行为一致。

## 编译

```bash
sudo apt update
sudo apt install libzxingcore-dev
cd /userdata/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select origincar_msg zxing_qr_code_detection
source install/setup.bash
```

Ubuntu 22.04 中 rosdep 键 `libzxing-dev` 会安装 `libzxingcore-dev`（zxing-cpp
1.2.0）。

## 启动

```bash
ros2 launch zxing_qr_code_detection zxing_qr_code_detection.launch.py
```

只使用一个相机时，可把另一个话题设为空：

```bash
ros2 launch zxing_qr_code_detection zxing_qr_code_detection.launch.py \
  compressed_image_topic:='' raw_image_topic:=/camera/color/image_raw
```

可用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `raw_image_topic` | `/aurora/rgb/image_raw` | 原始图像话题，空字符串禁用 |
| `compressed_image_topic` | `/image` | 压缩图像话题，空字符串禁用 |
| `scan_interval_ms` | `150` | 每个输入话题的最小扫码间隔 |
| `stop_after_detection` | `true` | 首次识别后停止处理 |
| `try_harder` | `true` | 启用 zxing-cpp 深度搜索 |
| `try_rotate` | `true` | 尝试旋转方向 |
