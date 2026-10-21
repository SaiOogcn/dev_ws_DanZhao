# qr_screen_display

这个包包含两个节点：

- `ring_photo_client`：收到路径控制器的一次性拍照触发后，从指定相机取最新图像，传给 PC，并发布 PC 返回的中文结果。
- `qr_screen_display`：在 HDMI/DP 外接屏幕显示二维码、行驶方向和中文图生文结果。

使用的话题：

- `/qr_code_result` (`std_msgs/msg/String`)：四位二维码数字
- `/sign_switch` (`origincar_msg/msg/Sign`)：3=顺时针，4=逆时针
- `/take_photo/trigger` (`std_msgs/msg/String`)：`front` 使用前置 YOLO 相机，`aurora` 使用车尾 Aurora
- `/take_photo/result` (`std_msgs/msg/String`)：PC 返回的中文图生文结果
- `/take_photo/status` (`std_msgs/msg/String`)：上传和推理过程状态

相机输入默认值：

- 前置：`/image` (`sensor_msgs/msg/CompressedImage`)
- 车尾：`/aurora/rgb/image_raw` (`sensor_msgs/msg/Image`)，上传前自动旋转 180 度

启动：

```bash
ros2 launch qr_screen_display qr_screen_display.launch.py \
  server_ip:=172.20.10.9
```

PC 端仍运行现有的 `pc_server_test0.2.py`，通信协议和原来的
`rdk_client_test_0.4.py` 一致。默认上传 512x512、JPEG 质量 60 的图片。

两个拍照位置在 `lio_hybird` 启动时独立调整，例如：

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  ring_photo_forward_path_point:=125 \
  ring_photo_reverse_path_point:=120
```

`/tmp/ring_photo_latest.jpg` 保存最近一次实际上传的图，便于现场确认取景和 Aurora 旋转方向。
