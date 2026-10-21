# 录包、回放和调试

## 固定录包模板

```bash
ros2 bag record -o ~/bags/demo_run \
  /scan /imu/data /odom /cmd_vel /tf /tf_static \
  /win_detection /demo3_lio/odom
```

视觉调试再加图像话题。不要默认录全量图像，否则磁盘和回放速度会成为新的问题。

## 回放

```bash
ros2 bag info ~/bags/demo_run
ros2 bag play ~/bags/demo_run --clock
```

启动节点时将 `use_sim_time` 设为 `true`。回放时先看输入频率，再看输出频率，最后看 RViz 轨迹。每次实验记录：代码版本、参数、障碍物摆放、成功/失败、最小距离和下一步改动。

## 调试顺序

1. 节点是否启动，参数是否来自当前工作空间。
2. 输入话题是否存在，消息类型和 QoS 是否匹配。
3. TF 是否连通：`ros2 run tf2_ros tf2_echo odom base_footprint`。
4. 输出是否有合理时间戳、频率和数值。
5. 最后才改算法参数。一次只改一组参数。
