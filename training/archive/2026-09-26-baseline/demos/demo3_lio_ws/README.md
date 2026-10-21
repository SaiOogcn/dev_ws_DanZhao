# Demo 3 工作空间

这是二维 LIO 的教学骨架。输入是 `/scan`、`/imu/data` 和 `/odom`，输出是 `/demo3_lio/odom`。先用 rosbag 回放，再上车：

```bash
cd demo3_lio_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch demo3_lio demo3.launch.py
```

完成扫描去畸变、假点过滤、外参变换、轮速和 IMU 预测、NDT/ICP 配准与观测拒绝。完整任务说明见 `../../demos/demo3-lio.md`。
