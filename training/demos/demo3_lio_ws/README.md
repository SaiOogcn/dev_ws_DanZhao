# Demo 3 工作空间

复制 `src/demo3_lio` 到 dove 工作空间后，选择性构建：

```bash
cp -a training/demos/demo3_lio_ws/src/demo3_lio /userdata/dev_ws/src/
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select demo3_lio
source install/setup.bash
```

LIO 节点保留 IMU、odom、雷达数据接口和复制 odom 的占位输出；去畸变、去噪、外参、预测、配准、子地图和观测拒绝由学员完成。控制器只保留雷达停车和零速安全骨架。参见 `training/demos/README.md` 第 7 章。

`rviz/demo3.rviz` 是配套的 RViz 配置，Fixed Frame 为 `odom`，默认不启动，加 `rviz:=true` 打开。
