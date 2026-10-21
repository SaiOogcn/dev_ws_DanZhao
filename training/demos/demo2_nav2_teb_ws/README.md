# Demo 2 工作空间

复制 `src/demo2_nav2_teb` 到 dove 工作空间后，选择性构建：

```bash
cp -a training/demos/demo2_nav2_teb_ws/src/demo2_nav2_teb /userdata/dev_ws/src/
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select demo2_nav2_teb
source install/setup.bash
```

这是 Nav2 + TEB 全栈框架。地图、TF/EKF、costmap、TEB 参数和 FollowPath 核心逻辑均留有 TODO；`student_ready:=false` 时不会发送 goal。参见 `training/demos/README.md` 第 6 章。

`rviz/demo2.rviz` 是配套的 RViz 配置，Fixed Frame 为 `map`，含参考路径和两个代价地图，默认不启动，加 `rviz:=true` 打开。
