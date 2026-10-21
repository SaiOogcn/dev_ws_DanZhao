# Demo 2 工作空间

这是 Nav2 + TEB 的教学骨架。默认只发布录制路径，不启动 `controller_server`，这样可以先检查路径和 TF。确认车端安装了兼容的 TEB 插件后再运行：

```bash
cd demo2_nav2_teb_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch demo2_nav2_teb demo2.launch.py path_file:=/path/to/path.csv
ros2 launch demo2_nav2_teb demo2.launch.py path_file:=/path/to/path.csv start_controller:=true
```

完成 FollowPath Action、lifecycle、TF bridge 和全局规划器选择。完整任务说明见 `../../demos/demo2-nav2-teb.md`。
