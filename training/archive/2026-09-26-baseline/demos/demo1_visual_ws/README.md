# Demo 1 工作空间

这是 `origincar_control` 的教学骨架。输入是 `/win_detection`，输出是 `/cmd_vel`。先在车端 source TROS，再运行：

```bash
cd demo1_visual_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch demo1_visual_control demo1.launch.py
```

完成 `src/demo1_visual_control/src/visual_controller.cpp` 中的 TODO 后，再接入真实 YOLO 节点。完整任务说明见 `../../demos/demo1-visual.md`。
