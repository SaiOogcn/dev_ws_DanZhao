# Demo 1 工作空间

复制 `src/demo1_visual_control` 到 dove 工作空间后，选择性构建：

```bash
cp -a training/demos/demo1_visual_ws/src/demo1_visual_control /userdata/dev_ws/src/
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select demo1_visual_control
source install/setup.bash
```

`visual_controller.cpp` 保留 ROS 2 接口和安全零速占位，PID、YOLO 多框解析、避障、丢线回线和状态机都由学员完成。参见 `training/demos/README.md` 第 5 章。
