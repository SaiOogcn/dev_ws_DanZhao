# 环境准备

## 目标

你需要得到一台 Ubuntu 22.04 电脑，能启动 ROS 2 Humble/TROS，能通过 SSH 访问小车，并能在 VS Code 中打开车端工作空间。双系统安装属于高风险操作，先备份个人文件，再按电脑厂商文档制作启动 U 盘。

## 安装顺序

1. 在 Windows 中备份数据，关闭快速启动，确认 UEFI 模式。
2. 安装 Ubuntu 22.04，给系统预留至少 80 GB 空间。显卡驱动先用系统推荐版本。
3. 安装 Git、VS Code、Python 3 和基础编译工具。
4. 在车端安装或加载 TROS/ROS 2 Humble。培训电脑的发行版必须和车端消息接口兼容。
5. 安装 ROS 2 常用包：`rclcpp`、`rclpy`、`rviz2`、`robot_localization`、`nav2-bringup`、`rosbag2`。

```bash
sudo apt update
sudo apt install git build-essential python3-colcon-common-extensions \
  python3-rosdep python3-pip openssh-client
source /opt/tros/humble/setup.bash
```

## VS Code + Codex 工作方式

在 VS Code 中安装 Remote - SSH，使用车端账号打开 `/userdata/dev_ws`。Codex 用来读代码、生成小范围补丁和解释日志；构建、传感器驱动和真车测试仍在车端终端完成。修改前先运行 `git status`，每次只提交一个可说明的实验。

```bash
ssh sunrise@192.168.127.10
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select <包名>
source install/setup.bash
```

## 最小检查表

- `echo $ROS_DISTRO` 能输出 `humble` 或车端对应发行版。
- `ros2 doctor --report` 没有明显 DDS 错误。
- `ros2 topic list` 能看到 `/scan`、`/imu/data` 或 `/odom` 中至少一个真实输入。
- `rviz2` 能启动，且 Fixed Frame 与当前 TF 树一致。
