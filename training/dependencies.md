# 运行依赖和交付清单

本文件给教练使用。队员只拿到 `training` 目录；教练在 Linux 或车端工作空间中准备下面的依赖。依赖分为“编译必需”和“上车运行必需”，不要把厂商源码直接复制到培训包。

## 所有 demo 共用

编译工作空间至少需要：

- Ubuntu 22.04。
- 车端使用 RDK/TROS Humble（`/opt/tros/humble/setup.bash`）；普通 Ubuntu ROS 2 Humble 只用于离线练习。
- `ament_cmake`、`ament_python`、`colcon`、`rosdep`、`python3`。
- `rclcpp`、`rclpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`。
- `tf2`、`tf2_ros`、`tf2_geometry_msgs`。
- `rosbag2`、对应的存储插件和 `rviz2`。

上车运行还需要：

- 底盘驱动：接收 `/cmd_vel` 或 `/ackermann_cmd`，发布 `/odom`。
- IMU 驱动或姿态节点：发布 `/imu/data`。
- 2D 雷达驱动：发布 `/scan`，并提供 `laser` 到车体的静态 TF。
- 正确的时钟配置。录包回放时使用 `/clock` 和 `use_sim_time`。

## Demo 1：纯视觉巡线

教学骨架只需要：

- `ai_msgs`，提供 `ai_msgs/msg/PerceptionTargets`。
- `geometry_msgs`。
- 与该消息格式匹配的 YOLO 推理节点或录制好的 `/win_detection` rosbag。

上车运行还需要：

- 相机驱动、YOLO 模型节点和厂商推理运行时。完整车端视觉节点还使用 `dnn_node`、`hbm_img_msgs` 和 `hobot_cv`。
- `origincar_msg`，用于二维码接管消息；纯视觉基础练习可以先关闭二维码分支。
- 相机分辨率、检测类别名和骨架中的 `line`、`hat` 保持一致。
- 底盘控制节点，并确认只有视觉节点发布 `/cmd_vel`。

如果启用车端的激光安全层，还要保留 `visionpnav`。它把视觉节点的 `/vision_cmd_vel` 接到安全仲裁器，再由仲裁器发布 `/cmd_vel`，负责减速、刹停、有限后退和二维码交接。

没有相机和 YOLO 时，队员仍可用录包完成消息解析、状态机和 PID 调试。`ai_msgs` 仍是编译依赖；如果培训电脑没有它，应由教练提供消息包或预编译环境。

## Demo 2：Nav2 + TEB

编译路径发布器需要：

- `rclpy`、`nav_msgs`、`geometry_msgs`。
- `nav2_msgs`，用于 `FollowPath` Action client。
- `nav2_controller`、`nav2_costmap_2d`、`nav2_util` 和 `pluginlib`。

完整车端路线还要保留：

- `my_robot_nav`，负责调用 Nav2 官方 `bringup_launch.py`，加载地图和参数。
- `dzsavepath`，其中 `dzfollowpath` 已实现 `FollowPath` Action client、路径文件解析、TEB 控制器选择和局部代价地图清理后重试。
- 工作空间内的 ROS 2 TEB 插件 `teb_local_planner`，以及它的 `teb_msgs`、`costmap_converter`、`costmap_converter_msgs`、`libg2o`、SuiteSparse、Eigen 和 Boost 依赖。ROS 1 的 TEB 插件不能直接装到 Nav2 `controller_server`。
- Nav2 的 `lifecycle_manager` 和完整 bringup 组件。
- 提供 `map -> odom`、`odom -> base_footprint`、`base_footprint -> laser` 的 TF 节点。
- `/scan` 雷达输入和 `/odom` 或其他定位输入。
- 如果选择全局规划器，再保留 `nav2_planner`、地图服务和对应的地图文件。
- 如果使用录制路径，保留路径文件和 `dzfollowpath`；如果选择 Nav2 全局规划，则保留地图、AMCL 或其他定位节点，以及 `planner_server`。

教练第一次构建时要检查两个已知问题：`visionpnav` 源码虽然有 `visionpnav_safety` 安装目标，但现有安装目录可能没有可执行文件，必须重新构建并用 `ros2 pkg executables visionpnav` 验证；`my_robot_nav` 的资源安装语句位于 `BUILD_TESTING` 分支内，关闭测试构建后可能缺少 launch、参数、地图和行为树。复制到新机器时，还要把参数文件中的行为树绝对路径改成安装空间路径。

培训骨架只保留路径发布和参数入口，便于队员自己补齐控制链。完整参考路线已经具备 `controller_server`、TEB 插件、生命周期和 `FollowPath` client，可以作为教练验收基线。

## Demo 3：二维 LIO

编译 `demo3_lio_ws` 需要：

- `rclcpp`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf2`、`tf2_ros`。
- Eigen 3 开发包。建议在实现矩阵、Hessian 和协方差时使用系统 Eigen，不要手写矩阵运算。

上车运行还需要：

- `/scan`、`/imu/data` 和 `/odom` 三路输入。
- 雷达束时间信息有效，且 IMU 队列覆盖扫描首尾时间。
- 实测的雷达到车体、雷达到 IMU 外参。
- 可选的静态 TF 发布器和 RViz 2 配置。

完整 LIO 路线还需要：

- `lio_hybird` 的两个节点：平面 LIO 节点和纯追踪控制器。
- `visualization_msgs`、`tf2_geometry_msgs`、Eigen 3 和 `ai_msgs`。
- `image_bridge_hsv`、`image_binary`。它们用于上环视觉定位和安全边界观察；只做雷达 LIO 时可以关闭环视功能。
- 录制路径、场地 keepout mask 和与路径坐标系一致的初始位姿。

当前培训骨架可以构建并启动，但在 TODO 补完前只会发布轮速里程计的占位结果。完整参考路线已经实现 IMU/轮速预测、逐束去畸变、假点和墙体处理、NDT 子地图、ICP 兜底、纯追踪、局部采样避障和恢复状态机。补完教学骨架后，仍要用静止、直行和原地转动三段 rosbag 验证，不能仅凭“节点启动成功”判定 LIO 可用。

## 建议保留的车端目录

教练自己的运行工作空间至少保留以下类别：

1. 培训包 `training`。
2. 厂商底盘、相机、雷达和 IMU 驱动包。
3. 厂商消息和推理包，至少包括 `ai_msgs`、`origincar_msg`、`dnn_node`、`hbm_img_msgs` 和 `hobot_cv`。
4. `robot_localization` 或车端既有状态估计节点。
5. Nav2、`my_robot_nav`、`dzsavepath`、ROS 2 TEB 插件及其数值优化依赖。
6. `lio_hybird` 运行所需的 Eigen、`visualization_msgs`、图像桥接节点、路径和 keepout mask。
7. `rosbag2`、`rviz2`、TF 工具和运行所需的参数文件。

不需要交给队员的内容：比赛控制器、完整 LIO 实现、原始串口协议实现、未裁剪的历史 rosbag 和其他 demo 的参考答案。
