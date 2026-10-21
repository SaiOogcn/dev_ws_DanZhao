# 教练验收基线

本文件给教练使用。新人只需要阅读三个 demo 讲义和对应的教学工作空间。完整参考路线保留在教练的车端工作空间中，用来核对接口、启动顺序和验收结果。

## 三条路线的完整组成

| 路线 | 节点或包 | 输入 | 输出 | 关键依赖 |
| --- | --- | --- | --- | --- |
| 视觉巡线 | `origincar_control` | `/win_detection`、二维码消息 | `/cmd_vel` | `ai_msgs`、厂商推理运行时、`origincar_msg` |
| 视觉安全层 | `visionpnav` | `/vision_cmd_vel`、`/scan`、二维码消息 | `/cmd_vel` | `origincar_control`、`ai_msgs`、`origincar_msg` |
| Nav2 + TEB | `my_robot_nav`、`dzsavepath`、`teb_local_planner` | 地图或 CSV 路径、TF、`/scan`、定位 | `/follow_path`、`/cmd_vel` | Nav2、`teb_msgs`、`costmap_converter`、`libg2o`、SuiteSparse、Eigen、Boost |
| LIO 导航 | `lio_hybird` | `/scan`、`/imu/data`、`/odom`、CSV 路径 | `/lio_hybird/odom`、`/cmd_vel` | Eigen、`visualization_msgs`、`tf2_geometry_msgs`、图像桥接包 |

## 视觉路线

基础路线只启动视觉控制器和底盘。若要启用安全仲裁，启动顺序是：

1. 底盘、相机和 YOLO 节点。
2. 视觉控制器，把输出重映射到 `/vision_cmd_vel`。
3. `visionpnav`，独占发布 `/cmd_vel`。

同一时间不能启动另一个发布 `/cmd_vel` 的视觉控制器、Nav2 控制器或 LIO 纯追踪节点。安全层的 `/scan` 角度零点必须先确认指向车头。

## Nav2 + TEB 路线

完整路线有两种路径来源：

- 录制路径：`dzfollowpath` 读取 CSV，创建 `FollowPath` Action goal，设置 `controller_id=TEB`，直接交给 `controller_server`。
- 全局规划：`my_robot_nav` 启动 Nav2 bringup，由 `planner_server` 生成全局路径，再由 `controller_server` 调用 TEB。

录制路径文件的 7 个数值字段是：`x,y,z,qx,qy,qz,qw`。车端保存器会在第 7 个字段后再写一个逗号，`dzfollowpath` 的解析器也按这个格式读取。完整 LIO 路径还可以追加第 8 列行驶方向：`1` 表示前进，`-1` 表示倒车。`dzfollowpath` 使用前 7 列，纯追踪控制器会读取方向列。培训包的 `path_source.py` 支持 3 列格式 `x,y,yaw` 和 7 列格式，也能跳过额外的方向列。路径发布器仍不会读取第 8 列；如果教学任务包含倒车段，需要在 FollowPath 或控制器层补充方向处理。

完整参数使用 `base_link`、`/odom_combined` 和 `map`。培训讲义中的 `base_footprint`、`odom` 是简化骨架的默认值。上车前必须统一 `robot_base_frame`、里程计话题和 TF 树。

启动前检查：

```bash
ros2 lifecycle get /controller_server
ros2 action list | grep follow_path
ros2 topic info /cmd_vel -v
ros2 run tf2_tools view_frames
```

完整参考配置中，`my_robot_nav` 的 Nav2 参数文件含有车端绝对路径和车体 frame 配置。复制到另一台机器时，必须把行为树路径改为安装空间路径，并检查 `BUILD_TESTING` 关闭时 launch、params、maps 和 behavior tree 是否仍会安装。

## LIO 路线

`lio_hybird` 的启动文件同时启动平面 LIO、纯追踪控制器、可选的黄色区域图像处理和 RViz。完整路线的核心数据流是：

```text
/imu/data + /odom -> 五状态预测
/scan -> 时间计算 -> 逐束去畸变 -> ICP/NDT -> /lio_hybird/odom
/lio_hybird/odom + CSV 路径 + /scan -> 纯追踪和局部轨迹采样 -> /cmd_vel
```

LIO 节点默认不发布 `base_footprint` 的父 TF，避免和底盘 EKF 竞争。控制器直接读取 `/lio_hybird/odom`。如果切换到 Nav2 路线，必须停止 LIO 纯追踪节点；如果继续运行 LIO 路线，必须停止 Nav2 `controller_server`。

完整 LIO 运行还需要安装路径文件和 keepout mask。路径坐标系固定在场地坐标中，初始位姿、雷达外参和 `scan_index_time_reversed` 必须根据车端实测值填写。

## 已有构建证据和验证边界

参考工作空间中可以看到 `origincar_control`、`my_robot_nav`、`dzsavepath`、`teb_local_planner`、`lio_hybird` 和 `lio_pure_pursuit` 的 `build/`、`install/` 产物。这个证据说明这些路线曾在目标 Linux/TROS 环境中完成过构建。`visionpnav` 的源码和 CMake 安装目标已经存在，但当前 `install/visionpnav` 中没有对应可执行文件，不能把它视为已验收组件。

教练端第一次准备环境时，先重新构建 `visionpnav`，再检查下面的文件是否出现：

```text
install/visionpnav/lib/visionpnav/visionpnav_safety
install/visionpnav/share/visionpnav/launch/visionpnav.launch.py
```

`my_robot_nav` 还有一个安装条件：它的 `CMakeLists.txt` 把 launch、params、maps、behavior_trees 和 scripts 的安装语句放在 `if(BUILD_TESTING)` 内。使用 `-DBUILD_TESTING=OFF` 构建时，这些资源不会安装。教练应把资源安装语句移到条件外，或在构建后确认 `install/my_robot_nav/share/my_robot_nav/` 下的资源齐全。参数文件中的行为树路径也要改成安装空间路径，不能保留车端绝对路径。

这些检查仍不能替代上车验收。教练还需分别验证：节点能否启动、TF 是否连通、消息 QoS 是否匹配、TEB 插件能否加载，以及车辆在低速下是否按预期运动。
