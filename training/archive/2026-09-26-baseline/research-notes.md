# 调查依据

## 工作空间代码

本次设计阅读了以下源码和文档：

- `remote_dev_ws/src/origincar/origincar_control/src/origincar_control.cpp`：YOLO 结果遍历、线中心计算、障碍物高度比例、PID 和速度发布。
- `remote_dev_ws/src/origincar/origincar_base`：底盘串口、轮速、IMU 和 `robot_localization` EKF 启动配置。
- `remote_dev_ws/src/origincar/lio_hybird/README.md`：五状态平面 LIO、扫描去畸变、关键帧 NDT、局部避障和控制权说明。
- `remote_dev_ws/src/origincar/lio_ob`：LIO 实验分支和 RViz 配置。
- `remote_dev_ws/src/origincar/lio_hybird/STAGE_HANDOFF_20260806.md`：Nav2 + TEB 接管范围、`FollowPath`、局部代价地图、参数和失败案例。
- `remote_dev_ws/teamrx-dovesmart-main/reports/团队赛后总结报告.md`：硬件清单、话题、TF 约束、比赛最终基线和技术债。
- `remote_dev_ws/records/pre_qr_pid_run_01~03`：已有 rosbag、PID 对齐 CSV 和复盘数据。

## 网络资料

- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 官方文档](https://docs.nav2.org/)
- [ROS 2 TF2 教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [TEB Local Planner](http://wiki.ros.org/teb_local_planner)
- [ronjinliang/NdtLIO](https://github.com/ronjinliang/NdtLIO)：重点阅读 `Frontend`、增量二维 NDT、ESKF/IESKF、IMU 静止初始化、雷达束时间和 `T_IL` 外参说明。

## 设计取舍

三个 demo 都输出导航结果，但把难点拆开：视觉 demo 不引入定位，Nav2 demo 不要求新人重写 LIO，LIO demo 暂不要求比赛级局部避障。每个骨架都保留真实话题名、车体坐标系和已有参数作为起点，同时把最容易掩盖问题的部分标成 TODO。

## 版本和环境限制

仓库实际使用 TROS/ROS 2 Humble、RDK X5 和 Ubuntu 22.04。当前工作机是 Windows，缺少 `ament_cmake`、`ai_msgs`、TROS 驱动和车端 TEB 插件，因此这里只能完成静态检查和 Python 语法检查。三个工作空间必须在 Ubuntu/TROS 车端重新构建，并用真实话题或 rosbag 验证。
