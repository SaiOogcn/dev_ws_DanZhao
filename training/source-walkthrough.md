# 代码阅读索引

这份索引把培训文档中的概念对应到培训包内的教学文件。交付培训包后，队员只需要阅读 `training` 目录。

教练核对完整实现时，按下面的包边界定位问题：视觉路线看 `origincar_control` 和可选的 `visionpnav`；Nav2 路线看 `my_robot_nav`、`dzsavepath` 和 `teb_local_planner`；LIO 路线看 `lio_hybird` 的 LIO 节点与纯追踪节点。完整源码不随新人资料交付。

## 底盘和消息

| 主题 | 文件 | 阅读重点 |
| --- | --- | --- |
| Twist 输入 | `architecture.md` | `/cmd_vel` 的消息类型和单控制器约束 |
| 阿克曼输入 | `architecture.md` | 速度、转向角和底盘接口的关系 |
| 速度转换 | `architecture.md` | `steering = atan(wheelbase * angular.z / linear.x)` |
| 轮速和 IMU | `basics/ros2-basics.md` | 输入话题、QoS 和时间戳 |
| EKF | `architecture.md` | 参与融合的字段、frame 和 `publish_tf` |

## 视觉控制

| 主题 | 文件和函数 | 阅读重点 |
| --- | --- | --- |
| 消息解析 | `demos/demo1_visual_ws/src/demo1_visual_control/src/visual_controller.cpp:best_detection` | `targets → rois → Detection` |
| 线路选择 | 同上：`on_detections`、`make_command` | 线路框中心、图像中心和横向误差 |
| 障碍物触发 | 同上：`make_command` | 框高比例、触发阈值和速度限制 |
| 绕行方向 | `demos/demo1_visual_ws/README.md` | 障碍物中心、线路中心和绕行侧锁存设计 |
| 控制分支 | 同上：`make_command` | 丢线、巡线、减速和停车的优先级 |
| 控制权切换 | `architecture.md` | 单一 `/cmd_vel` 发布者和停车约定 |
| PID | `demos/demo1-visual-deep.md` | `dt`、积分、微分、限幅和 anti-windup |

## LIO 定位

| 阶段 | 函数 | 关键问题 |
| --- | --- | --- |
| 输入和参数 | `demos/demo3_lio_ws/src/demo3_lio/src/lio_student_node.cpp` | 话题、frame、外参、队列和门限 |
| IMU 缓存 | `on_imu` | 时间戳顺序、历史长度和线程安全 |
| 轮速观测 | `on_odom` | 只融合纵向速度，避免重复使用轮速位姿 |
| 时间插值 | `on_scan` TODO 1 | 历史覆盖和扫描首尾时间 |
| 束时间 | `demo3-lio-deep.md` | N10 顺序、消息时间语义 |
| 去畸变 | `on_scan` TODO 1 | 外参、逐束运动补偿和范围过滤 |
| ICP | `on_scan` TODO 3 | 最近邻、点到线残差、Huber 和 RMSE |
| 子地图 | `on_scan` TODO 4 | 关键帧体素均值、协方差和信息矩阵 |
| NDT | `on_scan` TODO 4 | 马氏距离、Hessian、迭代和观测拒绝 |
| 调度 | `on_scan` | 扫描排队、时间覆盖和丢旧帧 |
| 输出 | `publish_estimate` | `/demo3_lio/odom`、路径和诊断 |

## 路径跟踪和避障

`demo2_nav2_teb_ws` 负责读取 CSV 路径并发布 `nav_msgs/msg/Path`。`demo3_lio_ws` 预留位姿、路径和诊断输出，队员需要在 TODO 位置补齐时间同步、配准和观测拒绝。

TEB 的 `FollowPath`、局部 costmap、参数和失败原因见 [Demo 2 深入讲义](demos/demo2-nav2-teb-deep.md)。运行时必须保持单一 `/cmd_vel` 控制器。

## 阅读方法

每次读一个回调，写出四行：输入消息、更新的状态变量、输出话题、异常时的行为。再用 `ros2 topic echo` 和 `ros2 topic hz` 验证这四行。这样能把“代码看懂了”变成可运行的接口理解。
