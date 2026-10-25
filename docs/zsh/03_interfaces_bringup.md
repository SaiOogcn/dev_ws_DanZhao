# ROS 接口、TF、控制权与实车启动

[返回总说明](../../README.md)

本文命令供现场操作者使用。新增算法包为 [zsh_navigation](../../src/zsh_navigation/README.md)，源码整理期间未在小车构建或驱动车辆。

## 1. 当前接口契约

| 话题 | 类型 | 含义 / 使用注意 |
| --- | --- | --- |
| `/scan` | `sensor_msgs/msg/LaserScan` | 定位与避障原始输入；核对 frame、扫描时间和 QoS |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | 底盘原始 IMU |
| `/imu/data` | `sensor_msgs/msg/Imu` | 上层处理后的 IMU；确认启动来源 |
| `/odom` | `nav_msgs/msg/Odometry` | 底盘轮速里程计，`odom` / `base_footprint` |
| `/odom_combined` | `nav_msgs/msg/Odometry` | 既有 EKF 输出，与原始 `/odom` 不同 |
| `/robotvel` | `origincar_msg/msg/Data` | x 为纵向速度，z 为角速度；不是 Odometry |
| `/lio_hybird/odom` | `nav_msgs/msg/Odometry` | 当前二维 NDT-EKF 输出；其 frame 为 `lio_hybird_odom` |
| `/lio_hybird/submap` | `nav_msgs/msg/OccupancyGrid` | 当前子图显示 |
| `/lio_hybird/tracking_path` | `nav_msgs/msg/Path` | 当前跟踪路线 |
| `/lio_hybird/local_plan` | `nav_msgs/msg/Path` | 当前局部计划 |
| `/lio_hybird/obstacles` | `sensor_msgs/msg/PointCloud2` | 控制侧障碍显示 |
| `/lio_hybird/state` | `std_msgs/msg/String` | 控制状态诊断 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 默认底盘路径：`linear.x` m/s，`angular.z` rad/s |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | 仅在对应底盘路径启用并验证后使用；转角 rad |
| `/sign_switch` | `origincar_msg/msg/Sign` | 二维码任务结果，字段 `sign_data`；编码以识码与消费源码共同定义为准 |
| `/origincar_control/control_released` | `std_msgs/msg/Bool` | 原视觉控制释放确认 |
| `/take_photo/trigger` | `std_msgs/msg/String` | 拍照任务输入 |
| `/take_photo/result` | `std_msgs/msg/String` | 推理/拍照任务结果，业务语义见对应客户端 |
| `/take_photo/status` | `std_msgs/msg/String` | 拍照工作状态 |

表中默认名称受实际启动重映射影响。检查消息头和参数，不能仅凭话题同名判断接线正确。`String` 状态不是严格的错误码协议；迁移时如需自动恢复，应先定义稳定状态与原因编码。

## 2. 独立 NdtLIO 试运行接口

仅适用于小车已经安装该版本 `zsh_pkg` 的情况，本仓库本次未新增此包：

| 输出 | 消息 | 注意 |
| --- | --- | --- |
| `/origincar_trial/map` | OccupancyGrid | 建图显示与栅格保存 |
| `/origincar_trial/trajectory` | Path | 位姿轨迹显示 |
| `/origincar_trial/scan_point_cloud` | PointCloud2 | 扫描显示 |
| `/origincar_trial/ndt_odom` | Odometry | 发布频率、frame 和模式分支须补验，不能直接当作完整控制输入 |
| `/origincar_trial/initialpose` | PoseWithCovarianceStamped | 输入初值，不代表自动载入定位地图 |
| `/origincar_trial/tf`、`/origincar_trial/tf_static` | TFMessage | 独立 TF 话题；监听器需相同重映射 |

原始订阅仍需核对 `/scan`、`/imu/data` 和 `/robotvel`；命名空间不一定会改变源码中的绝对话题。纯 LO 模式不订阅后两者。

新增 `zsh_navigation` 已提供 Odometry/指定 TF 边的选择、坐标转换、时间插值及轮速接入；它不修改定位器本身。定位质量、地图版本、重定位成功和协方差的统一协议仍需扩展，不能用当前时效检查替代全部定位质量判定。

## 3. TF 所有权与参考点

当前默认底盘链路：

```text
odom --既有融合器--> base_footprint --> base_link
                                            ├── gyro_link
                                            └── laser
```

当前启动文件给出 `base_link → laser=(-0.03, 0, 0.23)m`、`base_link → gyro_link=(0, 0, 0.08)m`。它们是仓库配置，移到另一台车必须重新核实。二维滤波器的平面激光偏移与三维静态 TF 应保持一致。

现有 `lio_hybird` 不再重复发布 `odom → base_footprint`，而使用独立 `lio_hybird_odom` 和可选桥接。引入 NdtLIO 后，明确哪一个节点拥有全局修正，哪些节点只提供观测；TF 不能出现一个子坐标同时被两个父坐标发布。

场地坐标与地图坐标的对齐参数单独版本化。路径、障碍点云、控制器和可视化使用同一场地定义；不能用重命名 frame 字符串代替数值坐标变换。

## 4. 底盘单位和两条控制路径

依据 [base_serial.launch.py](../../src/base_orin/launch/base_serial.launch.py)、[base_orin.cpp](../../src/base_orin/src/base_orin.cpp) 以及 [转换脚本](../../src/base_orin/scripts/cmd_vel_to_ackermann_drive.py)：

- 默认 `akmcar=false`：串口走 Twist 回调，线速度与角速度转为下位机数值；角速度还叠加历史曲率补偿 `v * chassis_curvature_compensation`。
- `akmcar=true`：启用 Ackermann 路径与转换。现有串口回调对 steering_angle 还有缩放，必须依据下位机协议验证实际意义。
- 转换脚本使用 `L=0.143m`，这是轴距参数；应实测确认，不能把它当作轮距。

若 MPC 内部输出 `(v, δ)`，默认 Twist 路径转换为：

```text
linear.x = v
angular.z = v * tan(δ) / L
```

反向换算 `δ=atan(L*ω/v)` 仅对非零速度适用。零速时输出零角速度，不能用人为最小正速度规避除零而改变方向或产生尖峰。默认路径无法据此表达“原地转轮并保持角度”；如果需要，必须确认 Ackermann 与下位机支持，不能让阿克曼车执行差速车原地转向。

## 5. 唯一控制权

既有交接流程为：视觉控制负责二维码前段 → 识码事件 → 视觉控制停止并发布释放确认 → 路径控制确认实际速度低于约 `0.04m/s` 后接管。停车路线需显式设 `post_qr_route_mode:=parking`，默认值不能当作同一流程。

`ros2 topic info -v /cmd_vel` 可以列出发布器，但无法证明它们同时发了命令；有的节点只是保留发布器。需要结合各阶段实际发布时间和内容检查。反过来，仅看到一个话题也不能证明控制权正确。

新 MPC 上车时先停用原路径控制输出，或增加单一仲裁器。候选命令必须有来源、生成时间和有效期；仲裁器拒绝过期指令，失去控制器心跳时让下位机按验证过的看门狗停车。仅发一次零速不足以证明故障停车可靠。

## 6. 实车操作顺序

### 只读检查

在小车终端执行：

```bash
cd /userdata/dev_ws
source install/local_setup.bash
ros2 node list
ros2 pkg executables lio_hybird
ros2 topic list -t
ros2 topic info /scan -v
ros2 topic info /cmd_vel -v
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
```

使用 `ros2 topic hz /scan` 观察频率后 Ctrl+C 退出；频率正常仍要核对 header、抖动和测量时间。不要启动第二个相同驱动来“修复”已有话题。

### 现有基线

如果底盘与传感器尚未启动，由现场操作者按原车启动方案启动；本仓库底盘入口为 `ros2 launch base_orin origincar_bringup.launch.py`，它不代表雷达和相机都已经启动。

确认坐标、路径和控制权后，现有基线入口为：

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  pre_qr_control_mode:=external post_qr_route_mode:=parking
```

原视觉前段按需另开终端：

```bash
cd /userdata/dev_ws
source install/local_setup.bash
ros2 run origincar_control origincar_control
```

核实该程序键盘启动步骤与当前场地。不要同时再启动一个直接发 `/cmd_vel` 的新 MPC。

### 独立 NdtLIO 试运行

仅当原 `zsh_pkg` 已安装、对应配置已核实，且不会与当前定位 TF 冲突时使用：

```bash
ros2 pkg executables zsh_pkg
ros2 launch zsh_pkg ndt_lio_node.launch.py
```

默认试运行是纯雷达建图。它不是保存地图后自动重定位的完整启动命令，也不是本文目标 MPC 的入口。

### 需要重新编译时

新增 MPC 包由用户在**小车本机**定向构建并安装，默认仅预览：

```bash
cd /userdata/dev_ws
colcon build --packages-select zsh_navigation --symlink-install
source install/local_setup.bash
ros2 launch zsh_navigation navigation.launch.py
```

默认读取 `/lio_hybird/odom`、`/odom`、`/scan`，输出 `/zsh_navigation/candidate_cmd_vel`、`/zsh_navigation/status` 和路径。`/zsh_navigation/enable` 为 SetBool 服务。真实底盘输出需明确配置并完成标定，详见 [包说明](../../src/zsh_navigation/README.md)。

若修改 `lio_hybird`，只由用户在**小车本机**执行以下示例，先确认所需依赖已安装：

```bash
cd /userdata/dev_ws
colcon build --packages-select lio_hybird --symlink-install
source install/local_setup.bash
```

若变更自定义消息，则先确定受影响依赖再选择包构建。新控制器使用独立的 `zsh_navigation` 包，不覆盖小车既有 `zsh_pkg` 的 CMake 或入口。

禁止在 SSHFS 挂载目录运行构建，禁止清理其他人的构建产物。遵守根目录 [AGENTS.md](../../AGENTS.md)。

## 7. QoS、时效和日志

激光/IMU 应与驱动的 SensorData QoS 兼容；静态地图或路径若使用 transient-local，订阅者也要匹配。实际 QoS 用 `ros2 topic info -v` 检查，不猜测所有话题都是同一策略。

控制读取最近的有效测量时间而非仅接收时间，分别统计扫描年龄、位姿年龄和总执行延迟。超时阈值由控制速度和停止距离决定；定位 frame 不匹配或时间倒退时使当前轨迹失效。

选择实际存在的话题记录 rosbag：`/scan`、`/imu/data`、`/odom`、主定位输出、`/cmd_vel`、`/tf`、`/tf_static`、控制状态与局部路径。独立 NdtLIO 的 TF 话题需按其重映射另外记录。建议模型状态和求解诊断作为后续新增日志接口，不把它们列成当前已有话题。

## 8. 拍照与图生文扩展

现有 [ring_photo_client_node.py](../../src/origincar/qr_screen_display/qr_screen_display/ring_photo_client_node.py) 使用工作线程和 ZMQ 请求外部服务，具有超时与重试处理。相机图像编码、前视/深度相机来源、服务地址、请求协议和服务器启动方法必须与现场环境匹配。

有客户端源码不代表已有模型服务可用；没有得到真实结果时明确报告缺失，不阻塞高频控制回调，也不将历史服务地址当作通用配置发布。显示接口详见 [原包 README](../../src/origincar/qr_screen_display/README.md)。
