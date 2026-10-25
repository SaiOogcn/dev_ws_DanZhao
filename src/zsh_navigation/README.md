# zsh_navigation：全赛段规划与 CEM-MPC 源码

该包交付可独立导入的导航核心和 ROS 2 接入节点，不依赖 Gazebo、模型文件、碰撞真值或仿真轮角话题。定位输入由已经运行的 `lio_hybird` 或外部 NdtLIO 提供；此包不是新的定位建图器，也没有补齐 NdtLIO 地图持久化与重定位缺口。

## 代码路径

| 文件 | 职责 |
| --- | --- |
| [race_course.py](zsh_navigation/race_course.py) | A/B/C 全流程路线、方向和赛段定义 |
| [course_layout.py](zsh_navigation/course_layout.py) | 5×5 米场地约束，仅数值几何，无场景渲染 |
| [race_navigation.py](zsh_navigation/race_navigation.py) | 任务推进、换挡、局部重规划、后台规划与返回停车 |
| [frenet_corridor.py](zsh_navigation/frenet_corridor.py) | 五次平滑侧向偏移、多锥桶通道与航向连接 |
| [hybrid_corridor.py](zsh_navigation/hybrid_corridor.py) | 阿克曼航向格点、前进/倒车原语、Hermite 连接 |
| [race_mpc.py](zsh_navigation/race_mpc.py) | CEM 采样优化、转向动态、速度协调、整车碰撞预测 |
| [race_tuning.py](zsh_navigation/race_tuning.py) | 控制配置与执行器响应参数 |
| [obstacle_cloud.py](zsh_navigation/obstacle_cloud.py) | 激光截面聚类、锥桶底座恢复、保守障碍包络 |
| [pose_source.py](zsh_navigation/pose_source.py) | 场地坐标对齐、车体/传感器外参、时间插值 |
| [sensor_pipeline.py](zsh_navigation/sensor_pipeline.py) | 逐束扫描投影、时效检查、障碍记忆和 Twist 单位换算 |
| [full_race_navigation.py](zsh_navigation/full_race_navigation.py) | 真实 ROS 话题接入、显式启用、诊断与命令输出 |
| [audit.py](zsh_navigation/audit.py) | 不启动 ROS 的路线几何审查入口 |

`parking_entry.json` 只包含路径数值；场景生成、GUI、模型、图片、Gazebo 插件和试验报告均不属于该包。Frenet 与 CEM 为已有技术方法，本实现是面向当前比赛场地的组合实现，不宣称复现某篇 2026 年论文。

## 构建与预览

遵守工作空间 [AGENTS.md](../../AGENTS.md)：由用户在小车本机运行构建，禁止在 SSHFS 挂载目录编译。新增包独立于已有 `zsh_pkg` 和其他人的包，不覆盖其源码或安装规则。

```bash
cd /userdata/dev_ws
# 保持现场已使用的 ROS/TROS 环境，然后只构建新包。
colcon build --packages-select zsh_navigation --symlink-install
source install/local_setup.bash
ros2 pkg executables zsh_navigation
ros2 run zsh_navigation audit_course
ros2 launch zsh_navigation navigation.launch.py direction:=CW
```

逆时针使用 `direction:=CCW`。启动后处于禁用状态，`publish_drive=false`，不创建底盘命令发布器；收到正确输入后状态为 `DISABLED`。确认车辆处于规定起点且静止，可启用算法预览：

```bash
ros2 service call /zsh_navigation/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /zsh_navigation/status
ros2 topic echo /zsh_navigation/candidate_cmd_vel
```

预览输出不会驱动车辆，车不运动时不会自动完成赛段；它用于检查输入、路线、规划和输出方向。关闭算法使用同一服务的 `'{data: false}'`。发生锁存故障后需修正输入并重启节点；重新开始要求车辆回到规定起点，不能在半程把状态机重置为出发段。

## 输入与输出

| 接口 | 默认值 / 语义 |
| --- | --- |
| 位姿 | `/lio_hybird/odom`，父坐标 `lio_hybird_odom`，子坐标 `base_footprint` |
| 运动反馈 | `/odom`，子坐标 `base_footprint`，取纵向速度与角速度 |
| 雷达 | `/scan`，坐标 `laser`；起止时刻、束时间方向按驱动配置 |
| 启停服务 | `/zsh_navigation/enable`，`std_srvs/srv/SetBool` |
| 候选命令 | `/zsh_navigation/candidate_cmd_vel`，`geometry_msgs/msg/Twist` |
| 状态 | `/zsh_navigation/status`，`std_msgs/msg/String`，JSON 格式 |
| 路线显示 | `/zsh_navigation/reference`、`/zsh_navigation/local_plan`，`nav_msgs/msg/Path`，场地坐标 `field` |
| 底盘命令 | 仅显式配置 `publish_drive=true` 后提供 `command_topic`，默认 `/cmd_vel` |

配置见 [navigation.yaml](config/navigation.yaml)。`field_offset` 是定位参考坐标到场地的 SE(2) 变换；`body_to_pose_sensor` 表示定位输出点相对车体的外参，若位姿已是车体则保持零。`laser_in_body` 单独用于扫描投影，不重复扣减定位外参。

外部 NdtLIO 可以配置 Odometry 输入，或用 `pose_tf_topic`、`pose_frame`、`pose_tf_child` 选择独立 TF 话题中的一条明确变换。例如试运行 TF 的父坐标若确认为 `map`、子坐标确认为 `base`，可配置对应名称及 `/origincar_trial/tf`；实际名称必须从消息核实，不能仅照抄示例。位姿输入不会自动创建地图或执行全局重定位。

扫描逐束使用具有前后样本的位姿插值，不外推；若定位晚于扫描，暂存扫描等待数据，过期则停止供给有效控制。长时间不同步、错误坐标、位姿大幅跳变、控制计算超时和障碍数量超限会明确报错。障碍记忆仅适用于场上静态锥桶；不具备动态目标速度预测。

## 接入实车底盘

先完成预览、记录回放和现场标定，再复制一份参数文件作为实车配置。`publish_drive=true` 且 `hardware_calibrated=true` 后，启用服务才允许进入底盘输出；启动节点本身不会开始行驶。这两个参数是操作声明，不能替代实测证明。

```bash
ros2 launch zsh_navigation navigation.launch.py \
  config:=/你的实车参数文件绝对路径.yaml direction:=CW
```

本节点不实现与其他控制器的自动仲裁。直接输出到底盘前，应由现场操作者停用已有路径/视觉控制输出，确认唯一控制权；如保留多控制器，应让本节点输出到独立话题并接入经过验证的仲裁器。

底盘接口为 Twist：线速度 m/s，角速度 rad/s，`ω=v*tan(δ)/0.143`，倒车保留速度符号，零速输出零角速度。它不提供仿真中零速独立保持轮角的接口。节点停用时向已接管的底盘发送零速并停止输出；通讯中断或进程被强制终止时必须依靠底盘已验证的指令超时机制，节点本身无法保证收到最后一帧零速。

当前核心几何为轴距 `0.143m`、前悬轮廓 `0.2015m`、后向轮廓 `0.1375m`、半宽 `0.1095m`，部分高性能路径以 `0.0715m` 为后轴偏移。场地采用 5×5 米的数值边界，包含从图中估计的圆角；这些不是自动读取的实车标定。尺寸或场地不符时应修改并重新验证相应几何，不能只勾选确认参数。

默认速度限制为 `0.30m/s`，速度、横向加速度、制动和转向响应可在参数中设置。`profile=sprint` 选择完整规划/预测逻辑，不代表必须高速行驶。响应系数来自开发阶段试验，尚未在本车完成辨识，不能直接作为实车性能保证；`balanced` 保留另一套比较模式，其历史响应模型同样需要检查。

二维码、图生文和显示任务不在本节点内自动执行。当前方向由参数明确指定，避免把人工方向设置说成识码成功。完整任务扩展可接已有识码/拍照模块，但必须实现相应任务与控制权交接。

## 无硬件检查

无需 ROS 环境即可运行核心测试和几何审查，不会向车辆发布命令：

```bash
cd src/zsh_navigation
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 -m zsh_navigation.audit
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 -m pytest -q -p no:cacheprovider test
```

这些检查覆盖输入时效、逐束坐标变换、前后向指令符号、路线合法性、车辆与锥桶碰撞、规划连接及理想运动学任务推进，不代表本车已完成闭环验收。验收路径见 [完整实车验证文档](../../docs/zsh/04_validation_migration.md)。

本次源码整理的验证结果：44 项离线检查通过，包含独立进程规划器导入和 ROS 消息适配检查；顺、逆时针路线均无几何审查错误或赛段断点。ROS 适配检查只使用消息对象，没有创建节点、向底盘发命令或运行工作空间构建。
