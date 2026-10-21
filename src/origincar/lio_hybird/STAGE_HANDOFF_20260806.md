# lio_hybird 阶段开发与调试交接（2026-08-06）

> 历史记录说明：本文记录的 Nav2/TEB 方案已从当前 `lio_hybird` 源码、launch、参数和依赖中移除；当前运行版本只使用内置纯追踪和局部避障。文中的 TEB 内容仅保留作历史背景，不代表当前可用启动方式。

## 1. 当前结论

扫码后从下半场进入中间通道，当前应以 **倒车揉库方案 `parking`** 作为后续开发和真车调试基线。

- `parking`：保留明确的前进、倒车、再前进换向过程。结合 `lio_new` 和 TEB 方案的真车表现，目前成功概率更高。
- `right_turn`：全前进向右后方调头，保留作实验对照。它更依赖可用空间、定位精度、转向响应和避障器的连续可行解，现阶段不作为比赛基线。
- TEB 只负责固定路径被障碍物阻挡时的局部绕行，不负责替代整段固定路线，也不跨越揉库中的硬换向点。
- TEB 必须保留倒车能力，以处理真实死路；当前通过较高的前进偏好权重抑制无必要倒车，而不是禁止倒车。

重要：截至本文档日期，launch 中 `post_qr_route_mode` 的代码默认值仍是 `right_turn`。真车测试必须显式传：

```bash
post_qr_route_mode:=parking
```

后续人员确认揉库基线无误后，应优先考虑把 launch 默认值改成 `parking`，但修改前后都必须从启动日志确认实际加载的路线。

## 2. 代码边界与版本状态

包路径：

```text
src/origincar/lio_hybird
```

注意包名沿用现状，拼写是 `hybird`，不是 `hybrid`。

- `lio_hybird` 是从 `lio_new` 独立出来的实验包。
- 不应修改或破坏 `lio_new`；它仍是独立可运行的对照基线。
- 不要修改 `origincar_track`，只能参考其视觉巡线实现。
- 当前整个 `src/origincar/lio_hybird/` 在工作区中尚未被 Git 跟踪。不要使用 Git reset/checkout 期望恢复它，也不要假设能够通过提交历史找到每一版代码。
- 修改源文件后必须在车端重新构建并重新 `source install/setup.bash`，否则很容易运行到旧的 install 内容。

建议后续负责人在确认当前版本可运行后，先建立一个明确的 Git 基线提交或外部备份，再继续改算法。

## 3. 当前系统架构

### 3.1 定位

`submap_lio_node` 输入：

```text
/imu/data          Madgwick 处理后的 IMU
/odom              底盘轮速里程计
/scan              LSN10 LaserScan
```

主要输出：

```text
/lio_hybird/odom   控制器使用的场地位姿
lio_hybird_odom -> odom TF 桥（供 Nav2/TEB 使用）
```

定位核心是二维 IMU/轮速预测、激光逐点去畸变、关键帧 NDT 子地图迭代更新，并保留初始化阶段的 scan-to-scan ICP 兜底。它不是 Nav2 AMCL，也没有依赖 Nav2 全局地图定位。

### 3.2 控制权切换

完整任务的控制权顺序如下：

```text
第一赛段
  first_line_mode=vision -> origincar_control 产生私有速度，主控制器转发
  first_line_mode=lio    -> lio_hybird 内置路径追踪/局部避障
        |
        | /sign_switch 收到有效二维码结果
        v
立即截断第一赛段命令并制动
        |
        v
选择顺/逆时针的扫码后固定路径
        |
        +-> 路径畅通：内置 Pure Pursuit 跟踪
        |
        +-> 雷达阻挡，且当前位置到第72点全是前进段：TEB 临时接管
        |       TEB结束/失败回退后再交还内置控制器
        |
        +-> 揉库前进/倒车硬换向：始终由内置控制器处理
        v
第72点（进入中间通道/环形区的交接点）之后继续固定路径追踪
```

第一赛段视觉模式并不是直接让 `origincar_control` 发布 `/cmd_vel`。它发布到私有话题 `/lio_hybird/visual_first_line_cmd`，由主控制器在自己拥有控制权时转发，从而保证扫码瞬间能够统一制动和切换。

### 3.3 Nav2/TEB 的实际范围

`teb_handoff.launch.py` 只启动：

- Nav2 `controller_server`；
- 滚动局部代价地图；
- lifecycle manager；
- `FollowPath` 动作 `/follow_path` 中的控制器 `TEB`。

没有启动 AMCL、map server、全局规划器、BT Navigator 或 Nav2 全局栅格地图。因此这不是完整 Navigation2 导航。TEB 接收的是 `lio_hybird` 截取出的剩余固定路径，只在必要时围绕它做局部优化。

由于没有 BT Navigator 恢复树，`FollowPath` 的一次 abort 不会自动进入 Nav2 的标准恢复流程。当前控制器自行完成：停车、清滚动局部代价地图、按当前位置重新截取剩余路径、再次下发。

## 4. 扫码后路径方案

路径由以下脚本生成：

```text
scripts/generate_qr_split_paths.py
```

不要直接修改 `install/` 下的路径。应修改源目录脚本/路径并重新构建。

### 4.1 揉库基线 `parking`

文件：

```text
paths/path_post_qr_parking_clockwise.txt
paths/path_post_qr_parking_counterclockwise.txt
```

统一设计接入姿态约为：

```text
(x=4.20 m, y=1.50 m, yaw=30.2 deg)
```

前72点的阶段划分：

- 点1-4：前进到约 `(4.30, 1.56)`；
- 点5-30：倒车揉库，后半段复用原 `path_competition_ideal` 的揉库几何；
- 点30：约 `(2.70, 0.85)`；
- 点31-72：恢复前进，进入中间通道；
- 点72：约 `(2.70, 2.791)`，作为 TEB 交还/通道入口目标。

当前静态几何检查结果：

- 最小曲率半径约 `0.451 m`；
- 以约 30 cm x 20 cm 车体外框检查，前72点没有进入黑色禁行区；
- 每份完整路径共200点。

这些是离线几何结果，不等于真车动态安全结论。必须继续验证制动距离、舵机回正延迟和 LIO 偏差下的车体包络。

### 4.2 实验对照 `right_turn`

文件：

```text
paths/path_post_qr_right_turn_clockwise.txt
paths/path_post_qr_right_turn_counterclockwise.txt
```

前72点全部前进，以约 `0.40 m` 半径向右后方调头，关键经过点大致为：

```text
(4.40, 0.754) -> (3.20, 0.754) -> (2.80, 1.154) -> 第72点
```

当前静态几何检查结果：

- 最小曲率半径约 `0.399 m`；
- 前72点车体外框离线检查没有进入黑色禁行区；
- 每份完整路径共200点。

该方案对场地边缘余量和转向响应更敏感，当前只用于 A/B 对照，不应在未显式指定时误当作基线。

路径预览：

```text
paths/path_qr_post_variants_preview.png
```

原始 `paths/path_competition_ideal.txt` 保留，未被上述生成过程删除或覆盖。

### 4.3 直接倒入环形区 `reverse_ring_ccw`

该方案根据后续示意图新增，只用于二维码结果为逆时针 `sign_data=4` 的任务：

```text
paths/path_post_qr_reverse_ring_counterclockwise.txt
```

运行选择：

```bash
post_qr_route_mode:=reverse_ring_ccw
```

路径阶段：

- 点1从约 `(4.20, 1.50, 30.2 deg)` 立即开始倒车；
- 倒车经过下半区和中间通道，采用两段约 `0.50 m` 半径的圆滑转弯；
- 点55到达约 `(1.75, 3.20, 0 deg)`，车辆位于环形下侧左段、车头朝右；
- 点56切换为前进，沿环形下侧向右直行；
- 点78与原逆时针环形路线的点78完全衔接，之后复用原环形和返回路线。

离线检查结果：

- 完整路径200点；
- 方向段为点1-55倒车、点56-200前进，只有一次硬换向；
- 最小曲率半径约 `0.408 m`；
- 以约 30 cm x 20 cm 车体外框检查，全部路径点没有进入黑色禁行区；
- 环形视觉的原点78索引保持不变。

该模式没有擅自生成顺时针镜像路径。如果扫码结果是顺时针 `sign_data=3`，launch 会回退到原 `parking` 顺时针路径。预览文件：

```text
paths/path_qr_reverse_ring_preview.png
```

该路线目前是待真车验证的实验方案，不改变第1节中 `parking` 作为已知基线的结论。

## 5. 当前 TEB 策略和参数

配置文件：

```text
config/teb_handoff_nav2.yaml
```

关键值：

```yaml
controller_frequency: 15.0
max_samples: 40
allow_init_with_backwards_motion: true
max_vel_x: 0.60
max_vel_x_backwards: 0.45
max_vel_theta: 2.8
acc_lim_x: 1.4
acc_lim_theta: 3.5
min_turning_radius: 0.20
no_inner_iterations: 4
no_outer_iterations: 3
weight_kinematics_forward_drive: 100.0
local_costmap.update_frequency: 8.0
local_costmap.publish_frequency: 3.0
local_costmap.resolution: 0.05
```

解释：

- `allow_init_with_backwards_motion=true` 和 `max_vel_x_backwards=0.45` 保证真实死路时仍能倒车。
- `weight_kinematics_forward_drive=100.0` 是强前进偏好，不是倒车禁令。
- 优化迭代数和控制频率已降低，目的是减少 TEB 过载造成的控制延迟和 LIO 扫描队列堆积。
- 局部代价地图用原始 `/scan` 做 clearing，用 `/lio_hybird/obstacles` 的过滤障碍点云做 marking。
- TEB 仅在“当前点到第72点没有方向切换”时允许接管。揉库倒车段不会被 TEB 跨越。

重试参数位于 `config/lio_hybird.yaml` 和 launch 参数：

```yaml
teb_handoff_timeout: 20.0
teb_retry_delay: 1.0
teb_retry_max_attempts: 0       # 0 = 不限次数
teb_retry_min_field_margin: 0.25
teb_retry_clear_costmap: true
```

无限重试只允许发生在离物理场地边缘至少 `0.25 m` 的位置。靠近边缘时停止无限重发，交回有边界约束的内置规划器，避免 TEB 在墙边反复输出危险动作。

## 6. 已知失败与根因

### 6.1 旧路线导致 TEB 主动长距离倒车

已分析的一次失败中：

```text
扫码位姿约 (3.421, 1.222, 30.2 deg)
旧绿色路径初始航向接近 180 deg
TEB 从约 (3.59, 1.36) 倒到 (3.56, 0.40)
随后 Failed to make progress 并反复重试
```

根因不是简单的“TEB 倒车开关错误”，而是当前姿态与旧路径初始方向相差约150度，且旧配置前进偏好权重仅约 `0.4`。在 TEB 的代价函数里，倒车比原地完成大角度转向更便宜，于是它选择了长距离倒车。后续出现高频超时，进一步导致 LIO 扫描队列满和 costmap 丢弃过期扫描。

当前修正包括：重新设计扫码后路径、提高前进偏好、降低 TEB 计算负担、限制路径截断搜索窗口，并禁止 TEB 跨越方向切换点。

### 6.2 TEB 接管后 RViz 车框冻结

RViz 绿色车框来自：

```text
/lio_hybird/debug_markers
```

当前 marker 在内置 Pure Pursuit 发布速度命令时刷新。TEB 接管后，Pure Pursuit 为避免争抢 `/cmd_vel` 会停止发速度，因此 marker 看起来停住；这不必然表示 `/lio_hybird/odom` 或 LIO 停止更新。

判断定位是否真的停住，应查看：

```bash
ros2 topic hz /lio_hybird/odom
ros2 topic echo /lio_hybird/odom --once
```

后续应把车辆位姿 marker 改成按 odom/固定定时器刷新，使它与控制权无关。这是明确未完成项。

### 6.3 TEB controller_server 参数类型崩溃

曾出现：

```text
parameter 'height' has invalid type: integer vs double
```

当前 `teb_handoff_nav2.yaml` 中局部地图 `width: 4`、`height: 4` 已按整数写入。若车端仍出现同一错误，首先怀疑 install 中仍是旧配置，而不是继续修改源文件。

### 6.4 编译环境限制

当前开发机缺少车端 TROS `ai_msgs` 完整环境，无法在本机完成全部 C++ 链接验证。Python 路径生成器、launch 构造、YAML 解析和路径静态检查已做过，但最终编译与运行必须在车端完成。

## 7. 车端构建与启动

### 7.1 构建

```bash
cd /userdata/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
colcon build --packages-select origincar_control lio_hybird --symlink-install \
  --allow-overriding origincar_control
source install/setup.bash
```

如果不重编 `origincar_control`，可只编：

```bash
colcon build --packages-select lio_hybird --symlink-install
source install/setup.bash
```

### 7.2 外部前置节点

运行 `lio_hybird.launch.py` 前，至少要确保以下输入存在：

```text
/scan
/imu/data
/odom
```

视觉第一赛段还需要 `origincar_control` 原有的图像/检测输入；环形视觉纠偏默认需要 Aurora 的：

```text
/aurora/rgb/image_raw
```

二维码节点默认由 `lio_hybird.launch.py` 内部启动。不要再启动第二套相同二维码节点。底盘、雷达和 Aurora 驱动仍需按车端 bringup 分别启动。

### 7.3 当前推荐比赛基线命令

```bash
ros2 launch lio_hybird lio_hybird.launch.py \
  first_line_mode:=vision \
  post_qr_route_mode:=parking \
  enable_teb_handoff:=true \
  start_teb_controller:=true \
  visual_force_left_region:=true \
  visual_force_left_before_x:=2.50 \
  visual_force_left_below_y:=0.50 \
  rviz:=true
```

不要同时运行另一套 Nav2 `controller_server`。若确实复用外部 controller server，必须确认它提供 `/follow_path`、包含 ID 为 `TEB` 的控制器、TF 使用 `lio_hybird_odom` 树，并传：

```bash
start_teb_controller:=false
```

## 8. 启动后必须核对

先看全部节点，不要只依赖 grep：

```bash
ros2 node list
ros2 action list
ros2 lifecycle get /controller_server
```

预期至少能看到：

```text
/submap_lio
/lio_hybird_controller
/lio_hybird_first_line_visual_controller    # first_line_mode=vision 时
/controller_server
/follow_path                                # action
```

`/controller_server` 应为 `active`。然后检查输入和输出频率：

```bash
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 topic hz /lio_hybird/odom
```

确认实际参数，而不是只看 YAML：

```bash
ros2 param get /lio_hybird_controller first_line_mode
ros2 param get /lio_hybird_controller enable_teb_handoff
ros2 param get /lio_hybird_controller teb_handoff_end_path_point
ros2 param get /lio_hybird_controller teb_retry_max_attempts
```

确认当前运行的是本工作区版本：

```bash
ros2 pkg prefix lio_hybird
```

结果应指向 `/userdata/dev_ws/install/lio_hybird`。若启动日志仍显示旧参数，重新构建并重新 source，不要继续在旧进程上调参。

## 9. RViz 与调试话题

主要话题：

```text
/lio_hybird/odom               LIO 位姿
/lio_hybird/tracking_path      当前选中的完整固定路径
/lio_hybird/lookahead          Pure Pursuit 前瞻点
/lio_hybird/local_plan         内置局部规划轨迹
/lio_hybird/teb_handoff_path   下发给 TEB 的剩余固定路径
/local_plan                    TEB 优化后的轨迹
/local_costmap/costmap         TEB 滚动局部代价地图
/lio_hybird/obstacles          过滤后的障碍点云
/lio_hybird/state              控制权/运行状态
/lio_hybird/debug_markers      车辆和调试 marker（TEB阶段有冻结问题）
```

推荐同时开状态监视：

```bash
ros2 topic echo /lio_hybird/state
```

揉库时应能看到类似 `TRACK_REVERSE` 的状态。发生障碍接管时，应同时观察 `/lio_hybird/teb_handoff_path` 和 `/local_plan`，不能只看绿色车框。

## 10. 真车录包模板

每次改变路径、TEB 或 LIO 参数后，至少录以下数据：

```bash
ros2 bag record -o /tmp/lio_hybird_parking_test \
  /lio_hybird/odom \
  /odom \
  /imu/data \
  /scan \
  /cmd_vel \
  /sign_switch \
  /lio_hybird/state \
  /lio_hybird/tracking_path \
  /lio_hybird/lookahead \
  /lio_hybird/local_plan \
  /lio_hybird/teb_handoff_path \
  /lio_hybird/obstacles \
  /local_plan \
  /local_costmap/costmap \
  /tf \
  /tf_static
```

图像数据量大，只有调二维码或环形视觉时再增加 Aurora/二值图话题。rosbag 没有真实世界真值，测试时仍需同步录一段能看见场地边界和车辆姿态的视频，并记录：障碍物摆放位置、车辆停止/人工搬动时刻、是否发生碰撞或接管。

## 11. 推荐调试顺序

不要一上来同时修改 LIO、路径、TEB 和速度。按以下顺序隔离问题：

1. 静止检查传感器频率、TF 和 `/lio_hybird/odom` 是否连续。
2. 无障碍、低速验证 `post_qr_route_mode:=parking` 的前进-倒车-前进换向点。
3. 无障碍逐步恢复比赛速度，测量换向制动距离和车体最小边界余量。
4. 在揉库前进/倒车段放障碍，确认内置规划器处理且 TEB 不跨越硬换向点。
5. 在揉库完成后的全前进段放单个锥桶，确认 TEB 才发生接管并在第72点前交回。
6. 重复 TEB abort 场景，确认15 Hz配置下不再持续拖慢 LIO，也不出现扫描队列长期满。
7. 最后再测试环形区、禁行区和多个障碍组合。

每次只改一组参数，并保存命令、日志、rosbag 和障碍摆位照片。没有这些记录的“体感变好/变差”很难区分路径、定位和局部规划的问题。

## 12. 后续开发优先级

### P0：比赛基线

1. 将 launch 默认路线从 `right_turn` 改成 `parking`，并在启动日志明确打印所选文件、方向段和第72点坐标。
2. 真车验证新揉库路径，重点确认点4-5和点30-31的制动、换向和前瞻点搜索。
3. 确认扫码时车辆位姿在路径设计接入姿态附近；偏差过大时应明确停车或有限连接策略，不能静默跳到空间最近点。

### P1：可观测性和实时性

1. 修复 TEB 接管时 `/lio_hybird/debug_markers` 车辆框冻结，改为由 odom 或固定频率刷新。
2. 记录并统计 controller loop 超时、TEB单周期耗时、LIO扫描排队/丢弃次数。
3. 验证 TEB `15 Hz`、costmap `8 Hz` 后是否仍会造成 LIO 队列堆积。

### P2：安全脱困

1. 真车验证距离场地边缘不足 `0.25 m` 时，TEB无限重试退出后内置规划是否真的能安全脱困。
2. 区分“TEB无解”“TF/数据过期”“进度检查超时”，不要把三种失败统一当作障碍无解。
3. 在真实死路验证 TEB仍能有限倒车，确认高前进权重没有实际封死倒车解。

### P3：方案对照

只有 `parking` 基线稳定并有完整数据后，再用相同障碍摆位和速度测试 `right_turn`。评价指标应至少包括：总用时、最小边界余量、接管次数、倒车距离、TEB失败次数和LIO丢扫描数。

## 13. 修改时不要破坏的约束

- 不删除 `path_competition_ideal.txt` 和两套扫码后路径。
- 不把 TEB 改成完全禁止倒车。
- 不让 TEB 跨越前进/倒车硬换向点。
- 不让 TEB 在固定路径畅通时接管正常巡线。
- 不修改 `origincar_track`。
- 不因修改 `lio_hybird` 使 `lio_new` 失效。
- 不直接编辑 `install/` 作为正式修改来源。
- 不根据 RViz marker 冻结直接判断 LIO 停止，必须检查 odom 话题。

这份文档描述的是 2026-08-06 的阶段状态。后续改变控制权边界、路径点定义、默认路线、TEB接管条件或重试机制时，必须同步更新本文档。
