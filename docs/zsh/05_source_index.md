# 实现状态与源码路径索引

[返回总说明](../../README.md)

本次提交为 README 和技术文档整理。下面明确区分本分支已有源码、独立小车试运行源码，以及需要实车移植的算法模块；未新增仿真文件，也没有通过文档凭空增加节点。

## 1. 本分支已有代码

| 实现环节 | 实际路径 | 阅读重点 |
| --- | --- | --- |
| 底盘启动 | [origincar_bringup.launch.py](../../src/base_orin/launch/origincar_bringup.launch.py) | 串口、IMU、EKF、TF 的组合 |
| 底盘控制模式 | [base_serial.launch.py](../../src/base_orin/launch/base_serial.launch.py) | `akmcar`、设备、波特率与命令路径 |
| 串口协议与反馈 | [base_orin.cpp](../../src/base_orin/src/base_orin.cpp) | Twist/Ackermann 回调、单位缩放、里程计与 IMU |
| Ackermann 转换 | [cmd_vel_to_ackermann_drive.py](../../src/base_orin/scripts/cmd_vel_to_ackermann_drive.py) | 轴距和角速度到转角的换算 |
| 既有轮速/IMU 融合 | [ekf.yaml](../../src/base_orin/config/ekf.yaml)、[imu.yaml](../../src/base_orin/config/imu.yaml) | TF 所有权、二维融合、姿态来源 |
| 二维 NDT-EKF 定位 | [submap_lio_node.cpp](../../src/origincar/lio_hybird/src/submap_lio_node.cpp) | 五状态预测、去畸变、`rebuild_ndt_submap`、`align_scan_to_submap` |
| 实车跟踪和避障基线 | [pure_pursuit_node.cpp](../../src/origincar/lio_hybird/src/pure_pursuit_node.cpp) | Pure Pursuit、候选路径、障碍记忆、脱困、任务交接 |
| 定位/控制参数 | [lio_hybird.yaml](../../src/origincar/lio_hybird/config/lio_hybird.yaml) | 时间、外参、NDT、控制和避障参数 |
| 包编译与安装入口 | [CMakeLists.txt](../../src/origincar/lio_hybird/CMakeLists.txt) | 实际构建的是哪几个节点 |
| 基线启动与路线选择 | [lio_hybird.launch.py](../../src/origincar/lio_hybird/launch/lio_hybird.launch.py) | 二维码前控制模式、路线分支、停车模式 |
| 顺时针停车路线 | [path_post_qr_parking_clockwise.txt](../../src/origincar/lio_hybird/paths/path_post_qr_parking_clockwise.txt) | 路线坐标与停车段 |
| 逆时针停车路线 | [path_post_qr_parking_counterclockwise.txt](../../src/origincar/lio_hybird/paths/path_post_qr_parking_counterclockwise.txt) | 对向路线和任务方向 |
| 视觉前段控制 | [origincar_control 包](../../src/origincar/origincar_control) | 键盘启动、识码后停止、控制权释放 |
| 二维码识别 | [qr_detection_opencv.py](../../src/origincar/qr_code_detection/qr_code_detection/qr_detection_opencv.py) | 结果编码、发布与重复事件 |
| 自定义识码消息 | [Sign.msg](../../src/origincar/origincar_msg/msg/Sign.msg) | `int32 sign_data` |
| 自定义速度消息 | [Data.msg](../../src/origincar/origincar_msg/msg/Data.msg) | x/y/z 的业务含义不能等同标准 Odometry |
| 拍照请求与服务连接 | [ring_photo_client_node.py](../../src/origincar/qr_screen_display/qr_screen_display/ring_photo_client_node.py) | 相机、工作线程、ZMQ、超时与重试 |
| 显示 | [qr_screen_display_node.py](../../src/origincar/qr_screen_display/qr_screen_display/qr_screen_display_node.py) | 显示状态和任务结果 |

包名实际拼写为 `lio_hybird`。命令、路径和依赖必须保持此拼写，不能将其自动改为 `lio_hybrid`。

## 2. 独立 NdtLIO 试运行的路径

以下为之前小车包内的逻辑相对路径，**不是本分支中可点击打开的文件**。小车是否仍保有同版源码和安装产物，需要现场核对：

```text
src/zsh_pkg/
  CMakeLists.txt
  origincar_trial/
    src/NdtLIO/
      ros2node/ndt_lio_node.cpp
      # 核心中包括 frontend、map、ndt_inc、eskf、ieskf 等实现
    config/mapping_trial_lo.yaml
    launch/ndt_lio_node.launch.py
    scripts/run.sh
    scripts/save_map.sh
```

关键审查点：父包实际安装 `test_ros2_lio` 和 `process_imu`；默认 `with_imu=0`；地图分辨率配置以像素/米计；定位分支输出和滤波器访问存在模式适用边界；保存占据地图不能代替保存 NDT 地图。详见 [定位文档](01_ndtlio_mapping_localization.md)。

## 3. 新规划与 MPC 的模块对应关系

下表给出被整理方法的模块职责及实车接入位置。模块名用于识别算法来源和组织迁移，**这些文件没有随本次文档添加到本分支，也没有对应新 `ros2 run` 入口**。不应把下表当作现有目录清单。

| 算法模块名 | 完整职责 | 实车需要接入的内容 |
| --- | --- | --- |
| `pose_source.py` | 选择定位输入、时间插值、场地对齐、传感器偏移 | NdtLIO/二维 NDT 输出适配、质量门控、真实 TF |
| `race_course.py` | 赛段、路线、方向、边界和任务语义 | 本车场地坐标、实际赛规和停车目标 |
| `obstacle_cloud.py` | 扫描累积、聚类、锥桶轮廓、保守回退 | 雷达时序、锥桶尺寸、定位误差与真实视野 |
| `frenet_corridor.py` | 多障碍侧向偏移、航向连续、汇合验证 | 车体/后轴参考点、场地边界、侧别保持 |
| `hybrid_corridor.py` | 航向格点、车辆原语、解析连接、倒车末段 | 轴距、转角上限、全车碰撞与计算预算 |
| `race_mpc.py` | 转向动态预测、CEM 优化、速度与碰撞约束 | 执行器辨识、实时预算、底盘单位与饱和 |
| `race_navigation.py` | 后台规划、预规划、版本和结果有效性管理 | 独立进程生命周期、地图/定位重置、任务事件 |
| `full_race_navigation.py` | ROS 主接入与任务执行编排 | 实际传感器、命令唯一出口、超时状态、安装入口 |

移植时先抽离依赖环境的适配代码，保留可独立回放的算法核心；不能仅改话题名称就当作实车移植完成。具体顺序见 [迁移验收](04_validation_migration.md)。

## 4. 阅读顺序与交付边界

先读根 README 了解完整链路，再按 [定位](01_ndtlio_mapping_localization.md) → [规划控制](02_mission_planning_mpc.md) → [接口](03_interfaces_bringup.md) → [验收](04_validation_migration.md) 阅读。遇到“当前实现”和“建议实现”冲突，以实际源码、安装入口和已验证运行记录为准。

本次提交仅增加说明文档：没有复制仿真、没有覆盖底盘或定位源码、没有清理构建产物。后续若交付完整 MPC 实车包，应另行提交对应源码、安装规则、标定参数和真实验收记录，并更新这里的状态表。
