# 睿行小车新人培训

这套资料面向第一次接触 Linux、ROS 2 和移动机器人的队员。读完后，你应能看懂小车的数据流，独立启动一个练习工作空间，录一段 rosbag，并用 RViz 2 和命令行定位问题。

三个 demo 都能完成“沿路径导航并绕开障碍物”。它们关注的技术边界不同：

| demo | 核心输入 | 核心输出 | 学习重点 |
| --- | --- | --- | --- |
| 1. 视觉巡线 | `/win_detection` 的 YOLO 框 | `/cmd_vel` | 检测框解析、状态机、PID 和视觉避障 |
| 2. Nav2 + TEB | 录制路径、TF、`/scan` | Nav2 控制器输出 | TF 树、全局路径选择、局部代价地图、TEB 调参 |
| 3. LIO | `/scan`、`/imu/data`、`/odom` | `/demo3_lio/odom` | 时间同步、去畸变、外参、NDT/ICP、异常点处理 |

## 建议顺序

先读 [技术栈地图](architecture.md)，再完成 [环境准备](basics/environment.md) 和 [ROS 2 基础](basics/ros2-basics.md)。之后按 demo 1、demo 2、demo 3 的顺序学习。每个 demo 都先用录包或仿真输入跑通，再上车。

## 目录

- [技术栈地图](architecture.md)
- [环境准备：Ubuntu、ROS 2、VS Code 和 Codex](basics/environment.md)
- [ROS 2 基础与常用命令](basics/ros2-basics.md)
- [录包、回放和调试](basics/bag-debug.md)
- [调查依据与版本说明](research-notes.md)
- [Demo 1：纯视觉巡线与避障](demos/demo1-visual.md)
- [Demo 2：Nav2 + TEB 路径导航](demos/demo2-nav2-teb.md)
- [Demo 3：二维 LIO 编写与使用](demos/demo3-lio.md)

## 代码骨架

- `demos/demo1_visual_ws`：C++ 视觉控制器，TODO 位于回调和控制量生成处。
- `demos/demo2_nav2_teb_ws`：Python 路径发布器、Nav2/TEB 参数和启动骨架。
- `demos/demo3_lio_ws`：C++ LIO 节点，TODO 位于去畸变、假点过滤、预测和配准处。

![扫码后路径预览](assets/path_qr_post_variants_preview.png)

上图来自仓库中的路径预览。它用于讲解“录制路径是几何输入，不等于已经解决了定位、跟踪和避障”。

骨架故意没有给出完整比赛实现。队员需要补齐 TODO，并提交参数记录、rosbag 和复盘报告。

## 参考代码

- 本车底盘：`remote_dev_ws/src/origincar/origincar_base`
- 视觉控制：`remote_dev_ws/src/origincar/origincar_control`
- 当前二维 LIO：`remote_dev_ws/src/origincar/lio_hybird`、`remote_dev_ws/src/origincar/lio_ob`
- 录制路径和控制交接说明：`remote_dev_ws/src/origincar/lio_hybird/STAGE_HANDOFF_20260806.md`
- NDT-LIO 参考实现：[ronjinliang/NdtLIO](https://github.com/ronjinliang/NdtLIO)

> 车端是 TROS/ROS 2 Humble 环境。培训电脑可以先完成静态阅读、节点编译和录包回放；涉及 `ai_msgs`、底盘串口和雷达驱动的步骤，必须在车端验证。
