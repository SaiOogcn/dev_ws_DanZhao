# ROS 2 基础观看路线

课程入口：<https://www.bilibili.com/video/BV1GW42197Ck/>

这份路线给完全没接触过 ROS 2 的新队员使用，适用于 YOLO 视觉、Nav2 调参和 LIO 三个方向。目标是建立 ROS 2 的共同基础：知道节点、话题、参数、Launch、TF 和传感器数据各自负责什么，能在真车上观察运行状态，能判断问题属于哪个模块，具体代码实现可以交给 AI agent。（其实这个文档里的所有工作都可以交给Codex去完成，但是我不建议你去这么做，因为你不会学到任何东西）

## 使用方法

1. 按顺序观看。每个阶段先看“必看”，再拿真车做“阶段检查”。
2. 遇到代码时，先看输入、输出和模块职责，不要求第一遍完整复现。
3. 视频中的安装命令、文件路径和参数可能随 ROS 2 版本变化。以我们项目的环境和文档为准。
4. 第 2 章的 Python 和 C++ 示例选一种看即可。零基础队员推荐先看 Python。
5. 有真车可用时，优先看命令行、RViz、TF、传感器和第 9 章真机内容。Gazebo 仿真只在无法上车或需要补概念时观看。
6. 方向 2 的同学完成本路线后，继续看[方向 2：Nav2 局部规划器调参与优化指南](06_方向2_Nav2_TEB真车调参与优化指南.md)。方向 1 和方向 3 的同学先完成本路线，后续按各自专题文档学习。

## 观看总览

| 阶段 | 课程范围 | 目的 | 建议投入 |
| --- | --- | --- | --- |
| 1 | 第 1 章 | 能使用 Linux，知道 ROS 2 是什么 | 半天到 1 天 |
| 2 | 第 2～4 章 | 理解节点、话题、服务、参数和 Launch | 2～3 天 |
| 3 | 第 5～6 章 | 理解 TF、RViz 和传感器数据，能在真车上观察它们 | 2～3 天 |
| 4 | 第 7 章 | 知道导航、全局路径和局部规划器分别负责什么 | 1～2 天 |
| 5 | 第 9 章 | 理解真车的底盘、里程计、传感器和 TF | 1～2 天 |

## 阶段 1：先让 ROS 2 跑起来

### 必看

- [1.1 ROS 部落的自我介绍](https://www.bilibili.com/video/BV1tE4m1d7Ug/)
  - 目的：知道 ROS 2 解决什么问题，理解“多个节点协作”的基本思路。
- [1.2.4 在 Ubuntu 中安装 ROS 2](https://www.bilibili.com/video/BV1Jz421B7Ey/)
  - 目的：认识 ROS 2 的安装、环境加载和发行版概念。安装时以队内统一环境为准。
- [1.3 运行你的第一个机器人](https://www.bilibili.com/video/BV1iE421w7d5/)
  - 目的：先看到一个机器人运行起来，建立“节点发布数据、机器人产生动作”的直观认识。
- [1.4.1 Linux 终端基础操作](https://www.bilibili.com/video/BV1NE421w72J/)
  - 目的：会打开终端、切换目录、查看文件和运行命令。课程合集页面可按标题搜索。
- [1.4.5 Linux 基础之环境变量](https://www.bilibili.com/video/BV1pm42137bR/)
  - 目的：理解为什么新终端需要 `source` ROS 2 环境，避免后面把环境问题误判成代码问题。

### 可以跳过

- `1.2.3.1` WSL 安装：只有 Windows 用户需要看。
- `1.4.3` Python 和 `1.4.4` C++：已有编程基础时快速浏览即可。
- 具体安装故障排查：先使用队内提供的环境镜像或安装说明。

### 阶段检查

队员应能完成下面的操作，并用自己的话解释结果：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /rosout
```

检查重点：知道 `node` 是运行中的功能单元，知道 `topic` 是节点之间传递数据的通道。

## 阶段 2：理解 ROS 2 的通信和启动方式

### 必看

- [2.1.1 编写你的第一个节点（Python 示例）](https://www.bilibili.com/video/BV1iy411i727/)
  - 目的：认识节点的输入、输出、定时器和日志。代码只需看懂结构。
- [2.2 使用功能包组织 Python 节点](https://www.bilibili.com/video/BV1ky411e7hp/)
  - 目的：理解功能包、工作空间、构建和安装后的目录关系。
- [3.1 话题通信介绍](https://www.bilibili.com/video/BV1D3WXeSELP/)
  - 目的：理解发布者、订阅者、消息类型和话题名称。
- [3.3.1 发布速度控制海龟画圆](https://www.bilibili.com/video/BV1pas7eGEkk/)
  - 目的：把“发布速度指令”与“机器人运动”联系起来。
- [3.3.2 订阅 pose 实现闭环控制](https://www.bilibili.com/video/BV1mLsseKEiN/)
  - 目的：理解控制系统为什么要同时读取状态和发送指令。
- [4.1.1 服务通信介绍](https://www.bilibili.com/video/BV1kAHxepEcf/)
  - 目的：区分话题和服务。服务适合一次请求一次响应的操作。
- [4.4.1 参数声明设置](https://www.bilibili.com/video/BV1G54neuEiH/)
  - 目的：理解参数如何改变节点行为，建立阅读 Nav2 YAML 配置的基础。
- [4.6.1 使用 Launch 启动多个节点](https://www.bilibili.com/video/BV1hdtLeZE9y/)
  - 目的：理解一个 Launch 如何同时启动机器人、传感器和导航相关节点。

### 选看

- [2.1.2 编写你的第一个节点（C++ 示例）](https://www.bilibili.com/video/BV1FH4y1c7Px/)
  - 已经需要维护 C++ 节点时再看。
- [3.4.2 自定义通信接口](https://www.bilibili.com/video/BV1dTHgeeEPm/)
  - 需要阅读项目自定义消息或服务时再看。
- `3.2` 小说示例、`4.2` 人脸检测示例：关注通信形式即可，不要陷入业务代码。

## 阶段 3：理解 TF、RViz 和传感器数据

### 必看

- [5.1.1 通过命令行使用 TF](https://www.bilibili.com/video/BV1ietCe2E9F/)
  - 目的：开始理解坐标系和 TF 查询。后面看到 `map`、`odom`、`base_link` 时不会只记名称。
- [5.2.2 通过 Python 发布动态 TF](https://www.bilibili.com/video/BV1mxsheXEpc/)
  - 目的：理解机器人运动时坐标关系如何更新。
- [6.2.2 在 RViz 中显示机器人](https://www.bilibili.com/video/BV1Vq2LYoECn/)
  - 目的：会用 RViz 观察模型、TF 和传感器数据。

### 真车优先补看

- [9.1 移动机器人系统设计](https://www.bilibili.com/video/BV1NyFNeEEAh/)
  - 目的：从系统角度看主控板、底盘、传感器和 ROS 2 主机的分工。
- [9.3.5 机器人里程计计算](https://www.bilibili.com/video/BV1BJA5eTE9t/)
  - 目的：理解里程计怎样产生，以及它为什么影响定位和导航。
- [9.5.1 驱动并显示雷达点云](https://www.bilibili.com/video/BV1JnPeewEQ5/)
  - 目的：理解真机传感器怎样进入 ROS 2。
- [9.5.4 发布里程计 TF](https://www.bilibili.com/video/BV1Uq93YRE5T/)
  - 目的：理解真车导航需要哪些 TF。

### 选看

- [6.1 机器人建模与仿真概述](https://www.bilibili.com/video/BV1CB19Y1Ew8/)
  - 仿真或模型配置遇到问题时再看。
- [6.4.6 惯性测量传感器仿真](https://www.bilibili.com/video/BV1qLCRYYEMa/)
  - 需要了解 IMU 数据来源时再看。
- [6.4.5 激光雷达传感器仿真](https://www.bilibili.com/video/BV1vTCUYyEcX/?share_source=copy_web&vd_source=150d9a0863326af2040a993f956da587)
  - 需要补传感器仿真概念时再看。
- [6.5.5 使用两轮差速控制器控制机器人](https://www.bilibili.com/video/BV13SyDYmEPx/)
  - 只看速度指令到控制器的关系。项目使用阿克曼底盘，不能直接照搬两轮差速参数。
- `6.2` 和 `6.5` 中的 XML、插件和控制器代码：第一遍只看输入、输出和启动关系。

### 阶段检查

队员应能在 RViz 中指出：

- 机器人当前的 `base_link`；
- 里程计对应的 `odom`；
- 地图或定位对应的 `map`；
- 激光雷达数据和机器人速度指令。

## 阶段 4：建立导航框架的基本认识

第 7 章只要求理解模块职责。课程使用的机器人和项目底盘可能不同，视频中的地图、话题名和参数不能直接照搬。

### 必看顺序

1. [7.1 机器人导航介绍](https://www.bilibili.com/video/BV1gf1KYoEMf/)
   - 目的：区分建图、定位、全局路径和局部规划。
2. [7.3.1 Navigation2 介绍与安装](https://www.bilibili.com/video/BV1ezSiYdEJG/)
   - 目的：认识 Nav2 的定位、地图、代价地图、规划器、控制器和行为模块。
3. [7.3.2 编写 Launch 并启动导航](https://www.bilibili.com/video/BV1kiSHYaEh8/)
   - 目的：理解导航节点怎样由 Launch 组合起来。
4. [7.3.4 进行单点与路点导航](https://www.bilibili.com/video/BV1bPSoY3E9D/)
   - 目的：观察导航任务怎样接收目标并执行。
5. [7.4.1 使用话题初始化机器人位姿](https://www.bilibili.com/video/BV11bDDYXEsC/)
   - 目的：理解初始位姿为什么会影响导航。
6. [7.4.2 使用 TF 获取机器人实时位置](https://www.bilibili.com/video/BV1i5DaYnE31/)
   - 目的：理解定位和导航怎样获得机器人位姿。
7. [7.5.1 完成机器人系统架构设计](https://www.bilibili.com/video/BV1eZm1YmEhm/)
   - 目的：把节点、传感器、导航和应用逻辑放回整体架构。

### 选看

- [7.2.1 构建第一张导航地图](https://www.bilibili.com/video/BV1Ff1jYqEyU/) 和 [7.2.2 将地图保存为文件](https://www.bilibili.com/video/BV1kB1JYmEvR/)：项目使用地图或需要理解地图文件时再看。
- [9.0.7 FishBot-Nav2 导航测试](https://www.bilibili.com/video/BV1ePrFYvEvQ/)：想看完整导航示例时再看。
- [7.5.2 编写巡检控制节点](https://www.bilibili.com/video/BV17WmhYyEZi/)：需要了解应用层如何连续调用导航任务时再看。

### 阶段检查

队员应能解释下面这条流程，并在项目中找到对应节点或话题：

```text
传感器和里程计
        ↓
TF、定位和地图
        ↓
全局路径
        ↓
局部规划器
        ↓
底盘控制接口
        ↓
机器人运动
```

## 阶段 5：上车实践顺序

下面的顺序适合第一次接触真车。每完成一步，再进入下一步。找不到数据时，先记录命令输出和报错，不要直接改参数。

| 上车步骤 | 先看什么 | 要完成的事情 |
| --- | --- | --- |
| 1. 启动环境 | 1.4.1、1.4.5 | 能打开终端，正确加载 ROS 2 环境 |
| 2. 查看系统 | 3.1、4.6.1 | 列出节点、话题和启动文件，知道每个模块是否运行 |
| 3. 查看坐标和传感器 | 5.1.1、6.2.2、9.5.1、9.5.4 | 在 RViz 中找到 TF、里程计和传感器数据 |
| 4. 理解底盘反馈 | 9.1、9.3.5 | 找到底盘控制接口和里程计来源 |
| 5. 进入具体方向 | 方向 1、方向 2 或方向 3 的专题文档 | 在已有系统上做小范围实验 |

## 第一轮学习的验收标准

队员完成第一轮后，不要求独立写出 ROS 2 或 Nav2 代码，但应能完成下面的任务：

1. 画出我们项目的节点、话题、TF 和 Launch 关系。
2. 在 RViz 中找到机器人位姿、传感器数据和当前路径。
3. 根据报错或现象，初步判断问题属于节点、话题、参数、TF、传感器还是底盘。
4. 阅读 AI agent 生成的修改，指出它改的是哪个模块、输入输出是什么、可能影响流程中的哪一段。

## 与三个方向的关系

| 方向 | 本文需要看到的深度 | 本文之后的重点 |
| --- | --- | --- |
| YOLO 视觉检测与 PID 巡线避障 | 完成阶段 1～3，阶段 4 看 7.1 和 7.3.1 | 视觉话题、图像处理、PID 和控制输出 |
| Nav2 局部规划器调参与优化 | 完成全部阶段，再看方向 2 专题文档 | 全局路径、TEB、阿克曼底盘和真车调参 |
| LIO 编写 | 完成阶段 1～3，重点掌握 TF、传感器和里程计 | 点云、IMU、坐标变换和状态估计 |

## 建议补充

课程中的 Gazebo、URDF 建模和插件实现属于按需内容。需要仿真、模型或插件开发时，再按项目任务补看。Nav2 的生命周期节点、行为树、Costmap layer 和 QoS，可在进入方向 2 后按项目需要补看 [Nav2 官方文档](https://docs.nav2.org/)。

配套代码可参考：[fishros/ros2bookcode](https://github.com/fishros/ros2bookcode)。其中 `chapt7`、`chapt8` 和 `chapt9` 分别对应导航、导航扩展和真机实战。
