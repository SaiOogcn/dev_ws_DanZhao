# **subscribe_node**


**作用**：订阅者，可订阅deptrum-ros-driver功能包的发布节点，通过订阅发布的话题，将图像帧数据流转为图像保存，更直观地浏览 Rgb、Ir、Depth等图片效果,和点云信息。

**编译**：deptrum-ros-driver编译时会自动编译此程序

(**说明**：`sub_node`供客户和测试人员使用【交互式】。`sub_node_ci`仅供CI单元测试内部使用【传参式】)

## **run sub_node**

1. 对于`sub_node`节点直接运行

   `sub_node`节点存在位置：`/dev_ws/install/deptrum-ros-driver-设备名/lib/deptrum-ros-driver-设备名`

   运行：`./sub_node`

2. 对于`sub_node_ci`节点使用launch运行

   sub_node_ci launch文件存在位置：各设备的launch目录

   运行：`ros2 launch deptrum-ros-driver-设备名 sub_node_ci_设备名_launch.py`

## 具体设备说明

#### 1、stellar400 / stellar420

**RUN**
对于 `sub_node` :
`cd /dev_ws/install/deptrum-ros-driver-stellar400/lib/deptrum-ros-driver-stellar400`
`./sub_node`

对于 `sub_node_ci` :
`ros2 launch deptrum-ros-driver-stellar400 sub_node_ci_stellar400_launch.py`

**图像保存位置**：`/tmp/image`

**sub_node_ci支持参数设置**：`enable_rgb、enable_ir、enable_depth、enable_pointcloud`（默认均为true）



#### 2、aurora930

**RUN**
对于 `sub_node` :
`cd /dev_ws/install/deptrum-ros-driver-aurora930/lib/deptrum-ros-driver-aurora930`
`./sub_node`

对于 `sub_node_ci` :
`ros2 launch deptrum-ros-driver-aurora930 sub_node_ci_aurora930_launch.py`

**图像保存位置**：`/tmp/image`

**sub_node_ci支持参数设置**：`enable_rgb、enable_ir、enable_depth、enable_pointcloud`（默认均为true）



#### 3、nebula

**RUN**
对于 `sub_node` :
`cd /dev_ws/install/deptrum-ros-driver-nebula/lib/deptrum-ros-driver-nebula`
`./sub_node`

对于 `sub_node_ci` :
`ros2 launch deptrum-ros-driver-nebula sub_node_ci_nebula_launch.py`

**图像保存位置**：`/tmp/image`

**sub_node_ci支持参数设置**：`enable_rgb、enable_ir、enable_depth、enable_pointcloud、enable_stof_rgb、enable_flag、enable_stof_depth、enable_stof_pointcloud`（默认均为true）
