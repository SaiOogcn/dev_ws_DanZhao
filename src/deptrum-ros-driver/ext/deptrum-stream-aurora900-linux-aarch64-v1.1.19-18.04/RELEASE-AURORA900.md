# Aurora 900 Release 1.1.19:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. rgb 畸变矫正支持，使能畸变矫正后，rgb内参获取为畸变参数为0。

# Aurora 900 Release 1.1.18:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 大分辨率rgb切换支持。
2. 支持laser/led开关灯。
3. 完善深度效果。

# Aurora 900 Release 1.1.17:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复人脸roi区域重建问题。

# Aurora 900 Release 1.1.16:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. Aurora990支持。

# Aurora 900 Release 1.1.13:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 添加设置/获取激光器电流接口
2. 添加从jpg文件中特征值比对接口。
3. 修改设置从抓拍回调中获取特征值接口名称。

# Aurora 900 Release 1.1.12:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复扩展信息memcpy导致的崩溃问题。

# Aurora 900 Release 1.1.11:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 删除模型，通过脚本下载模型
2. 更新dim-face模型版本
3. 更新dim-face-sdk版本
4. 更新camera-sdk版本
5. 修改算法的枚举值
6. aurora 936, pid 0x1936支持。

## Submodule Versions
1. dim-face-model v2.1.0
2. dim-face-sdk v2.1.1
3. camera sdk version 2.10.93

# Aurora 900 Release 1.1.10:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 热插拔后出流线程不会退出
2. 修复热插拔崩溃问题
3. 910添加linux和windows的静态编译版本
4. 910适配TCF项目
5. 更新camera-sdk版本
## Submodule Versions
1. camera-sdk v2.12.11_tcf261

# Aurora 900 Release 1.1.9:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 添加设置图像分辨率接口。
2. 文档更新。
3. 910添加抓拍深度开关
4. 910热插拔无效问题修复
5. 文档更新

## Submodule Versions

# Aurora 900 Release 1.1.8:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 添加设置扫码模式接口。
2. 文档更新。
3. 910添加算法pipeline流程
4. 文档更新

## Submodule Versions


# Aurora 900 Release 1.1.7:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 心跳参数检查，需要至少连续3次失败才进行重启。
2. 文档更新。

## Submodule Versions
1. camera sdk version 2.10.91
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.6:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 心跳逻辑增加通讯失败即回调心跳失败逻辑，但不进行重启，功能可选择开启。

## Submodule Versions
1. camera sdk version 2.10.81
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.5:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 加速非windows平台热插拔功能。
2. 完善usb读取逻辑。

## Submodule Versions
1. camera sdk version 2.10.81
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.4:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 枚举部分优化，去除序列号获取不到的设备。

## Submodule Versions
1. camera sdk version 2.10.72
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.3:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 心跳功能接入。

## Submodule Versions
1. camera sdk version 2.10.70
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.1:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 兼容近距离场景，开放depth range 设置功能。
2. 修复aurora900 点云计算错误问题。

## Submodule Versions
1. camera sdk version 2.10.65
2. light-magic sdk version v3.7.48
2. depth-magic sdk version v3.7.48

# Aurora 900 Release 1.1.0:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复RegisterFrameCb 释放时调用DestroyStream会卡死

## Submodule Versions
1. camera sdk version 2.10.64
2. light-magic sdk version v3.7.8

# Aurora 900 Release 1.0.63:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 近距离版本优化。
2. 开放Depth Range设置功能。

## Submodule Versions
1. camera sdk version 2.10.40
2. light-magic sdk version v3.7.46
2. depth-magic sdk version v3.7.44

# Aurora 900 Release 1.0.60:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. aurora912 出流问题修复。
2. 热插拔问题修复。

## Submodule Versions
1. camera sdk version 2.10.27
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.54:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 新增rgbd ir点云输出。

## Submodule Versions
1. camera sdk version 2.10.27
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.53:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修改rgb和ir在不同分辨率下设置对齐接口后深度没有除以16的问题

## Submodule Versions
1. camera sdk version 2.10.27
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.52:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. sdk默认边缘裁边设置为15像素

## Submodule Versions
1. camera sdk version 2.10.27
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.51:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 解决arm平台下setmode失败问题，没有正确返回返回值

## Submodule Versions
1. camera sdk version 2.10.27
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.50:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 深度裁边放置上位机操作
2. 新增aurora900gtest aurora930gtest
## Submodule Versions

1. camera sdk version 2.10.22
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.49:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复aurora900 sample rgb图像异常问题和900 开rgbir流 getframe报错问题
## Submodule Versions

1. camera sdk version 2.10.22
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.48:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 解决930的内存泄漏，sample的点云显示函数依存内存泄漏
2. 解决内部代码GetInternalVersionInfo函数奔溃问题
## Submodule Versions

1. camera sdk version 2.10.21
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.47:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 支持aurora932

## Submodule Versions

1. camera sdk version 2.10.19
2. light-magic sdk version v3.6.45

# Aurora 900 Release 1.0.46:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 代码规范修改供scope合并代码，文档更新

## Submodule Versions

1. camera sdk version 2.10.15
2. light-magic sdk version v3.6.34

# Aurora 900 Release 1.0.45:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. depturm-scope解决热插拔无法开流问题

## Submodule Versions

1. camera sdk version 2.10.15
2. light-magic sdk version v3.6.34

# Aurora 900 Release 1.0.44:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 解决900深出图为黑问题
2. 解決单开depth or rgbd流无法出流问题
3. 解决sample win编译不过问题

## Submodule Versions

1. camera sdk version 2.10.14
2. light-magic sdk version v3.6.34

# Aurora 900 Release 1.0.43:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. aurora910新增点云，修复开双流帧率错误问题

## Submodule Versions

1. camera sdk version 2.10.14
2. light-magic sdk version v3.6.34

# Aurora 900 Release 1.0.42:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 适配931版本，去除rgb显示
2. 修改light-magic版本至3.6.34
3. 解决深度校准后深度值不对问题
4. 解决931无法出点云问题

## Submodule Versions

1. camera sdk version 2.10.14
2. light-magic sdk version v3.6.29

# Aurora 900 Release 1.0.41:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 解决交叉编译平台sample缺少viewer3d库问题

## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25

# Aurora 900 Release 1.0.40:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 解决非对齐模式下深度值不对问题

## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25

# Aurora 900 Release 1.0.39:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 增加调节电流接口SetLaserDriver
## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25

# Aurora 900 Release 1.0.38:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 增加接口FilterOutRangeDepthMap，默认150-4000
2. 910忽略错误码0x3000d
## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25

# Aurora 900 Release 1.0.37:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 支持910设备rgbdir出图
## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25
# Aurora 900 Release 1.0.36:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 调用算法filteroutrangedepthmap接口范围修改为150-3000
## Submodule Versions

1. camera sdk version 2.10.10
2. light-magic sdk version v3.5.25
# Aurora 900 Release 1.0.35:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 适配deptrum 符号导出更新
2. 深度对齐新增调用算法filteroutrangedepthmap接口

## Submodule Versions

1. camera sdk version 2.10.2
2. light-magic sdk version v3.5.25

# Aurora 900 Release 1.0.34:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修正深度除16位depth_scale入参lightmagic值错误问题
2. 客户设置rgb,ir不同分辨率不支持对齐
3. 解决切换分辨率奔溃问题

## Submodule Versions

1. camera sdk version 2.10.1
2. light-magic sdk version v3.5.22

# Aurora 900 Release 1.0.33:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修改camerasdk版本和lightmagic版本

## Submodule Versions

1. camera sdk version 2.10.1
2. light-magic sdk version v3.5.19


# Aurora 900 Release 1.0.32:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 适配新版本Camera sdk的接口定义及命名修改。

## Submodule Versions

1. camera sdk version 2.10.0
2. light-magic sdk version v3.5.16


# Aurora 900 Release 1.0.31:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 新增接口DepthCorrection，不调用默认开启
2. 删除接口SetRemoveFilterSize

## Submodule Versions

1. camera sdk version v2.9.51
2. light-magic sdk version v3.5.16



# Aurora 900 Release 1.0.30:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 640大小15帧深度算法下移
2. 增加接口SetRemoveFilterSize,范围为[30,400]，暂供调试使用

## Submodule Versions

1. camera sdk version v2.9.51
2. light-magic sdk version v3.5.16


# Aurora 900 Release 1.0.29:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 930深度图输出除以16

## Submodule Versions

1. camera sdk version v2.9.50
2. light-magic sdk version v3.5.14


# Aurora 900 Release 1.0.28:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复camerasdk上传depth时间戳为0的问题
2. 时间戳赋值为包的timestamp，流的timestamp代表上电后流逝时间
3. 适配camerasdk命名规范调整

## Submodule Versions

1. camera sdk version v2.9.50
2. light-magic sdk version v3.4.14

# Aurora 900 Release 1.0.27:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 商汤pid修改为910->912

## Submodule Versions

1. camera sdk version v2.9.48
2. light-magic sdk version v3.4.14



# Aurora 900 Release 1.0.26:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修改点云不能显示问题。
2. 修改sdk文档

## Submodule Versions

1. camera sdk version v2.9.47
2. light-magic sdk version v3.4.14

# Aurora 900 Release 1.0.25:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 原始数据大小会随上位机设置分辨率改变，修改存原始数据逻辑

## Submodule Versions

1. camera sdk version v2.9.47
2. light-magic sdk version v3.4.14

# Aurora 900 Release 1.0.24:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 透传一些设置接口给外部 SendFileToDevice  GetFileFromDevice ConfigKeys

## Submodule Versions

1. camera sdk version v2.9.46
2. light-magic sdk version v3.4.14

# Aurora 900 Release 1.0.23:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 更换算法版本，优化点云中出现飞点

## Submodule Versions

1. camera sdk version v2.9.44
2. light-magic sdk version v3.4.14

# Aurora 900 Release 1.0.22:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 兼容Aurora900
2. 修改camera sdk版本，解决raw数据不对问题

## Submodule Versions

1. camera sdk version v2.9.44
2. light-magic sdk version v3.4.12

# Aurora 900 Release 1.0.21:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 更新算法，处理点云数据

## Submodule Versions

1. camera sdk version v2.9.41
2. light-magic sdk version v3.4.12

# Aurora 900 Release 1.0.20:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修改支持显示rgb yuv格式
2. 修改热插拔逻辑，为了兼容400心跳逻辑

## Submodule Versions

1. camera sdk version v2.9.41
2. light-magic sdk version v3.4.7

# Aurora 900 Release 1.0.19:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 模组支持设置出流选项

## Submodule Versions

1. camera sdk version v2.9.40
2. light-magic sdk version v3.4.7

# Aurora 900 Release 1.0.18:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 支持切换分辨率，设置ir帧率，rgb帧率和ir帧率同步，所以不设置，另外帧率大小只可设为10fps和12fps

## Submodule Versions

1. camera sdk version v2.9.39
2. light-magic sdk version v3.4.5

# Aurora 900 Release 1.0.17:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修改算法版本，优化生成点云接口

## Submodule Versions

1. camera sdk version v2.9.37
2. light-magic sdk version v3.4.4

# Aurora 900 Release 1.0.16:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 930非对齐模式下调用GeneratePointCloudFromIrDepth

## Submodule Versions

1. camera sdk version v2.9.37
2. light-magic sdk version v3.4.2

# Aurora 900 Release 1.0.15:

## Major Features and Improvements
## Breaking Changes
## Bug Fixes and Other Changes
1. sample用opencv显示，方便测长稳
2. 增加调用算法对齐接口开关。
## Submodule Versions
1. camera sdk version v2.9.31
2. light-magic sdk version v3.3.61
# Aurora 900 Release 1.0.14:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 930模组增加debug模式
2. 修复开流失败不返回错误码

## Submodule Versions

1. camera sdk version v2.9.30
2. light-magic sdk version v3.3.61

# Aurora 900 Release 1.0.13:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 930算法下移到模组里，添加了*fitting_aurora930
2. camera sdk兼容商汤910
3. 910适配热插拔

## Submodule Versions

1. camera sdk version v2.9.29
2. light-magic sdk version v3.3.61

# Aurora 900 Release 1.0.12:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 930算法下移到模组里，添加了*fitting_aurora930

## Submodule Versions

1. camera sdk version v2.9.28
2. light-magic sdk version v3.3.61

# Aurora 900 Release 1.0.11:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 停流时清空队列

## Submodule Versions

1. camera sdk version v2.9.23-rgbyuv
2. light-magic sdk version v3.3.54

# Aurora 900 Release 1.0.10:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 当前适配900 910 930设备，但910的camera sdk是特殊版本

## Submodule Versions

1. camera sdk version v2.9.20
2. light-magic sdk version v3.3.52

# Aurora 900 Release 1.0.9:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 适配新的deptrum，修改common_types.h

## Submodule Versions

1. camera sdk version v2.9.17
2. light-magic sdk version v3.3.47

# Aurora 900 Release 1.0.8:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 增加910设备

## Submodule Versions

1. camera sdk version v2.9.12-rgbyuv
2. light-magic sdk version v4.3.43

# Aurora 900 Release 1.0.6:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 适配930设备

## Submodule Versions

1. camera sdk version v2.9.9
2. light-magic sdk version v3.3.38

# Aurora 900 Release 1.0.5:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 规范sdk包格式，远距离版本

## Submodule Versions

1. camera sdk version 2.8.72
2. light-magic sdk version 3.3.28

# Aurora 900 Release 1.0.4:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 规范sdk包格式，近距离版本

## Submodule Versions

1. camera sdk version 2.8.72
2. light-magic sdk version 3.3.28

# Aurora 900 Release 1.0.3:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 支持近距离版本，分辨率为960 * 1536，深度距离150~1000。

## Submodule Versions

1. camera sdk version 2.8.64
2. light-magic sdk version 3.3.10

# Aurora 900 Release 1.0.2:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 修复GetFrames接口获取额外数据崩溃问题，，深度距离300~2000。

## Submodule Versions

1. camera sdk version 2.8.64
2. light-magic sdk version 3.3.10

# Aurora 900 Release 1.0.1:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

1. 算法更新，FOV缩小版本。
2. 提供接口，控制depth ir镜像开关。

## Submodule Versions

1. camera sdk version 2.8.64
2. light-magic sdk version 3.3.10



# Aurora 900 Release 1.0.0:

## Major Features and Improvements

## Breaking Changes

## Bug Fixes and Other Changes

## Submodule Versions

1. camera sdk version 2.8.58
2. light-magic sdk version 3.1.6