# Deptrum 相机 示例代码
这些示例代码用来演示获取Depth、IR、Color、点云等.  详细的接口文档和示例解释参考 (../docs)。

## 示例
| **名称**         | **语言** | **描述**                                     | **注意事项** |
| ---------------- | -------- | -------------------------------------------- | ------------ |
| hello_deptrum    | C++      | 演示连接到设备获取SDK版本和设备信息          |
| sample_viewer    | C++      | 演示使用SDK获取Depth/Ir/Rgb等流，并显示      |
| sample_lite      | C++      | 演示使用SDK获取Depth/Ir/Rgb等流，不进行显示  |
| firmware_upgrade | C++      | 演示选择固件bin或者img文件给设备升级固件版本 |

## 编译
创建编译目录build，进入build目录
    1)  win: 默认编译x86,若编译Win64,以vs2015编译为例子, `cmake .. -G "Visual Studio 14 2015 Win64"`
    2)  other platform: `cmake ..`
注意：windows 需配置环境变量，OpenCV_DIR