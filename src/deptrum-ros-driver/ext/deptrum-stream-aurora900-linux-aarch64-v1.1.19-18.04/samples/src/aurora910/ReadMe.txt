                                                                                                                                                                                           
linux平台samples样例编译说明
    1. mkdir build
    2. cd build
    3. cmake ..
    4. make
    5. 将facula_sdk.so或者facula_sdk.so*(*代表版本号)拷贝到生成的可执行facula_test目录下，执行程序

windows平台samples样例编译说明
    1. mkdir build
    2. cd build
    3. cmake ..
    4. 使用VS2019(或其他VS版本)编译release程序
    5. 将facula_sdk.dll拷贝到facula_test.exe目录下，执行程序

注：sample程序默认使用了第三方库opencv做显示，版本为4.6.0，附带在3rdparty中。若环境不支持，请做如下处理：

    解决方案1：关闭显示功能。
              编译时，第三步使用cmake .. -DDISABLE_INTERFACE=ON


   解决方案2：手动搭建环境。
    步骤：确认apt-get软件源可用————安装opencv依赖————安装opencv
    1：若软件源不可用，推荐换成国内源，这里以清华源为例：
{
1.备份 sources.list

sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
2.编辑sources.list 文件

sudo vim /etc/apt/sources.list
3.使用#注释原来的内容,并添加以下内容

deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse

（注：在更新国内的软件源时，有的是http的，有的是https的，然后当更新的软件源是https时，在执行 apt-get update 命令后，可能会出现如下错误：
    E: The method driver /usr/lib/apt/methods/https could not be found.
    N: Is the package apt-transport-https installed?
    该异常是缺少依赖的apt-transport-https，可以在执行 apt-get update 前先安装该依赖包：
    apt-get install apt-transport-https）

然后执行
sudo cp /etc/apt/sources.list.d/raspi.list /etc/apt/sources.list.d/raspi.list.bak 
sudo rm /etc/apt/sources.list.d/raspi.list 

4.更新系统软件 并 更新已安装的包　

sudo apt-get update && apt-get upgrade -y
}
    2：安装opencv依赖：
{
    sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

    sudo apt-get install libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
}   
    3：安装opencv：
{
    此时有3种情况：
    ①：直接使用3rdparty中附带opencv库即可
    ②：有些平台使用静态库编译会报错，若3rdparty中附带了动态库，
       可以将CMakelists.txt中的第五行link_directories(../3rdparty/opencv-4.6.0/lib-static)
       改为link_directories(../3rdparty/opencv-4.6.0/lib-shared)后再试
    ③：若3rdparty中附带的第三方库完全不可用，但仍想使用sample中的显示功能，请手动下载opencv4.6.0版本
       安装后，将CMakelists.txt中的第4和5行注释掉，改为opencv安装路径下对应的include和lib。

}

