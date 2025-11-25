# Fast-LIVO2-Drvier-ROS2
针对ROS2适配雷达和相机驱动，以及Vikit
### 适配平台：
Ubuntu22.04
ROS2 Humble
硬件X86平台，ARM平台尚未验证

### 具体说明：

1.livox_ros_driver2清理了不适合ROS2的日志打印

2.mvs_ros2_pkg是将LIV_handhold中的mvs_ros_driver驱动包改为ROS2版本

3.添加Vikit，以适配Fast LIVO2的ROS2版本

#### Sophus安装

正常直接在终端中，输入以下指令：

sudo apt install ros-humble-sophus

如果有链接错误（例如vikit_common找不到sophus），也可以先删除旧版 Sophus残余

sudo rm -rf /usr/local/include/sophus
sudo rm -rf /usr/local/lib/cmake/Sophus

再重新安装

sudo apt install ros-humble-sophus
