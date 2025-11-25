# Fast-LIVO2-Drvier-ROS2
针对ROS2适配雷达和相机驱动，以及Vikit

Fast-LIVO2-ROS2版本源码，请参照这里[链接](https://github.com/integralrobotics/FAST-LIVO2)

使用教程，请参阅这里[链接](https://gitee.com/gwmunan/ros2/wikis/%E5%AE%9E%E6%88%98%E6%95%99%E7%A8%8B/Fast-LIVO2%E6%8B%93%E5%B1%95%E4%B9%8BROS2%E9%80%82%E9%85%8D)
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
