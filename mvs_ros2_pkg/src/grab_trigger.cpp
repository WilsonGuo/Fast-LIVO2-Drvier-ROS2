#include "MvCameraControl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

using namespace std;

// 日志宏
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)

struct time_stamp
{
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

enum PixelFormat : unsigned int
{
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};
std::vector<PixelFormat> PIXEL_FORMAT = {RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8};

// 全局参数
bool exit_flag = false;
int width, height;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
sensor_msgs::msg::CameraInfo camera_info_msg_;
float image_scale = 1.0;
int trigger_enable = 1;

// -------------------- Signal --------------------
void SignalHandler(int signal)
{
  if (signal == SIGINT)
  {
    ROS_INFO("Received Ctrl+C, exiting...");
    exit_flag = true;
  }
}

void SetupSignalHandler()
{
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

// -------------------- CameraInfo --------------------
void fillCameraInfoFromYAML(cv::FileStorage &fs)
{
  camera_info_msg_.width = width;
  camera_info_msg_.height = height;
  camera_info_msg_.distortion_model = "plumb_bob";

  // D
  camera_info_msg_.d.clear();
  cv::FileNode nodeD = fs["D"];
  for (auto it = nodeD.begin(); it != nodeD.end(); ++it)
  {
    camera_info_msg_.d.push_back((double)(*it));
  }

  // K
  camera_info_msg_.k.fill(0);
  cv::FileNode nodeK = fs["K"];
  int idx = 0;
  for (auto it = nodeK.begin(); it != nodeK.end(); ++it)
  {
    camera_info_msg_.k[idx++] = (double)(*it);
  }

  // R
  camera_info_msg_.r.fill(0);
  cv::FileNode nodeR = fs["R"];
  idx = 0;
  for (auto it = nodeR.begin(); it != nodeR.end(); ++it)
  {
    camera_info_msg_.r[idx++] = (double)(*it);
  }

  // P
  camera_info_msg_.p.fill(0);
  cv::FileNode nodeP = fs["P"];
  idx = 0;
  for (auto it = nodeP.begin(); it != nodeP.end(); ++it)
  {
    camera_info_msg_.p[idx++] = (double)(*it);
  }
}

// -------------------- Worker Thread --------------------
static void *WorkThread(void *pUser)
{
  int nRet = MV_OK;

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet)
  {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]", nRet);
    return nullptr;
  }

  MV_FRAME_OUT stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

  ROS_INFO("Capture loop start.");

  cv::Mat srcImage;
  std::vector<uint8_t> rgbBuffer(stParam.nCurValue * 3);

  while (!exit_flag && rclcpp::ok())
  {
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 10000);
    if (nRet != MV_OK)
    {
      ROS_WARN("Capture timeout, retrying...");
      continue;
    }

    if (srcImage.empty() ||
        srcImage.cols != stImageInfo.stFrameInfo.nWidth ||
        srcImage.rows != stImageInfo.stFrameInfo.nHeight)
    {
      srcImage.create(stImageInfo.stFrameInfo.nHeight,
                      stImageInfo.stFrameInfo.nWidth,
                      CV_8UC3);
      width = stImageInfo.stFrameInfo.nWidth;
      height = stImageInfo.stFrameInfo.nHeight;
    }

    stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;
    stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;
    stConvertParam.pSrcData = stImageInfo.pBufAddr;
    stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
    stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = rgbBuffer.data();
    stConvertParam.nDstBufferSize = rgbBuffer.size();

    nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
    if (nRet != MV_OK)
    {
      ROS_WARN("MV_CC_ConvertPixelType failed! nRet [%x]", nRet);
      MV_CC_FreeImageBuffer(pUser, &stImageInfo);
      continue;
    }

    memcpy(srcImage.data, rgbBuffer.data(), rgbBuffer.size());

    // ------------------- 安全版 timestamp -------------------
    rclcpp::Time stamp;
    if (trigger_enable && pointt && pointt != MAP_FAILED && pointt->low != 0)
    {
      double t = pointt->low / 1e9;
      stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    }
    else
    {
      stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }

    // ------------------- 发布图像 -------------------
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.height = srcImage.rows;
    msg.width = srcImage.cols;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = srcImage.step;
    msg.data.assign(srcImage.data, srcImage.data + srcImage.total() * srcImage.elemSize());

    // ------------------- 发布 camera_info -------------------
    sensor_msgs::msg::CameraInfo info_msg = camera_info_msg_;
    info_msg.header.stamp = stamp;

    pub->publish(msg);
    camera_info_pub->publish(info_msg);

    MV_CC_FreeImageBuffer(pUser, &stImageInfo);
  }

  ROS_INFO("Capture thread exiting.");
  return nullptr;
}

// -------------------- Main --------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    ROS_ERROR("Please provide params file path!");
    return -1;
  }
  std::string params_file = argv[1];

  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened())
  {
    ROS_ERROR("Failed to open settings file at: %s", params_file.c_str());
    return -1;
  }

  trigger_enable = (int)Params["TriggerEnable"];
  std::string pub_topic = (std::string)Params["TopicName"];
  int PixelFormat = (int)Params["PixelFormat"];
  image_scale = (float)Params["image_scale"];
  if (image_scale < 0.1)
    image_scale = 1;

  auto node = rclcpp::Node::make_shared("mvs_trigger");
  pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);
  camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(pub_topic + "/camera_info", 10);

  const char *user_name = getlogin();
  std::string path_for_time_stamp = "/home/" + std::string(user_name) + "/timeshare";
  int fd = open(path_for_time_stamp.c_str(), O_RDWR);
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  SetupSignalHandler();

  // 枚举设备
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(stDeviceList));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (nRet != MV_OK || stDeviceList.nDeviceNum == 0)
  {
    ROS_ERROR("No MVS device found.");
    return -1;
  }

  // 创建handle
  void *handle = nullptr;
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);
  if (nRet != MV_OK)
  {
    ROS_ERROR("MV_CC_CreateHandle fail!");
    return -1;
  }

  MV_CC_OpenDevice(handle);
  MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[PixelFormat]);
  MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);

  MV_CC_StartGrabbing(handle);

  width = (int)Params["Width"];
  height = (int)Params["Height"];
  
  // 读取相机参数
  fillCameraInfoFromYAML(Params);

  // 启动线程
  pthread_t thread_id;
  nRet = pthread_create(&thread_id, NULL, WorkThread, handle);
  if (nRet != 0)
  {
    ROS_ERROR("thread create failed.ret = %d", nRet);
    return -1;
  }

  while (rclcpp::ok() && !exit_flag)
  {
    rclcpp::spin_some(node);
    usleep(100000);
  }

  pthread_join(thread_id, NULL);

  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);

  munmap(pointt, sizeof(time_stamp));

  rclcpp::shutdown();
  return 0;
}
