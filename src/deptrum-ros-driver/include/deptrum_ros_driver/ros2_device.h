#ifndef ROS2_DEVICE_H
#define ROS2_DEVICE_H

#include <atomic>
#include <memory>
#include "deptrum/common_types.h"
#include "deptrum_ros_driver/camera_parameters.h"
// #include "deptrum/deptrum_stream.h"
#include "deptrum/device.h"
#include "deptrum/stream_types.h"
// #include "deptrum/stellar400.h"
#include <cv_bridge/cv_bridge.h>
#include "deptrum/stream.h"
#include "rclcpp/rclcpp.hpp"
#include "ros2_types.h"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
class RosDevice {
 private:
 public:
  RosDevice(std::shared_ptr<rclcpp::Node> node);
  virtual ~RosDevice();
  std::shared_ptr<rclcpp::Node> node_;
  // std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  // std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  virtual int Start() = 0;
  virtual void Stop() = 0;

  virtual void InitDevice() = 0;
  virtual bool SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) = 0;
  virtual void FramePublisherThread();
  //  virtual void RegisterTopic() = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_camerainfo_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_camerainfo_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr camera_version_publisher_;

  std::atomic<bool> running_;
  std::thread frame_publisher_thread_;
  std::thread camera_version_publisher_thread_;
  std::thread sn_publisher_thread_;
  // int RgbFrameToRos(sensor_msgs::ImagePtr &rgb_image, const deptrum::stream::StreamFrame
  // &rgb_frame) int RgbFrameToRos(sensor_msgs::msg::Image::UniquePtr& msg, const
  // deptrum::stream::StreamFrame& rgb_frame);

  DeptrumRosDeviceParams params_;
  deptrum::stream::DeviceDescription device_information_;
  std::string FrameModeToString(deptrum::FrameMode mode);
  unsigned int hot_plug_num = 0;
  std::shared_ptr<deptrum::stream::Device> device_;
  deptrum::stream::Stream* streams_;
  StreamCameraParameters camera_parameters_;

  virtual int rgb_frame_publish(const deptrum::stream::StreamFrame& rgb_frame,
                                const builtin_interfaces::msg::Time& capture_time);
  virtual int ir_frame_publish(const deptrum::stream::StreamFrame& ir_frame,
                               const builtin_interfaces::msg::Time& capture_time);
  virtual int depth_frame_publish(const deptrum::stream::StreamFrame& depth_frame,
                                  const builtin_interfaces::msg::Time& capture_time);
  void TimestampToRos(const uint64_t& frame_timestamp_us, builtin_interfaces::msg::Time& ros_time);
  std::string mat_type2encoding(int mat_type);
  // int PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
  //                          const deptrum::stream::StreamFrame& point_frame);
  void DeviceConnectCb(int flag, const deptrum::DeviceInformation& device_information);
  void UpgradeProgress(int process);

  std::function<void(int)> upgrade_handle_;
  std::function<void(int, const deptrum::DeviceInformation&)> handle_;
  // std::function<void(bool)> heart_beat_handle_;
  void PrintFPS();

  int PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
                           const deptrum::stream::StreamFrame& point_frame);
  int PointCloudFrameToColorRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                const deptrum::stream::StreamFrame& point_frame);
};

#endif  // ROS2_DEVICE_H
