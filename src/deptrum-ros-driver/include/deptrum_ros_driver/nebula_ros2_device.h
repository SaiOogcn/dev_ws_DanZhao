#ifndef NEBULA_ROS2_DEVICE_H
#define NEBULA_ROS2_DEVICE_H
#include "deptrum/nebula_series.h"
#include "ros2_device.h"
class nebulaRosDevice : public RosDevice {
 public:
  // using RosDevice::RosDevice;
  nebulaRosDevice(std::shared_ptr<rclcpp::Node> node);
  virtual ~nebulaRosDevice();
  void InitDevice() override;
  //   void RegisterTopic() override;
  bool SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) override;
  int Start() override;
  void Stop() override;

 private:
  int PointCloudFrameToRos(sensor_msgs::msg::PointCloud2& point_cloud,
                           const deptrum::stream::StreamFrame& point_frame);
  int PointCloudFrameToColorRos(sensor_msgs::msg::PointCloud2& point_cloud,
                                const deptrum::stream::StreamFrame& point_frame);

  void FramePublisherThread() override;
  void SerialNumberPublisherThread();

  void ImuFramePublisher();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stof_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr flag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stof_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rd_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rmd_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ext_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sn_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr stof_pointcloud_publisher_;
  int stof_depth_frame_publish(const deptrum::stream::StreamFrame& depth_frame,
                               const builtin_interfaces::msg::Time& capture_time);
  int stof_rgb_frame_publish(const deptrum::stream::StreamFrame& rgb_frame,
                             const builtin_interfaces::msg::Time& capture_time);
  int flag_frame_publish(const deptrum::stream::StreamFrame& flag_frame,
                         const builtin_interfaces::msg::Time& capture_time);
  int ext_rgb_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rgb_frame,
                            const builtin_interfaces::msg::Time& capture_time);
  int rd_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rd_frame,
                       const builtin_interfaces::msg::Time& capture_time);
  int rmd_frame_publish(const deptrum::stream::CustomFrame<uint8_t>& rmd_frame,
                        const builtin_interfaces::msg::Time& capture_time);
  int imu_frame_publish(const deptrum::stream::OpaqueData<uint8_t>& imu_frame);
};
#endif  //  NEBULA_ROS2_DEVICE_H