#ifndef STELLAR400_ROS2_DEVICE_H
#define STELLAR400_ROS2_DEVICE_H
#include "deptrum/stellar400_series.h"
#include "ros2_device.h"
class Stellar400RosDevice : public RosDevice {
 public:
  // using RosDevice::RosDevice;
  Stellar400RosDevice(std::shared_ptr<rclcpp::Node> node);
  virtual ~Stellar400RosDevice();
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

  void PointCloudFramePublisherThread();
  void RgbFramePublisherThread() override;
  void DepthIrFramePublisherThread() override;
  void HeartBeatCb(int result);
  std::function<void(bool)> heart_beat_handle_;
  // void RgbFramePublisherThread() override;
  // void DepthIrFramePublisherThread() override;
};
#endif  // STELLAR400_ROS2_DEVICE_H