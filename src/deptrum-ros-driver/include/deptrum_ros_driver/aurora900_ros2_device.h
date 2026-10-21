#ifndef AURORA900_ROS2_DEVICE_H
#define AURORA900_ROS2_DEVICE_H
#include "deptrum/aurora900_series.h"
#include "ros2_device.h"
class Aurora900RosDevice : public RosDevice {
 public:
  // using RosDevice::RosDevice;
  Aurora900RosDevice(std::shared_ptr<rclcpp::Node> node);
  virtual ~Aurora900RosDevice();
  void InitDevice() override;
  //   void RegisterTopic() override;
  bool SetDeviceParams(std::shared_ptr<deptrum::stream::Device>& device) override;
  int Start() override;
  void Stop() override;

 private:
  // void RgbFramePublisherThread() override;
  // void DepthIrFramePublisherThread() override;
};
#endif  // AURORA900_ROS2_DEVICE_H