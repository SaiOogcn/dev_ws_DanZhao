#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
// #include "deptrum/deptrum_stream.h"
#include "deptrum/device.h"
#include "glog/logging.h"
#include "ros2_types.h"
class StreamCameraParameters {
 public:
  StreamCameraParameters(std::shared_ptr<rclcpp::Node> node) {
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  };
  void Initialize(std::shared_ptr<deptrum::stream::Device> device,
                  const int boot_order = 1,
                  DeptrumRosDeviceParams params = {});

  int GetIrWidth() { return ir_intri_.cols; }
  int GetIrHeight() { return ir_intri_.rows; }
  int GetRgbWidth() { return rgb_intri_.cols; }
  int GetRgbHeight() { return rgb_intri_.rows; }

  int GetIrCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);
  int GetRgbCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

  std::string rgb_camera_frame_ = "rgb_camera_link";
  std::string depth_camera_frame_ = "depth_camera_link";
  std::string imu_frame_ = "imu_link";
  // std::string ir_camera_frame_ = "ir_camera_link";
  void SetFrameidSuffix(std::string boot_order);
  void PrintCameraIntrinsicParam(const deptrum::Intrinsic& intri);
  void PrintCameraExtrinsicParameters(const deptrum::Extrinsic& CameraExtrinsicParameters);
  void PublishDepthToRgbTf();
  void PrintCameraInformation();

  deptrum::Intrinsic ir_intri_;
  deptrum::Intrinsic rgb_intri_;
  deptrum::Extrinsic ext_;
  DeptrumRosDeviceParams params_;

  // tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

 private:
  int boot_order_;
};

#endif  // CAMERA_PARAMETERS_H
