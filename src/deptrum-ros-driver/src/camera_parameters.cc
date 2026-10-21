#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.hpp>
#include "deptrum_ros_driver/camera_parameters.h"

void StreamCameraParameters::Initialize(std::shared_ptr<deptrum::stream::Device> device,
                                        const int boot_order,
                                        DeptrumRosDeviceParams params) {
  boot_order_ = boot_order;
  params_ = params;
  if (device == nullptr) {
    LOG(ERROR) << "BOOT_ORDER_" << boot_order_ << "::Device Not Created";
    rclcpp::shutdown();
    exit(-1);
  }

  int status = device->GetCameraParameters(ir_intri_, rgb_intri_, ext_);
  if (status != 0) {
    LOG(ERROR) << "BOOT_ORDER_" << boot_order_ << "::Get camera parameters failed, " << status;
    rclcpp::shutdown();
    exit(status);
  }
  PrintCameraInformation();
  PublishDepthToRgbTf();
  if (boot_order_ > 1) {
    std::string frame_id = std::to_string(boot_order_);
    SetFrameidSuffix(frame_id);
  }
}

int StreamCameraParameters::GetIrCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) {
  camera_info.header.frame_id = depth_camera_frame_;
  camera_info.width = GetIrWidth();
  camera_info.height = GetIrHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
  camera_info.d = {ir_intri_.distortion_coeffs[0],
                   ir_intri_.distortion_coeffs[1],
                   ir_intri_.distortion_coeffs[2],
                   ir_intri_.distortion_coeffs[3],
                   ir_intri_.distortion_coeffs[4]};

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  camera_info.k = {ir_intri_.focal_length[0],
                   0.0f,
                   ir_intri_.principal_point[0],
                   0.0f,
                   ir_intri_.focal_length[1],
                   ir_intri_.principal_point[1],
                   0.0f,
                   0.0,
                   1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  camera_info.p = {ir_intri_.focal_length[0],
                   0.0f,
                   ir_intri_.principal_point[0],
                   0.0f,
                   0.0f,
                   ir_intri_.focal_length[1],
                   ir_intri_.principal_point[1],
                   0.0f,
                   0.0f,
                   0.0,
                   1.0f,
                   0.0f};

  // Rectification matrix (stereo cameras only)
  camera_info.r = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  return 0;
}

int StreamCameraParameters::GetRgbCameraInfo(sensor_msgs::msg::CameraInfo& camera_info) {
  camera_info.header.frame_id = rgb_camera_frame_;
  camera_info.width = GetRgbWidth();
  camera_info.height = GetRgbHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
  camera_info.d = {rgb_intri_.distortion_coeffs[0],
                   rgb_intri_.distortion_coeffs[1],
                   rgb_intri_.distortion_coeffs[2],
                   rgb_intri_.distortion_coeffs[3],
                   rgb_intri_.distortion_coeffs[4]};

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  camera_info.k = {rgb_intri_.focal_length[0],
                   0.0f,
                   rgb_intri_.principal_point[0],
                   0.0f,
                   rgb_intri_.focal_length[1],
                   rgb_intri_.principal_point[1],
                   0.0f,
                   0.0,
                   1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  camera_info.p = {rgb_intri_.focal_length[0],
                   0.0f,
                   rgb_intri_.principal_point[0],
                   0.0f,
                   0.0f,
                   rgb_intri_.focal_length[1],
                   rgb_intri_.principal_point[1],
                   0.0f,
                   0.0f,
                   0.0,
                   1.0f,
                   0.0f};

  // Rectification matrix (stereo cameras only)
  camera_info.r = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  return 0;
}

void StreamCameraParameters::PrintCameraInformation() {
  LOG(INFO) << "BOOT_ORDER_" << boot_order_ << "::Camera Parameters:";
  LOG(INFO) << "BOOT_ORDER_" << boot_order_ << "::\t Ir Intrinsics:";
  PrintCameraIntrinsicParam(ir_intri_);

  LOG(INFO) << "BOOT_ORDER_" << boot_order_ << "::\t Rgb Intrinsics:";
  PrintCameraIntrinsicParam(rgb_intri_);

  LOG(INFO) << "BOOT_ORDER_" << boot_order_ << "::\t Ir to Rgb Extrinsic:";
  PrintCameraExtrinsicParameters(ext_);
}

void StreamCameraParameters::PrintCameraIntrinsicParam(const deptrum::Intrinsic& intri) {
  LOG(INFO) << "\t\t Resolution:";
  LOG(INFO) << "\t\t\t Rows:" << intri.rows;
  LOG(INFO) << "\t\t\t Cols:" << intri.cols;
  LOG(INFO) << "\t\t Intrinsics:";
  LOG(INFO) << "\t\t\t fx: " << intri.focal_length[0];
  LOG(INFO) << "\t\t\t fy: " << intri.focal_length[1];
  LOG(INFO) << "\t\t\t cx: " << intri.principal_point[0];
  LOG(INFO) << "\t\t\t cy: " << intri.principal_point[1];
  LOG(INFO) << "\t\t\t k1: " << intri.distortion_coeffs[0];
  LOG(INFO) << "\t\t\t k2: " << intri.distortion_coeffs[1];
  LOG(INFO) << "\t\t\t p1: " << intri.distortion_coeffs[2];
  LOG(INFO) << "\t\t\t p2: " << intri.distortion_coeffs[3];
  LOG(INFO) << "\t\t\t k3: " << intri.distortion_coeffs[4];
}

void StreamCameraParameters::PrintCameraExtrinsicParameters(const deptrum::Extrinsic& ext) {
  LOG(INFO) << "\t\t Extrinsics:";
  LOG(INFO) << "\t\t\t Rotation[0]: " << ext.rotation_matrix[0] << ", " << ext.rotation_matrix[1]
            << ", " << ext.rotation_matrix[2];
  LOG(INFO) << "\t\t\t Rotation[1]: " << ext.rotation_matrix[3] << ", " << ext.rotation_matrix[4]
            << ", " << ext.rotation_matrix[5];
  LOG(INFO) << "\t\t\t Rotation[2]: " << ext.rotation_matrix[6] << ", " << ext.rotation_matrix[7]
            << ", " << ext.rotation_matrix[8];
  LOG(INFO) << "\t\t\t Translation: " << ext.translation_vector[0] << ", "
            << ext.translation_vector[1] << ", " << ext.translation_vector[2];
}

void StreamCameraParameters::PublishDepthToRgbTf() {
  tf2::Vector3 depth_to_rgb_translation(ext_.translation_vector[0] / 1000.0f,
                                        ext_.translation_vector[1] / 1000.0f,
                                        ext_.translation_vector[2] / 1000.0f);
  tf2::Matrix3x3 depth_to_rgb_rotation(ext_.rotation_matrix[0],
                                       ext_.rotation_matrix[1],
                                       ext_.rotation_matrix[2],
                                       ext_.rotation_matrix[3],
                                       ext_.rotation_matrix[4],
                                       ext_.rotation_matrix[5],
                                       ext_.rotation_matrix[6],
                                       ext_.rotation_matrix[7],
                                       ext_.rotation_matrix[8]);
  tf2::Transform depth_to_rgb_transform(depth_to_rgb_rotation, depth_to_rgb_translation);

  geometry_msgs::msg::TransformStamped static_transform;
  static_transform.transform = tf2::toMsg(depth_to_rgb_transform);

  static_transform.header.stamp = rclcpp::Clock().now();
  static_transform.header.frame_id = depth_camera_frame_;
  static_transform.child_frame_id = rgb_camera_frame_;

  static_broadcaster_->sendTransform(static_transform);
}

void StreamCameraParameters::SetFrameidSuffix(std::string boot_order) {
  // tf_prefix_ += "_" + boot_order;
  rgb_camera_frame_ += "_" + boot_order;
  depth_camera_frame_ += "_" + boot_order;
  // ir_camera_frame_ += "_" + boot_order;
   imu_frame_ += "_" + boot_order;
}
