#include "demo3_lio/lio_student_node.hpp"

#include <cmath>

LioStudentNode::LioStudentNode() : Node("demo3_lio_student") {
  declare_parameter("voxel_size", voxel_size_);
  declare_parameter("keyframe_distance", keyframe_distance_);
  declare_parameter("max_range", max_range_);
  get_parameter("voxel_size", voxel_size_);
  get_parameter("keyframe_distance", keyframe_distance_);
  get_parameter("max_range", max_range_);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&LioStudentNode::on_scan, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&LioStudentNode::on_imu, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, std::bind(&LioStudentNode::on_odom, this, std::placeholders::_1));
  estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>("/demo3_lio/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/demo3_lio/path", 10);
}

void LioStudentNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_buffer_.push_back(*msg);
  while (imu_buffer_.size() > 400) imu_buffer_.pop_front();
  have_imu_ = true;
}

void LioStudentNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
}

void LioStudentNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_imu_ || !have_odom_) return;
  // TODO(trainee 1): 用每束时间戳从 imu_buffer_ 插值，完成扫描去畸变。
  // TODO(trainee 2): 删除 NaN、越界、孤立点和明显不符合邻束连续性的假点。
  // TODO(trainee 3): 将点变换到 base/odom，并用轮速 + IMU 预测给 NDT/ICP 初值。
  // TODO(trainee 4): 完成体素子地图、NDT 匹配和观测拒绝门限。
  publish_estimate(msg->header.stamp);
}

void LioStudentNode::publish_estimate(const builtin_interfaces::msg::Time &stamp) {
  auto estimate = latest_odom_;
  estimate.header.stamp = stamp;
  estimate.header.frame_id = "odom";
  estimate.child_frame_id = "base_footprint";
  estimate_pub_->publish(estimate);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioStudentNode>());
  rclcpp::shutdown();
  return 0;
}
