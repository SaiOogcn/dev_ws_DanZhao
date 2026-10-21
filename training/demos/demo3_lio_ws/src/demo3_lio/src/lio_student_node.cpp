#include "demo3_lio/lio_student_node.hpp"

#include <cmath>

LioStudentNode::LioStudentNode() : Node("demo3_lio_student") {
  declare_parameter("voxel_size", voxel_size_);
  declare_parameter("keyframe_distance", keyframe_distance_);
  declare_parameter("scan_min_range", scan_min_range_);
  declare_parameter("scan_max_range", scan_max_range_);
  declare_parameter("scan_topic", scan_topic_);
  declare_parameter("imu_topic", imu_topic_);
  declare_parameter("odom_topic", odom_topic_);
  declare_parameter("estimate_topic", estimate_topic_);
  declare_parameter("path_topic", path_topic_);
  declare_parameter("odom_frame", odom_frame_);
  declare_parameter("base_frame", base_frame_);
  declare_parameter("laser_frame", laser_frame_);
  declare_parameter("imu_frame", imu_frame_);
  declare_parameter("laser_x", laser_x_);
  declare_parameter("laser_y", laser_y_);
  declare_parameter("laser_yaw", laser_yaw_);
  get_parameter("voxel_size", voxel_size_);
  get_parameter("keyframe_distance", keyframe_distance_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("estimate_topic", estimate_topic_);
  get_parameter("path_topic", path_topic_);
  get_parameter("odom_frame", odom_frame_);
  get_parameter("base_frame", base_frame_);
  get_parameter("laser_frame", laser_frame_);
  get_parameter("imu_frame", imu_frame_);
  get_parameter("laser_x", laser_x_);
  get_parameter("laser_y", laser_y_);
  get_parameter("laser_yaw", laser_yaw_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioStudentNode::on_scan, this, std::placeholders::_1));
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioStudentNode::on_imu, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&LioStudentNode::on_odom, this, std::placeholders::_1));
  estimate_pub_ = create_publisher<nav_msgs::msg::Odometry>(estimate_topic_, 10);
  // TODO(学员): 轨迹话题已经建好，骨架不发布。需要观测轨迹时在匹配更新后
  // 追加位姿并发布，控制器和 RViz 都订阅这个话题。
  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
  RCLCPP_WARN(get_logger(),
              "LIO training skeleton: fusion, filtering and scan matching are TODOs");
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

  // TODO(学员 1): 用 scan.time_increment 和 IMU 缓存插值，完成逐束去畸变。
  // 束的时间方向和 scan 时间戳语义不在参数里，先从 time_increment 判断：
  // 参考实现 lio_hybird 用 scan_index_time_reversed=true、scan_stamp_is_end=true。
  // TODO(学员 2): 用 scan_min_range/scan_max_range 过滤 NaN/Inf、越界点、
  // 盲区点和孤立点，并统计过滤比例。
  // TODO(学员 3): 使用 laser->imu->base 外参把点变换到统一坐标系。
  // TODO(学员 4): 用 odom+IMU 做预测，完成 scan-to-scan 或 scan-to-map 匹配。
  // TODO(学员 5): 建立体素子地图、关键帧和观测拒绝门限，拒绝错误匹配。
  publish_estimate(msg->header.stamp);
}

void LioStudentNode::publish_estimate(const builtin_interfaces::msg::Time &stamp) {
  // 这是明确的占位基线：复制 odom 只为让 TF/控制器接口可观察，不能算 LIO。
  // TODO(学员): 用自己的融合结果替换这里。如果要补 TF，注意底盘 EKF 已经
  // 发布 odom -> base_footprint，同一条边不能有两个发布者。
  auto estimate = latest_odom_;
  estimate.header.stamp = stamp;
  estimate.header.frame_id = odom_frame_;
  estimate.child_frame_id = base_frame_;
  estimate_pub_->publish(estimate);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioStudentNode>());
  rclcpp::shutdown();
  return 0;
}
