#include "demo3_lio/lio_student_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

LioStudentController::LioStudentController() : Node("demo3_lio_controller") {
  declare_parameter("enable_motion", enable_motion_);
  declare_parameter("odom_topic", odom_topic_);
  declare_parameter("scan_topic", scan_topic_);
  declare_parameter("path_topic", path_topic_);
  declare_parameter("path_file", path_file_);
  declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter("lookahead_distance", lookahead_distance_);
  declare_parameter("linear_speed", linear_speed_);
  declare_parameter("max_angular_z", max_angular_z_);
  declare_parameter("goal_tolerance", goal_tolerance_);
  declare_parameter("obstacle_stop_distance", obstacle_stop_distance_);
  declare_parameter("obstacle_sector_rad", obstacle_sector_rad_);
  declare_parameter("control_period_s", control_period_s_);
  get_parameter("enable_motion", enable_motion_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("scan_topic", scan_topic_);
  get_parameter("path_topic", path_topic_);
  get_parameter("path_file", path_file_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("lookahead_distance", lookahead_distance_);
  get_parameter("linear_speed", linear_speed_);
  get_parameter("max_angular_z", max_angular_z_);
  get_parameter("goal_tolerance", goal_tolerance_);
  get_parameter("obstacle_stop_distance", obstacle_stop_distance_);
  get_parameter("obstacle_sector_rad", obstacle_sector_rad_);
  get_parameter("control_period_s", control_period_s_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&LioStudentController::on_odom, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioStudentController::on_scan, this, std::placeholders::_1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 1, std::bind(&LioStudentController::on_path, this, std::placeholders::_1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  timer_ = create_wall_timer(std::chrono::duration<double>(control_period_s_),
                             std::bind(&LioStudentController::on_timer, this));
  load_path();
  RCLCPP_INFO(get_logger(), "LIO controller output=%s enable_motion=%s",
              cmd_vel_topic_.c_str(), enable_motion_ ? "true" : "false");
}

void LioStudentController::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_ = *msg;
  have_odom_ = true;
}

void LioStudentController::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_scan_ = *msg;
  have_scan_ = true;
}

void LioStudentController::on_path(const nav_msgs::msg::Path::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = *msg;
  have_path_ = !path_.poses.empty();
}

void LioStudentController::load_path() {
  if (path_file_.empty()) return;
  std::ifstream input(path_file_);
  if (!input) {
    RCLCPP_WARN(get_logger(), "path_file cannot be opened: %s", path_file_.c_str());
    return;
  }
  nav_msgs::msg::Path loaded;
  loaded.header.frame_id = "odom";
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') continue;
    std::stringstream stream(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(stream, field, ',')) {
      try { values.push_back(std::stod(field)); } catch (...) { values.clear(); break; }
    }
    if (values.size() < 3) continue;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = loaded.header.frame_id;
    pose.pose.position.x = values[0];
    pose.pose.position.y = values[1];
    const double yaw = values[2];
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    loaded.poses.push_back(pose);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = loaded;
  have_path_ = !path_.poses.empty();
}

geometry_msgs::msg::Twist LioStudentController::stop_command() const {
  return geometry_msgs::msg::Twist{};
}

void LioStudentController::on_timer() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enable_motion_ || !have_odom_ || !have_scan_ || !have_path_) return;

  // 保留一个可验证的最低安全功能：前方近距离障碍物只发布零速度。
  for (std::size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
    const double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
    const float range = latest_scan_.ranges[i];
    if (std::abs(angle) <= obstacle_sector_rad_ && std::isfinite(range) &&
        range >= latest_scan_.range_min && range <= obstacle_stop_distance_) {
      cmd_pub_->publish(stop_command());
      return;
    }
  }

  // TODO(学员): 下面是 demo3 的主要考察内容，当前故意不产生运动指令：
  // 1. 把 path_ 变换到 latest_odom_.child_frame_id；
  // 2. 找到最近点和 lookahead 点，处理路径方向与终点；
  // 3. 根据阿克曼几何/曲率限制生成 Twist；
  // 4. 增加障碍物确认、绕行、重新捕获路径的状态机；
  // 5. 到达终点、超时、LIO 质量下降时可靠停车。
  // 完成前必须保持零速度，完成后再删除这一行并逐项实车验证。
  cmd_pub_->publish(stop_command());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioStudentController>());
  rclcpp::shutdown();
  return 0;
}
