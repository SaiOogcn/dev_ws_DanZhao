#pragma once

#include <deque>
#include <functional>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class LioStudentNode final : public rclcpp::Node {
public:
  LioStudentNode();

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_estimate(const builtin_interfaces::msg::Time &stamp);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr estimate_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::mutex mutex_;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_{false};
  bool have_imu_{false};
  double voxel_size_{0.20};
  double keyframe_distance_{0.20};
  double max_range_{8.0};
};
