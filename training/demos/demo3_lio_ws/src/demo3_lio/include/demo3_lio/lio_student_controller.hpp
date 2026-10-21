#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LioStudentController final : public rclcpp::Node {
public:
  LioStudentController();

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_path(const nav_msgs::msg::Path::SharedPtr msg);
  void on_timer();
  void load_path();
  geometry_msgs::msg::Twist stop_command() const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  nav_msgs::msg::Odometry latest_odom_;
  sensor_msgs::msg::LaserScan latest_scan_;
  nav_msgs::msg::Path path_;
  bool have_odom_{false};
  bool have_scan_{false};
  bool have_path_{false};
  bool enable_motion_{false};
  std::string odom_topic_{"/demo3_lio/odom"};
  std::string scan_topic_{"/scan"};
  std::string path_topic_{"/demo3/path"};
  std::string path_file_{};
  std::string cmd_vel_topic_{"/cmd_vel"};
  double lookahead_distance_{0.45};
  double linear_speed_{0.20};
  double max_angular_z_{1.2};
  double goal_tolerance_{0.10};
  double obstacle_stop_distance_{0.35};
  double obstacle_sector_rad_{0.55};
  double control_period_s_{0.05};
};
