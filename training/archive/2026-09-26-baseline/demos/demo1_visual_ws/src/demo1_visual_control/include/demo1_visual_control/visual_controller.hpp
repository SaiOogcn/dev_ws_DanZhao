#pragma once

#include <limits>
#include <functional>
#include <string>
#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

struct Detection {
  std::string type;
  float xmin{0.0F};
  float ymin{0.0F};
  float xmax{0.0F};
  float ymax{0.0F};
  float score{0.0F};
};

class VisualController final : public rclcpp::Node {
public:
  VisualController();

private:
  void on_detections(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  Detection best_detection(const ai_msgs::msg::PerceptionTargets &msg,
                           const std::string &type) const;
  geometry_msgs::msg::Twist make_command(const Detection &line,
                                         const Detection &obstacle);

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  double image_center_{920.0};
  double line_kp_{0.0016};
  double obstacle_trigger_ratio_{0.22};
  double max_speed_{0.35};
  double obstacle_speed_{0.12};
  double max_angular_z_{1.8};
  double obstacle_memory_s_{0.35};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
};
