#include "demo1_visual_control/visual_controller.hpp"

#include <algorithm>
#include <cmath>

VisualController::VisualController() : Node("demo1_visual_controller") {
  declare_parameter("image_center", image_center_);
  declare_parameter("line_kp", line_kp_);
  declare_parameter("obstacle_trigger_ratio", obstacle_trigger_ratio_);
  declare_parameter("max_speed", max_speed_);
  declare_parameter("obstacle_speed", obstacle_speed_);
  declare_parameter("max_angular_z", max_angular_z_);
  declare_parameter("obstacle_memory_s", obstacle_memory_s_);
  get_parameter("image_center", image_center_);
  get_parameter("line_kp", line_kp_);
  get_parameter("obstacle_trigger_ratio", obstacle_trigger_ratio_);
  get_parameter("max_speed", max_speed_);
  get_parameter("obstacle_speed", obstacle_speed_);
  get_parameter("max_angular_z", max_angular_z_);
  get_parameter("obstacle_memory_s", obstacle_memory_s_);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  detections_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      "/win_detection", 10,
      std::bind(&VisualController::on_detections, this, std::placeholders::_1));
}

Detection VisualController::best_detection(
    const ai_msgs::msg::PerceptionTargets &msg, const std::string &type) const {
  Detection best;
  for (const auto &target : msg.targets) {
    if (target.type != type) continue;
    for (const auto &roi : target.rois) {
      if (roi.confidence <= best.score) continue;
      best = {type, roi.rect.x_offset, roi.rect.y_offset,
              roi.rect.x_offset + roi.rect.width,
              roi.rect.y_offset + roi.rect.height, roi.confidence};
    }
  }
  return best;
}

void VisualController::on_detections(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  const auto line = best_detection(*msg, "line");
  const auto obstacle = best_detection(*msg, "hat");
  last_detection_time_ = now();
  cmd_pub_->publish(make_command(line, obstacle));
}

geometry_msgs::msg::Twist VisualController::make_command(
    const Detection &line, const Detection &obstacle) {
  geometry_msgs::msg::Twist cmd;
  if (line.score < 0.2F) {
    // TODO(trainee): 设计丢线状态机。至少要有减速、短时保持和停车三种状态。
    return cmd;
  }

  const double line_x = 0.5 * (line.xmin + line.xmax);
  const double error = line_x - image_center_;
  cmd.angular.z = std::clamp(-line_kp_ * error, -max_angular_z_, max_angular_z_);
  cmd.linear.x = max_speed_;

  const double obstacle_height_ratio =
      (obstacle.score > 0.0F) ? (obstacle.ymax - obstacle.ymin) / 1080.0 : 0.0;
  if (obstacle_height_ratio >= obstacle_trigger_ratio_) {
    // TODO(trainee): 根据障碍物中心与巡线中心决定绕行侧，并加入确认帧/滞回。
    cmd.linear.x = obstacle_speed_;
  }
  return cmd;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualController>());
  rclcpp::shutdown();
  return 0;
}
