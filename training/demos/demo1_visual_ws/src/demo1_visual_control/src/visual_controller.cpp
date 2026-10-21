#include "demo1_visual_control/visual_controller.hpp"

#include <algorithm>

VisualController::VisualController() : Node("demo1_visual_controller") {
  declare_parameter("input_topic", input_topic_);
  declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter("image_width", image_width_);
  declare_parameter("image_height", image_height_);
  declare_parameter("image_center", image_center_);
  declare_parameter("line_confidence_threshold", line_confidence_threshold_);
  declare_parameter("line_kp", line_kp_);
  declare_parameter("line_ki", line_ki_);
  declare_parameter("line_kd", line_kd_);
  declare_parameter("integral_limit", integral_limit_);
  declare_parameter("derivative_alpha", derivative_alpha_);
  declare_parameter("obstacle_trigger_ratio", obstacle_trigger_ratio_);
  declare_parameter("obstacle_clear_ratio", obstacle_clear_ratio_);
  declare_parameter("obstacle_confirm_frames", obstacle_confirm_frames_);
  declare_parameter("obstacle_clear_frames", obstacle_clear_frames_);
  declare_parameter("max_speed", max_speed_);
  declare_parameter("obstacle_speed", obstacle_speed_);
  declare_parameter("lost_hold_speed", lost_hold_speed_);
  declare_parameter("search_angular_z", search_angular_z_);
  declare_parameter("max_angular_z", max_angular_z_);
  declare_parameter("obstacle_angular_z", obstacle_angular_z_);
  declare_parameter("obstacle_memory_s", obstacle_memory_s_);
  declare_parameter("lost_hold_s", lost_hold_s_);
  declare_parameter("lost_search_s", lost_search_s_);
  declare_parameter("command_timeout_s", command_timeout_s_);

  get_parameter("input_topic", input_topic_);
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("image_width", image_width_);
  get_parameter("image_height", image_height_);
  get_parameter("image_center", image_center_);
  get_parameter("line_confidence_threshold", line_confidence_threshold_);
  get_parameter("line_kp", line_kp_);
  get_parameter("line_ki", line_ki_);
  get_parameter("line_kd", line_kd_);
  get_parameter("integral_limit", integral_limit_);
  get_parameter("derivative_alpha", derivative_alpha_);
  get_parameter("obstacle_trigger_ratio", obstacle_trigger_ratio_);
  get_parameter("obstacle_clear_ratio", obstacle_clear_ratio_);
  get_parameter("obstacle_confirm_frames", obstacle_confirm_frames_);
  get_parameter("obstacle_clear_frames", obstacle_clear_frames_);
  get_parameter("max_speed", max_speed_);
  get_parameter("obstacle_speed", obstacle_speed_);
  get_parameter("lost_hold_speed", lost_hold_speed_);
  get_parameter("search_angular_z", search_angular_z_);
  get_parameter("max_angular_z", max_angular_z_);
  get_parameter("obstacle_angular_z", obstacle_angular_z_);
  get_parameter("obstacle_memory_s", obstacle_memory_s_);
  get_parameter("lost_hold_s", lost_hold_s_);
  get_parameter("lost_search_s", lost_search_s_);
  get_parameter("command_timeout_s", command_timeout_s_);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  // 车端 /win_detection 的发布端在录包里是 reliable + volatile。这里用
  // best-effort + volatile 订阅，兼容可靠和尽力而为两种发布端。
  const auto yolo_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                            .best_effort()
                            .durability_volatile();
  detections_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      input_topic_, yolo_qos,
      std::bind(&VisualController::on_detections, this, std::placeholders::_1));
  watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&VisualController::on_watchdog, this));
  RCLCPP_WARN(get_logger(),
              "training skeleton: core vision control is intentionally empty; output=%s",
              cmd_vel_topic_.c_str());
}

Detection VisualController::best_detection(
    const ai_msgs::msg::PerceptionTargets &msg, const std::string &type) const {
  (void)msg;
  (void)type;
  // TODO(学员): 从 msg.targets 中提取 target.type == type 的所有 ROI，
  // 再决定“最大框/最高置信度/离路径最近”哪一种选择对任务更合理。
  return {};
}

void VisualController::on_detections(
    const ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  last_detection_time_ = now();
  const auto line = best_detection(*msg, "line");
  const auto obstacle = best_detection(*msg, "hat");
  if (line.valid()) last_line_time_ = last_detection_time_;

  // 目前只输出零速度，避免半成品在真实车上运动。学员完成 make_command()
  // 后再允许它发布非零 /cmd_vel。
  cmd_pub_->publish(make_command(line, obstacle));
}

void VisualController::on_watchdog() {
  if (last_detection_time_.nanoseconds() == 0) return;
  const double age = (now() - last_detection_time_).seconds();
  if (age > command_timeout_s_) {
    cmd_pub_->publish(lost_line_command(
        (last_line_time_.nanoseconds() == 0)
            ? lost_search_s_ + lost_hold_s_ + 1.0
            : (now() - last_line_time_).seconds()));
  }
}

geometry_msgs::msg::Twist VisualController::make_command(
    const Detection &line, const Detection &obstacle) {
  (void)line;
  (void)obstacle;
  // TODO(学员 1): 计算 line_x - image_center 的误差和稳定 dt。
  // TODO(学员 2): 加入积分限幅、微分滤波、角速度/线速度限幅并记录调参数据。
  // TODO(学员 3): 多框 hat 的确认/清除滞回、绕行方向锁存和回线判断。
  // TODO(学员 4): 用显式状态机连接 TRACK/OBSTACLE/LOST_HOLD/LOST_SEARCH/STOP。
  state_ = State::STOP;
  last_command_ = geometry_msgs::msg::Twist{};
  return last_command_;
}

geometry_msgs::msg::Twist VisualController::lost_line_command(double age_s) {
  (void)age_s;
  // TODO(学员): 依据最后误差设计保持、搜索、反向扫描和重新捕获线路的状态转移。
  state_ = State::STOP;
  last_command_ = geometry_msgs::msg::Twist{};
  return last_command_;
}

void VisualController::reset_pid() {
  // TODO(学员): 在切换状态或检测超时时清理这些历史量。
  integral_error_ = 0.0;
  previous_error_ = 0.0;
  filtered_derivative_ = 0.0;
  previous_pid_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualController>());
  rclcpp::shutdown();
  return 0;
}
