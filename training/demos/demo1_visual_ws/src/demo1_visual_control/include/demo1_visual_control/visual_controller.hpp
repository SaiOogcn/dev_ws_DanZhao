#pragma once

#include <chrono>
#include <functional>
#include <string>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// 教学数据结构：请学员在 best_detection() 中把 ai_msgs 的 ROI 转成它。
struct Detection {
  std::string type;
  float xmin{0.0F};
  float ymin{0.0F};
  float xmax{0.0F};
  float ymax{0.0F};
  float score{0.0F};
  bool valid() const { return score > 0.0F; }
};

class VisualController final : public rclcpp::Node {
public:
  VisualController();

private:
  void on_detections(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
  void on_watchdog();

  // TODO(学员): 遍历 targets/rois，按类别和置信度筛选多个候选框。
  // 需要处理：空数组、低置信度、越界框、多个 line/hat 以及时间戳。
  Detection best_detection(const ai_msgs::msg::PerceptionTargets &msg,
                           const std::string &type) const;

  // TODO(学员): 在这里实现 PID、速度限幅和障碍物状态机。
  geometry_msgs::msg::Twist make_command(const Detection &line,
                                         const Detection &obstacle);

  // TODO(学员): 设计 LOST_HOLD -> LOST_SEARCH -> STOP -> 回线的策略。
  geometry_msgs::msg::Twist lost_line_command(double age_s);

  // TODO(学员): 丢线、重新见线或切换障碍物状态时，正确清理积分和历史误差。
  // 骨架不调用 reset_pid()，调用点要你在状态切换、超时和重新捕获处自己加。
  void reset_pid();

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::string input_topic_{"/win_detection"};
  // 这是车端真实底盘输入。未完成核心逻辑前，代码只发布零速度。
  std::string cmd_vel_topic_{"/cmd_vel"};
  double image_width_{1920.0};
  double image_height_{1080.0};
  double image_center_{920.0};
  double line_confidence_threshold_{0.20};
  double line_kp_{0.0016};
  double line_ki_{0.0};
  double line_kd_{0.0002};
  double integral_limit_{300.0};
  double derivative_alpha_{0.6};
  double obstacle_trigger_ratio_{0.22};
  double obstacle_clear_ratio_{0.16};
  int obstacle_confirm_frames_{3};
  int obstacle_clear_frames_{4};
  double max_speed_{0.35};
  double obstacle_speed_{0.12};
  double lost_hold_speed_{0.08};
  double search_angular_z_{0.45};
  double max_angular_z_{1.8};
  double obstacle_angular_z_{0.55};
  double obstacle_memory_s_{0.35};
  double lost_hold_s_{0.25};
  double lost_search_s_{1.0};
  double command_timeout_s_{0.45};

  enum class State { TRACK, OBSTACLE, LOST_HOLD, LOST_SEARCH, STOP };
  State state_{State::STOP};
  double integral_error_{0.0};
  double previous_error_{0.0};
  double filtered_derivative_{0.0};
  rclcpp::Time previous_pid_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_line_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist last_command_{};
};
