#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace guandao
{

constexpr double kPi = 3.14159265358979323846;

double wrap(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Waypoint
{
  Pose pose;
  double arc{0.0};
  int direction{0};
};

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class GuandaoNode : public rclcpp::Node
{
public:
  GuandaoNode()
  : Node("guandao_controller")
  {
    path_file_ = declare_parameter<std::string>("path_file", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    vision_topic_ = declare_parameter<std::string>("vision_topic", "/win_detection");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    path_frame_ = declare_parameter<std::string>("path_frame", "odom");
    control_frequency_ = declare_parameter<double>("control_frequency", 30.0);
    max_forward_speed_ = declare_parameter<double>("max_forward_speed", 0.60);
    max_reverse_speed_ = declare_parameter<double>("max_reverse_speed", 0.55);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 2.50);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 2.0);
    max_deceleration_ = declare_parameter<double>("max_deceleration", 0.35);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.12);
    min_lookahead_ = declare_parameter<double>("min_lookahead", 0.45);
    max_lookahead_ = declare_parameter<double>("max_lookahead", 1.05);
    lookahead_speed_gain_ = declare_parameter<double>("lookahead_speed_gain", 0.50);
    max_reverse_lookahead_ = declare_parameter<double>("max_reverse_lookahead", 0.45);
    cross_track_slowdown_distance_ = declare_parameter<double>(
      "cross_track_slowdown_distance", 0.60);
    direction_switch_tolerance_ = declare_parameter<double>(
      "direction_switch_tolerance", 0.16);
    direction_switch_speed_threshold_ = declare_parameter<double>(
      "direction_switch_speed_threshold", 0.04);
    direction_switch_reaction_time_ = declare_parameter<double>(
      "direction_switch_reaction_time", 0.75);

    image_width_ = declare_parameter<double>("image_width", 1920.0);
    image_height_ = declare_parameter<double>("image_height", 1080.0);
    image_center_x_ = declare_parameter<double>("image_center_x", 920.0);
    hat_trigger_ratio_ = declare_parameter<double>("hat_trigger_ratio", 0.22);
    hat_force_trigger_ratio_ = declare_parameter<double>("hat_force_trigger_ratio", 0.50);
    hat_stop_ratio_ = declare_parameter<double>("hat_stop_ratio", 0.80);
    hat_horizontal_limit_ = declare_parameter<double>("hat_horizontal_limit", 500.0);
    hat_width_scale_ = declare_parameter<double>("hat_width_scale", 1.55);
    hat_avoid_target_scale_ = declare_parameter<double>("hat_avoid_target_scale", 1.60);
    hat_direction_bias_px_ = declare_parameter<double>("hat_direction_bias_px", 180.0);
    hat_kp_ = declare_parameter<double>("hat_kp", 0.002);
    hat_ki_ = declare_parameter<double>("hat_ki", 0.00007);
    hat_kd_ = declare_parameter<double>("hat_kd", 0.00007);
    vision_timeout_ = declare_parameter<double>("vision_timeout", 0.35);
    use_vision_when_reversing_ = declare_parameter<bool>("use_vision_when_reversing", false);

    if (!load_path(path_file_)) {
      RCLCPP_FATAL(get_logger(), "无法加载路径文件：%s", path_file_.c_str());
      throw std::runtime_error("guandao path is missing or malformed");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos, std::bind(&GuandaoNode::odom_callback, this, std::placeholders::_1));
    vision_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      vision_topic_, rclcpp::QoS(10),
      std::bind(&GuandaoNode::vision_callback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    state_pub_ = create_publisher<std_msgs::msg::String>("/guandao/state", rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/guandao/tracking_path", rclcpp::QoS(1).transient_local());
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/guandao/lookahead", rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_))),
      std::bind(&GuandaoNode::control_step, this));
    publish_path();
    RCLCPP_INFO(
      get_logger(), "Guandao 已启动：odom=%s vision=%s cmd=%s 路径点=%zu（无雷达）",
      odom_topic_.c_str(), vision_topic_.c_str(), cmd_vel_topic_.c_str(), path_.size());
  }

private:
  bool load_path(const std::string & filename)
  {
    if (filename.empty()) return false;
    std::ifstream stream(filename);
    if (!stream.is_open()) return false;
    std::string line;
    while (std::getline(stream, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream in(line);
      double x, y, z, qx, qy, qz, qw;
      int direction = 0;
      if (!(in >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
      (void)z; (void)qx; (void)qy;
      in >> direction;
      Waypoint point;
      point.pose = {x, y, 2.0 * std::atan2(qz, qw)};
      point.direction = direction == 0 ? 0 : (direction > 0 ? 1 : -1);
      path_.push_back(point);
    }
    if (path_.size() < 2U) return false;
    for (std::size_t i = 0; i < path_.size(); ++i) {
      if (i > 0U) {
        path_[i].arc = path_[i - 1U].arc + std::hypot(
          path_[i].pose.x - path_[i - 1U].pose.x,
          path_[i].pose.y - path_[i - 1U].pose.y);
      }
      if (path_[i].direction == 0) {
        const std::size_t next = std::min(i + 1U, path_.size() - 1U);
        const std::size_t previous = i == path_.size() - 1U ? i - 1U : i;
        const double dx = path_[next].pose.x - path_[previous].pose.x;
        const double dy = path_[next].pose.y - path_[previous].pose.y;
        path_[i].direction = dx * std::cos(path_[i].pose.yaw) +
          dy * std::sin(path_[i].pose.yaw) >= 0.0 ? 1 : -1;
      }
    }
    return true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = quaternion_yaw(msg->pose.pose.orientation);
    measured_velocity_ = msg->twist.twist.linear.x;
    last_odom_received_ = now();
    have_odom_ = true;
  }

  void vision_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    bool found = false;
    double largest_height = 0.0;
    for (const auto & target : msg->targets) {
      if (target.type != "hat") continue;
      for (const auto & roi : target.rois) {
        const double xmin = roi.rect.x_offset;
        const double width = roi.rect.width;
        const double height = roi.rect.height;
        const double xmax = xmin + width;
        if (xmax < image_width_ * 400.0 / 1920.0 ||
          xmin > image_width_ * 1600.0 / 1920.0) continue;
        if (height <= largest_height) continue;
        largest_height = height;
        hat_center_x_ = xmin + 0.5 * width;
        hat_width_ = width;
        found = true;
      }
    }
    visual_triggered_ = false;
    if (!found) {
      visual_height_ratio_ = 0.0;
      return;
    }
    visual_height_ratio_ = largest_height / image_height_;
    const double gate = std::min(hat_horizontal_limit_, hat_width_scale_ * hat_width_);
    visual_triggered_ = visual_height_ratio_ >= hat_force_trigger_ratio_ ||
      (visual_height_ratio_ >= hat_trigger_ratio_ &&
      std::abs(hat_center_x_ - image_center_x_) < gate);
    if (visual_triggered_) {
      const double side = hat_center_x_ > image_center_x_ - hat_direction_bias_px_ ? -1.0 : 1.0;
      visual_error_ = hat_center_x_ + hat_avoid_target_scale_ * side * hat_width_ - image_center_x_;
      last_vision_received_ = now();
    }
  }

  std::size_t segment_end(std::size_t index) const
  {
    std::size_t end = index;
    while (end + 1U < path_.size() && path_[end + 1U].direction == path_[index].direction) ++end;
    return end;
  }

  void update_progress()
  {
    const std::size_t end = std::min(path_.size(), progress_ + 50U);
    double best = std::numeric_limits<double>::infinity();
    std::size_t best_index = progress_;
    for (std::size_t i = progress_; i < end; ++i) {
      const double distance = std::hypot(
        path_[i].pose.x - current_pose_.x, path_[i].pose.y - current_pose_.y);
      if (distance < best) {best = distance; best_index = i;}
    }
    if (best_index >= progress_ || best < 0.15) progress_ = best_index;
  }

  void publish_command(double speed, double yaw_rate)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = speed;
    command.angular.z = yaw_rate;
    cmd_pub_->publish(command);
  }

  void set_state(const std::string & state)
  {
    if (state == last_state_) return;
    last_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "状态：%s", state.c_str());
  }

  void publish_path()
  {
    nav_msgs::msg::Path message;
    message.header.frame_id = path_frame_;
    message.header.stamp = now();
    for (const auto & point : path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = point.pose.x;
      pose.pose.position.y = point.pose.y;
      pose.pose.orientation.z = std::sin(0.5 * point.pose.yaw);
      pose.pose.orientation.w = std::cos(0.5 * point.pose.yaw);
      message.poses.push_back(pose);
    }
    path_pub_->publish(message);
  }

  void publish_lookahead(const Pose & target)
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = path_frame_;
    message.header.stamp = now();
    message.pose.position.x = target.x;
    message.pose.position.y = target.y;
    message.pose.orientation.z = std::sin(0.5 * target.yaw);
    message.pose.orientation.w = std::cos(0.5 * target.yaw);
    lookahead_pub_->publish(message);
  }

  void control_step()
  {
    const rclcpp::Time current_time = now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    if (!(dt > 0.0 && dt < 0.5)) dt = 1.0 / control_frequency_;
    if (!have_odom_ || (current_time - last_odom_received_).seconds() > odom_timeout_) {
      commanded_speed_ = 0.0;
      publish_command(0.0, 0.0);
      set_state("WAIT_ODOM");
      return;
    }
    const double distance_to_goal = std::hypot(
      path_.back().pose.x - current_pose_.x,
      path_.back().pose.y - current_pose_.y);
    if (progress_ + 1U >= path_.size() ||
      (progress_ + 3U >= path_.size() && distance_to_goal <= goal_tolerance_)) {
      publish_command(0.0, 0.0);
      set_state("COMPLETE");
      return;
    }
    update_progress();
    const std::size_t end = segment_end(progress_);
    const int direction = path_[progress_].direction;
    const double distance_to_end = std::hypot(
      path_[end].pose.x - current_pose_.x, path_[end].pose.y - current_pose_.y);
    const bool switching = end + 1U < path_.size() &&
      distance_to_end <= std::max(
        direction_switch_tolerance_,
        std::abs(measured_velocity_) * direction_switch_reaction_time_ +
        measured_velocity_ * measured_velocity_ / (2.0 * std::max(0.1, max_deceleration_)) +
        0.5 * direction_switch_tolerance_);
    if (switching) {
      commanded_speed_ = approach(commanded_speed_, 0.0, max_deceleration_ * dt);
      publish_command(commanded_speed_, 0.0);
      set_state("APPROACH_DIRECTION_SWITCH");
      if (std::abs(commanded_speed_) < direction_switch_speed_threshold_ &&
        std::abs(measured_velocity_) < direction_switch_speed_threshold_) {
        progress_ = end + 1U;
        commanded_speed_ = 0.0;
        set_state("DIRECTION_SWITCH");
      }
      return;
    }

    const double lookahead = std::min(
      direction < 0 ? max_reverse_lookahead_ :
      clamp(min_lookahead_ + lookahead_speed_gain_ * std::abs(measured_velocity_),
      min_lookahead_, max_lookahead_), std::max(0.15, distance_to_end));
    const double target_arc = path_[progress_].arc + lookahead;
    std::size_t target_index = progress_;
    while (target_index + 1U <= end && path_[target_index].arc < target_arc) ++target_index;
    const Pose & target = path_[target_index].pose;
    publish_lookahead(target);
    const double dx = target.x - current_pose_.x;
    const double dy = target.y - current_pose_.y;
    const double c = std::cos(current_pose_.yaw);
    const double s = std::sin(current_pose_.yaw);
    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    double curvature = 2.0 * local_y / std::max(0.01, local_x * local_x + local_y * local_y);
    const double cross_track = std::hypot(
      path_[progress_].pose.x - current_pose_.x,
      path_[progress_].pose.y - current_pose_.y);
    double speed = direction > 0 ? max_forward_speed_ : max_reverse_speed_;
    if (std::abs(curvature) > 1e-4) speed = std::min(
      speed, std::sqrt(2.2 / std::abs(curvature)));
    if (cross_track > cross_track_slowdown_distance_) {
      speed *= clamp(cross_track_slowdown_distance_ / cross_track, 0.25, 1.0);
    }

    const bool visual_fresh = visual_triggered_ &&
      (current_time - last_vision_received_).seconds() <= vision_timeout_ &&
      (direction > 0 || use_vision_when_reversing_);
    if (visual_fresh) {
      const double derivative = (visual_error_ - previous_visual_error_) / std::max(1e-3, dt);
      visual_integral_ = clamp(visual_integral_ + visual_error_ * dt, -2000.0, 2000.0);
      const double visual_yaw = clamp(
        -(hat_kp_ * visual_error_ + hat_ki_ * visual_integral_ + hat_kd_ * derivative),
        -max_yaw_rate_, max_yaw_rate_);
      previous_visual_error_ = visual_error_;
      speed *= visual_height_ratio_ >= hat_stop_ratio_ ? 0.20 :
        (visual_height_ratio_ >= hat_force_trigger_ratio_ ? 0.55 : 0.80);
      curvature = speed > 0.03 ? visual_yaw / speed : curvature;
      set_state("AVOID_VISION");
    } else {
      set_state(direction > 0 ? "TRACK_FORWARD" : "TRACK_REVERSE");
    }
    const double desired_speed = direction * speed;
    const double limit = (std::abs(desired_speed) > std::abs(commanded_speed_) ?
      max_acceleration_ : max_deceleration_) * dt;
    commanded_speed_ = approach(commanded_speed_, desired_speed, limit);
    const double yaw_rate = clamp(commanded_speed_ * curvature, -max_yaw_rate_, max_yaw_rate_);
    publish_command(commanded_speed_, yaw_rate);
  }

  static double approach(double current, double target, double step)
  {
    if (target > current) return std::min(target, current + step);
    return std::max(target, current - step);
  }

  std::string path_file_, odom_topic_, vision_topic_, cmd_vel_topic_, path_frame_;
  double control_frequency_{30.0}, max_forward_speed_{0.60}, max_reverse_speed_{0.55};
  double max_yaw_rate_{2.5}, max_acceleration_{2.0}, max_deceleration_{0.35};
  double odom_timeout_{0.5}, goal_tolerance_{0.12};
  double min_lookahead_{0.45}, max_lookahead_{1.05}, lookahead_speed_gain_{0.50};
  double max_reverse_lookahead_{0.45}, cross_track_slowdown_distance_{0.60};
  double direction_switch_tolerance_{0.16}, direction_switch_speed_threshold_{0.04};
  double direction_switch_reaction_time_{0.75};
  double image_width_{1920.0}, image_height_{1080.0}, image_center_x_{920.0};
  double hat_trigger_ratio_{0.22}, hat_force_trigger_ratio_{0.50}, hat_stop_ratio_{0.80};
  double hat_horizontal_limit_{500.0}, hat_width_scale_{1.55}, hat_avoid_target_scale_{1.60};
  double hat_direction_bias_px_{180.0}, hat_kp_{0.002}, hat_ki_{0.00007}, hat_kd_{0.00007};
  double vision_timeout_{0.35};
  bool use_vision_when_reversing_{false};
  std::vector<Waypoint> path_;
  Pose current_pose_;
  std::size_t progress_{0U};
  bool have_odom_{false}, visual_triggered_{false};
  double measured_velocity_{0.0}, commanded_speed_{0.0};
  double hat_center_x_{0.0}, hat_width_{0.0}, visual_height_ratio_{0.0};
  double visual_error_{0.0}, previous_visual_error_{0.0}, visual_integral_{0.0};
  std::string last_state_;
  rclcpp::Time last_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_vision_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr vision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace guandao

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<guandao::GuandaoNode>());
  rclcpp::shutdown();
  return 0;
}
