#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{

struct BoundaryDetectResult
{
  bool found{false};
  int x{-1};
  double strength{0.0};
  double left_score{0.0};
  double right_score{0.0};
};

struct RowBoundary
{
  bool found_left{false};
  bool found_right{false};

  int left_boundary_x{-1};   // 左侧外边界：green/blue -> yellow
  int right_boundary_x{-1};  // 右侧边界：yellow -> green/blue，用于估计通道宽度

  double left_strength{0.0};
  double right_strength{0.0};

  double left_side_score{0.0};
  double right_side_score{0.0};
};

struct TrackPoint
{
  cv::Point2f boundary_point;
  cv::Point2f center_point;
  float weight{1.0F};
};

inline int clamp_int(int x, int lo, int hi)
{
  return std::max(lo, std::min(x, hi));
}

template <typename T>
T median_value(std::vector<T> values)
{
  if (values.empty()) {
    return T{};
  }

  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;

  if (values.size() % 2 == 1) {
    return values[mid];
  }

  return static_cast<T>((values[mid - 1] + values[mid]) / 2);
}

double weighted_average(
  const std::vector<double>& values,
  const std::vector<double>& weights)
{
  double sum = 0.0;
  double weight_sum = 0.0;

  for (size_t i = 0; i < values.size(); ++i) {
    sum += values[i] * weights[i];
    weight_sum += weights[i];
  }

  return sum / std::max(1e-6, weight_sum);
}

class PixelPID
{
public:
  PixelPID(double kp, double ki, double kd, double low, double high)
  : kp_(kp), ki_(ki), kd_(kd), low_(low), high_(high)
  {
    last_time_ = std::chrono::steady_clock::now();
  }

  void set_gains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void set_limits(double low, double high)
  {
    low_ = low;
    high_ = high;
  }

  void reset()
  {
    prev_err_ = 0.0;
    integral_ = 0.0;
    last_time_ = std::chrono::steady_clock::now();
  }

  double update(double err)
  {
    const auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    dt = std::clamp(dt, 1e-4, 0.3);

    integral_ += err * dt;
    integral_ = std::clamp(integral_, -3000.0, 3000.0);

    const double derivative = (err - prev_err_) / dt;
    const double out = kp_ * err + ki_ * integral_ + kd_ * derivative;

    prev_err_ = err;
    last_time_ = now;

    return std::clamp(out, low_, high_);
  }

private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};

  double low_{-1.0};
  double high_{1.0};

  double prev_err_{0.0};
  double integral_{0.0};

  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace


class LeftBoundaryOffsetFollower : public rclcpp::Node
{
public:
  LeftBoundaryOffsetFollower()
  : Node("left_boundary_offset_follower")
  {
    declare_parameters();
    load_parameters();

    pid_.set_gains(pid_kp_, pid_ki_, pid_kd_);
    pid_.set_limits(-max_wz_, max_wz_);

    auto image_qos = rclcpp::SensorDataQoS();
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      cmd_qos);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      image_qos,
      std::bind(&LeftBoundaryOffsetFollower::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "左外边界平移巡线节点启动 | 检测方式=归一化RGB颜色跳变 | image_topic=%s cmd_vel_topic=%s ROI=%.0f%%~%.0f%%",
      image_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      roi_top_ratio_ * 100.0,
      roi_bottom_ratio_ * 100.0);
  }

  ~LeftBoundaryOffsetFollower() override = default;

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("image_topic", image_topic_);
    declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
    declare_parameter<bool>("show_debug", show_debug_);

    declare_parameter<double>("roi_top_ratio", roi_top_ratio_);
    declare_parameter<double>("roi_bottom_ratio", roi_bottom_ratio_);
    declare_parameter<int>("scan_line_count", scan_line_count_);

    declare_parameter<double>("target_x_ratio", target_x_ratio_);

    declare_parameter<double>("lane_half_width_px", lane_half_width_px_);
    declare_parameter<double>("min_lane_half_width_px", min_lane_half_width_px_);
    declare_parameter<double>("max_lane_half_width_px", max_lane_half_width_px_);
    declare_parameter<double>("lane_width_ema_alpha", lane_width_ema_alpha_);

    declare_parameter<int>("min_valid_points", min_valid_points_);

    declare_parameter<int>("color_jump_window", color_jump_window_);
    declare_parameter<double>("min_color_jump", min_color_jump_);
    declare_parameter<int>("right_boundary_min_gap_px", right_boundary_min_gap_px_);

    declare_parameter<double>("linear_speed", linear_speed_);
    declare_parameter<double>("min_linear_speed", min_linear_speed_);
    declare_parameter<double>("lost_linear_speed", lost_linear_speed_);
    declare_parameter<double>("lost_angular_speed", lost_angular_speed_);

    declare_parameter<double>("pid_kp", pid_kp_);
    declare_parameter<double>("pid_ki", pid_ki_);
    declare_parameter<double>("pid_kd", pid_kd_);
    declare_parameter<double>("max_wz", max_wz_);

    declare_parameter<double>("heading_weight", heading_weight_);
    declare_parameter<double>("error_deadband_px", error_deadband_px_);
    declare_parameter<double>("wz_smooth_alpha", wz_smooth_alpha_);
    declare_parameter<double>("curve_slowdown_ratio", curve_slowdown_ratio_);
  }

  void load_parameters()
  {
    image_topic_ = get_parameter("image_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    show_debug_ = get_parameter("show_debug").as_bool();

    roi_top_ratio_ = get_parameter("roi_top_ratio").as_double();
    roi_bottom_ratio_ = get_parameter("roi_bottom_ratio").as_double();
    scan_line_count_ = get_parameter("scan_line_count").as_int();

    target_x_ratio_ = get_parameter("target_x_ratio").as_double();

    lane_half_width_px_ = get_parameter("lane_half_width_px").as_double();
    min_lane_half_width_px_ = get_parameter("min_lane_half_width_px").as_double();
    max_lane_half_width_px_ = get_parameter("max_lane_half_width_px").as_double();
    lane_width_ema_alpha_ = get_parameter("lane_width_ema_alpha").as_double();

    min_valid_points_ = get_parameter("min_valid_points").as_int();

    color_jump_window_ = get_parameter("color_jump_window").as_int();
    min_color_jump_ = get_parameter("min_color_jump").as_double();
    right_boundary_min_gap_px_ = get_parameter("right_boundary_min_gap_px").as_int();

    linear_speed_ = get_parameter("linear_speed").as_double();
    min_linear_speed_ = get_parameter("min_linear_speed").as_double();
    lost_linear_speed_ = get_parameter("lost_linear_speed").as_double();
    lost_angular_speed_ = get_parameter("lost_angular_speed").as_double();

    pid_kp_ = get_parameter("pid_kp").as_double();
    pid_ki_ = get_parameter("pid_ki").as_double();
    pid_kd_ = get_parameter("pid_kd").as_double();
    max_wz_ = get_parameter("max_wz").as_double();

    heading_weight_ = get_parameter("heading_weight").as_double();
    error_deadband_px_ = get_parameter("error_deadband_px").as_double();
    wz_smooth_alpha_ = get_parameter("wz_smooth_alpha").as_double();
    curve_slowdown_ratio_ = get_parameter("curve_slowdown_ratio").as_double();
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat bgr;

    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败：%s", e.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    const int img_w = bgr.cols;
    const int img_h = bgr.rows;

    const int roi_top = clamp_int(
      static_cast<int>(img_h * roi_top_ratio_),
      0,
      img_h - 2);

    const int roi_bottom = clamp_int(
      static_cast<int>(img_h * roi_bottom_ratio_),
      roi_top + 1,
      img_h - 1);

    cv::Rect roi_rect(0, roi_top, img_w, roi_bottom - roi_top);
    cv::Mat roi = bgr(roi_rect);

    cv::Mat score = build_color_score_image(roi);

    std::vector<RowBoundary> row_boundaries;
    std::vector<int> row_y_values;
    std::vector<double> observed_widths;
    std::vector<double> boundary_strengths;

    row_boundaries.reserve(scan_line_count_);
    row_y_values.reserve(scan_line_count_);
    observed_widths.reserve(scan_line_count_);
    boundary_strengths.reserve(scan_line_count_);

    for (int i = 0; i < scan_line_count_; ++i) {
      const double ratio =
        0.03 + 0.94 * static_cast<double>(i) /
        std::max(1, scan_line_count_ - 1);

      const int y = clamp_int(
        static_cast<int>(ratio * roi.rows),
        0,
        roi.rows - 1);

      RowBoundary rb = analyze_row_by_score(score, y);

      if (rb.found_left) {
        row_boundaries.push_back(rb);
        row_y_values.push_back(y);
        boundary_strengths.push_back(rb.left_strength);

        if (rb.found_right && rb.right_boundary_x > rb.left_boundary_x) {
          const double width =
            static_cast<double>(rb.right_boundary_x - rb.left_boundary_x);

          observed_widths.push_back(width);
        }
      }
    }

    update_lane_half_width(observed_widths);

    std::vector<TrackPoint> track_points;
    track_points.reserve(row_boundaries.size());

    for (size_t i = 0; i < row_boundaries.size(); ++i) {
      const RowBoundary& rb = row_boundaries[i];
      const int y = row_y_values[i];

      double half_width = current_lane_half_width_px_;

      if (rb.found_right && rb.right_boundary_x > rb.left_boundary_x) {
        half_width =
          0.5 * static_cast<double>(rb.right_boundary_x - rb.left_boundary_x);

        half_width = std::clamp(
          half_width,
          min_lane_half_width_px_,
          max_lane_half_width_px_);
      }

      const double center_x =
        static_cast<double>(rb.left_boundary_x) + half_width;

      if (center_x < 0.0 || center_x >= static_cast<double>(roi.cols)) {
        continue;
      }

      const float y_norm =
        static_cast<float>(y) / std::max(1.0F, static_cast<float>(roi.rows));

      TrackPoint tp;
      tp.boundary_point = cv::Point2f(
        static_cast<float>(rb.left_boundary_x),
        static_cast<float>(y));

      tp.center_point = cv::Point2f(
        static_cast<float>(center_x),
        static_cast<float>(y));

      // 下方点距离小车更近，权重大一些。
      tp.weight = 0.3F + 1.7F * y_norm * y_norm;

      track_points.push_back(tp);
    }

    geometry_msgs::msg::Twist cmd = compute_cmd(track_points, roi.cols);

    cmd_pub_->publish(cmd);

    if (show_debug_) {
      show_debug_log(
        roi,
        score,
        row_boundaries,
        observed_widths,
        boundary_strengths,
        track_points,
        cmd);
    }
  }

  cv::Mat build_color_score_image(const cv::Mat& bgr) const
  {
    cv::Mat blurred;
    cv::GaussianBlur(bgr, blurred, cv::Size(5, 5), 0);

    cv::Mat score(blurred.rows, blurred.cols, CV_32FC1);

    for (int y = 0; y < blurred.rows; ++y) {
      for (int x = 0; x < blurred.cols; ++x) {
        const cv::Vec3b pix = blurred.at<cv::Vec3b>(y, x);

        const float b = static_cast<float>(pix[0]);
        const float g = static_cast<float>(pix[1]);
        const float r = static_cast<float>(pix[2]);

        const float sum = r + g + b + 1.0F;

        const float r_norm = r / sum;
        const float b_norm = b / sum;

        // 黄区通常 R 比 B 更强，绿/蓝区通常 B 或 G 更强。
        // 使用归一化后的 R-B 可以减弱整体亮度变化影响。
        score.at<float>(y, x) = r_norm - b_norm;
      }
    }

    return score;
  }

  BoundaryDetectResult find_green_to_yellow_boundary_on_row(
    const cv::Mat& score,
    int y,
    int search_left,
    int search_right) const
  {
    BoundaryDetectResult result;

    search_left = clamp_int(search_left, 0, score.cols - 2);
    search_right = clamp_int(search_right, search_left + 1, score.cols - 1);

    const int half_window =
      std::max(2, color_jump_window_ / 2);

    double best_jump = min_color_jump_;
    int best_x = -1;
    double best_left_score = 0.0;
    double best_right_score = 0.0;

    for (int x = search_left + half_window;
         x < search_right - half_window;
         ++x) {
      double left_sum = 0.0;
      double right_sum = 0.0;

      for (int k = 1; k <= half_window; ++k) {
        left_sum += static_cast<double>(score.at<float>(y, x - k));
        right_sum += static_cast<double>(score.at<float>(y, x + k));
      }

      const double left_avg =
        left_sum / static_cast<double>(half_window);

      const double right_avg =
        right_sum / static_cast<double>(half_window);

      // green/blue -> yellow 时，score 应该从低变高。
      const double jump = right_avg - left_avg;

      if (jump > best_jump) {
        best_jump = jump;
        best_x = x;
        best_left_score = left_avg;
        best_right_score = right_avg;
      }
    }

    if (best_x >= 0) {
      result.found = true;
      result.x = best_x;
      result.strength = best_jump;
      result.left_score = best_left_score;
      result.right_score = best_right_score;
    }

    return result;
  }

  BoundaryDetectResult find_yellow_to_green_boundary_on_row(
    const cv::Mat& score,
    int y,
    int search_left,
    int search_right) const
  {
    BoundaryDetectResult result;

    search_left = clamp_int(search_left, 0, score.cols - 2);
    search_right = clamp_int(search_right, search_left + 1, score.cols - 1);

    const int half_window =
      std::max(2, color_jump_window_ / 2);

    double best_jump = min_color_jump_;
    int best_x = -1;
    double best_left_score = 0.0;
    double best_right_score = 0.0;

    for (int x = search_left + half_window;
         x < search_right - half_window;
         ++x) {
      double left_sum = 0.0;
      double right_sum = 0.0;

      for (int k = 1; k <= half_window; ++k) {
        left_sum += static_cast<double>(score.at<float>(y, x - k));
        right_sum += static_cast<double>(score.at<float>(y, x + k));
      }

      const double left_avg =
        left_sum / static_cast<double>(half_window);

      const double right_avg =
        right_sum / static_cast<double>(half_window);

      // yellow -> green/blue 时，score 应该从高变低。
      const double jump = left_avg - right_avg;

      if (jump > best_jump) {
        best_jump = jump;
        best_x = x;
        best_left_score = left_avg;
        best_right_score = right_avg;
      }
    }

    if (best_x >= 0) {
      result.found = true;
      result.x = best_x;
      result.strength = best_jump;
      result.left_score = best_left_score;
      result.right_score = best_right_score;
    }

    return result;
  }

  RowBoundary analyze_row_by_score(
    const cv::Mat& score,
    int y) const
  {
    RowBoundary result;

    BoundaryDetectResult left =
      find_green_to_yellow_boundary_on_row(
        score,
        y,
        0,
        score.cols - 1);

    if (!left.found) {
      return result;
    }

    result.found_left = true;
    result.left_boundary_x = left.x;
    result.left_strength = left.strength;
    result.left_side_score = left.left_score;
    result.right_side_score = left.right_score;

    // 尝试找右边界，用于估计通道宽度。
    // 没找到右边界也没关系，控制仍然使用固定 lane_half_width_px。
    const int right_search_left =
      left.x + std::max(right_boundary_min_gap_px_,
                        static_cast<int>(min_lane_half_width_px_));

    if (right_search_left < score.cols - 1) {
      BoundaryDetectResult right =
        find_yellow_to_green_boundary_on_row(
          score,
          y,
          right_search_left,
          score.cols - 1);

      if (right.found) {
        const double width =
          static_cast<double>(right.x - left.x);

        const double min_width = 2.0 * min_lane_half_width_px_;
        const double max_width = 2.0 * max_lane_half_width_px_;

        if (width >= min_width && width <= max_width) {
          result.found_right = true;
          result.right_boundary_x = right.x;
          result.right_strength = right.strength;
        }
      }
    }

    return result;
  }

  void update_lane_half_width(const std::vector<double>& observed_widths)
  {
    if (observed_widths.empty()) {
      current_lane_half_width_px_ = lane_half_width_px_;
      return;
    }

    const double observed_half =
      0.5 * median_value(observed_widths);

    const double clamped_half = std::clamp(
      observed_half,
      min_lane_half_width_px_,
      max_lane_half_width_px_);

    if (!lane_width_initialized_) {
      current_lane_half_width_px_ = clamped_half;
      lane_width_initialized_ = true;
      return;
    }

    current_lane_half_width_px_ =
      (1.0 - lane_width_ema_alpha_) * current_lane_half_width_px_ +
      lane_width_ema_alpha_ * clamped_half;

    current_lane_half_width_px_ = std::clamp(
      current_lane_half_width_px_,
      min_lane_half_width_px_,
      max_lane_half_width_px_);
  }

  geometry_msgs::msg::Twist compute_cmd(
    const std::vector<TrackPoint>& track_points,
    int roi_width)
  {
    geometry_msgs::msg::Twist cmd{};

    if (static_cast<int>(track_points.size()) < min_valid_points_) {
      lost_count_++;

      cmd.linear.x = lost_linear_speed_;

      // 丢线时小角速度向左找线。
      // 如果你的车实际表现为向右找线，请改成 -lost_angular_speed_。
      cmd.angular.z = lost_angular_speed_;

      last_wz_ = cmd.angular.z;
      pid_.reset();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        800,
        "丢失左外边界 | 有效点=%zu 当前半通道宽度=%.1f min_color_jump=%.3f color_jump_window=%d",
        track_points.size(),
        current_lane_half_width_px_,
        min_color_jump_,
        color_jump_window_);

      return cmd;
    }

    lost_count_ = 0;

    std::vector<double> center_x_values;
    std::vector<double> weights;

    center_x_values.reserve(track_points.size());
    weights.reserve(track_points.size());

    for (const auto& p : track_points) {
      center_x_values.push_back(static_cast<double>(p.center_point.x));
      weights.push_back(static_cast<double>(p.weight));
    }

    const double fused_center_x =
      weighted_average(center_x_values, weights);

    const double target_x =
      std::clamp(target_x_ratio_, 0.1, 0.9) * static_cast<double>(roi_width);

    const double far_x = center_x_values.front();
    const double near_x = center_x_values.back();

    const double heading_error_px = near_x - far_x;

    // 图像中轨迹点偏左：target_x - fused_center_x 为正，小车应左转。
    // ROS 中 angular.z > 0 通常表示左转。
    const double position_error_px = target_x - fused_center_x;

    double control_error_px =
      position_error_px + heading_weight_ * heading_error_px;

    if (std::abs(position_error_px) < error_deadband_px_ &&
        std::abs(heading_error_px) < error_deadband_px_) {
      control_error_px = 0.0;
    }

    double wz = pid_.update(control_error_px);

    wz =
      wz_smooth_alpha_ * wz +
      (1.0 - wz_smooth_alpha_) * last_wz_;

    wz = std::clamp(wz, -max_wz_, max_wz_);

    const double turn_ratio =
      std::min(1.0, std::abs(wz) / std::max(1e-6, max_wz_));

    const double speed =
      linear_speed_ *
      (1.0 - curve_slowdown_ratio_ * turn_ratio);

    cmd.linear.x = std::clamp(
      speed,
      min_linear_speed_,
      linear_speed_);

    cmd.angular.z = wz;

    last_wz_ = wz;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      800,
      "左边界跳变巡线 | 点数=%zu 目标中心=%.1f 轨迹中心=%.1f 位置误差=%.1f 航向误差=%.1f 控制误差=%.1f vx=%.3f wz=%.3f 半宽=%.1f",
      track_points.size(),
      target_x,
      fused_center_x,
      position_error_px,
      heading_error_px,
      control_error_px,
      cmd.linear.x,
      cmd.angular.z,
      current_lane_half_width_px_);

    return cmd;
  }

  void show_debug_log(
    const cv::Mat& roi,
    const cv::Mat& score,
    const std::vector<RowBoundary>& row_boundaries,
    const std::vector<double>& observed_widths,
    const std::vector<double>& boundary_strengths,
    const std::vector<TrackPoint>& track_points,
    const geometry_msgs::msg::Twist& cmd)
  {
    double score_min = 0.0;
    double score_max = 0.0;
    cv::minMaxLoc(score, &score_min, &score_max);

    cv::Scalar mean_score;
    cv::Scalar std_score;
    cv::meanStdDev(score, mean_score, std_score);

    const int left_boundary_rows =
      static_cast<int>(row_boundaries.size());

    int right_boundary_rows = 0;
    for (const auto& rb : row_boundaries) {
      if (rb.found_right) {
        right_boundary_rows++;
      }
    }

    const int valid_points =
      static_cast<int>(track_points.size());

    const double target_x =
      std::clamp(target_x_ratio_, 0.1, 0.9) *
      static_cast<double>(roi.cols);

    double fused_center_x = -1.0;
    double position_error_px = 0.0;
    double heading_error_px = 0.0;

    double left_far = -1.0;
    double left_near = -1.0;
    double center_far = -1.0;
    double center_near = -1.0;

    if (!track_points.empty()) {
      double weighted_sum = 0.0;
      double weight_sum = 0.0;

      for (const auto& p : track_points) {
        weighted_sum += static_cast<double>(p.center_point.x) *
                        static_cast<double>(p.weight);
        weight_sum += static_cast<double>(p.weight);
      }

      fused_center_x = weighted_sum / std::max(1e-6, weight_sum);
      position_error_px = target_x - fused_center_x;

      left_far = track_points.front().boundary_point.x;
      left_near = track_points.back().boundary_point.x;

      center_far = track_points.front().center_point.x;
      center_near = track_points.back().center_point.x;

      heading_error_px = center_near - center_far;
    }

    double median_width = -1.0;
    if (!observed_widths.empty()) {
      median_width = median_value(observed_widths);
    }

    double avg_strength = -1.0;
    double max_strength = -1.0;

    if (!boundary_strengths.empty()) {
      double sum = 0.0;
      for (double s : boundary_strengths) {
        sum += s;
      }

      avg_strength =
        sum / static_cast<double>(boundary_strengths.size());

      max_strength =
        *std::max_element(boundary_strengths.begin(), boundary_strengths.end());
    }

    std::string advice;

    if (valid_points < min_valid_points_) {
      if (left_boundary_rows == 0) {
        advice =
          "完全没有找到绿黄跳变边界：优先降低 min_color_jump，例如 0.035 -> 0.025；同时确认 ROI 是否覆盖边界";
      } else if (avg_strength > 0.0 && avg_strength < min_color_jump_ * 1.3) {
        advice =
          "边界跳变很弱：可以降低 min_color_jump，或增大 color_jump_window 增强抗噪";
      } else {
        advice =
          "找到了一些边界但有效轨迹点不足：可适当减小 min_valid_points，或检查 lane_half_width_px 是否导致中心点越界";
      }
    } else {
      if (std::abs(position_error_px) > 80.0) {
        if (position_error_px > 0.0) {
          advice =
            "轨迹中心偏左：建议增大 lane_half_width_px，让边界平移后的中心线向右移动";
        } else {
          advice =
            "轨迹中心偏右：建议减小 lane_half_width_px，让边界平移后的中心线向左移动";
        }
      } else if (std::abs(heading_error_px) > 120.0) {
        advice =
          "轨迹倾斜明显：可能在弯道或边界跳变点不稳定，建议降低 linear_speed，或适当增大 heading_weight";
      } else if (std::abs(cmd.angular.z) > 0.8 * max_wz_) {
        advice =
          "角速度接近上限：建议降低 pid_kp 或 linear_speed，避免转向过猛";
      } else if (observed_widths.empty()) {
        advice =
          "只检测到左边界，当前使用固定 lane_half_width_px；主要根据 position_error 调半宽";
      } else {
        advice =
          "状态基本正常：边界跳变检测有效，轨迹中心接近目标中心";
      }
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    oss
      << "\n========== 边界跳变巡线调试 ==========\n"
      << "ROI尺寸: " << roi.cols << " x " << roi.rows << "\n"
      << "score统计: min=" << score_min
      << ", max=" << score_max
      << ", mean=" << mean_score[0]
      << ", std=" << std_score[0] << "\n"
      << "跳变参数: min_color_jump=" << min_color_jump_
      << ", color_jump_window=" << color_jump_window_ << "\n"
      << "扫描状态: 扫描线数=" << scan_line_count_
      << ", 左边界行数=" << left_boundary_rows
      << ", 右边界行数=" << right_boundary_rows
      << ", 有效轨迹点=" << valid_points
      << ", 最少要求=" << min_valid_points_ << "\n"
      << "跳变强度: 平均=" << avg_strength
      << ", 最大=" << max_strength << "\n"
      << "宽度估计: 宽度样本数=" << observed_widths.size()
      << ", 中位宽=" << median_width
      << ", 当前半宽=" << current_lane_half_width_px_
      << ", 默认半宽=" << lane_half_width_px_ << "\n"
      << "边界位置: left_far=" << left_far
      << ", left_near=" << left_near << "\n"
      << "中心轨迹: center_far=" << center_far
      << ", center_near=" << center_near
      << ", fused_center=" << fused_center_x
      << ", target_x=" << target_x << "\n"
      << "误差状态: position_error=" << position_error_px
      << ", heading_error=" << heading_error_px << "\n"
      << "速度输出: vx=" << cmd.linear.x
      << ", wz=" << cmd.angular.z << "\n"
      << "调参建议: " << advice << "\n"
      << "====================================";

    const std::string debug_text = oss.str();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "%s",
      debug_text.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::string image_topic_{"/image_new"};
  std::string cmd_vel_topic_{"cmd_vel"};

  bool show_debug_{true};

  // 建议不要只看很底部，否则提前看不到弯道。
  double roi_top_ratio_{0.35};
  double roi_bottom_ratio_{0.98};
  int scan_line_count_{36};

  // 目标轨迹应该落在图像中间。
  double target_x_ratio_{0.50};

  // 左边界向右平移的默认距离。
  // 找不到右边界时，直接使用这个半宽。
  double lane_half_width_px_{260.0};
  double min_lane_half_width_px_{120.0};
  double max_lane_half_width_px_{520.0};
  double lane_width_ema_alpha_{0.20};

  int min_valid_points_{6};

  // 新的颜色跳变检测参数。
  int color_jump_window_{18};
  double min_color_jump_{0.035};
  int right_boundary_min_gap_px_{120};

  double linear_speed_{0.35};
  double min_linear_speed_{0.12};
  double lost_linear_speed_{0.05};
  double lost_angular_speed_{0.25};

  double pid_kp_{0.0030};
  double pid_ki_{0.0};
  double pid_kd_{0.0008};
  double max_wz_{1.20};

  double heading_weight_{0.18};
  double error_deadband_px_{5.0};
  double wz_smooth_alpha_{0.35};
  double curve_slowdown_ratio_{0.65};

  double current_lane_half_width_px_{260.0};
  bool lane_width_initialized_{false};

  double last_wz_{0.0};
  int lost_count_{0};

  PixelPID pid_{0.0030, 0.0, 0.0008, -1.20, 1.20};
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeftBoundaryOffsetFollower>());
  rclcpp::shutdown();
  return 0;
}