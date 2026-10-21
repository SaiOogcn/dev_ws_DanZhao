#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ai_msgs/msg/perception_targets.hpp> // 引入避障话题消息
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <array>
#include <vector>
#include <string>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <atomic>
#include <iostream>
#include <thread>

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#endif

namespace
{
constexpr int kNumScanLines = 30;

// ROI 内部扫描高度比例
constexpr std::array<float, kNumScanLines> kScanRatios{
  0.02f, 0.05f, 0.08f, 0.12f, 0.15f,
  0.18f, 0.21f, 0.25f, 0.28f, 0.31f,
  0.34f, 0.38f, 0.41f, 0.44f, 0.47f,
  0.51f, 0.54f, 0.57f, 0.60f, 0.64f,
  0.67f, 0.70f, 0.73f, 0.77f, 0.80f,
  0.83f, 0.86f, 0.90f, 0.94f, 0.98f
};

// 越靠近 ROI 下方，权重越大
constexpr std::array<float, kNumScanLines> kScanWeights{
  0.020f, 0.022f, 0.024f, 0.026f, 0.028f,
  0.030f, 0.032f, 0.034f, 0.036f, 0.038f,
  0.040f, 0.042f, 0.044f, 0.046f, 0.048f,
  0.050f, 0.052f, 0.054f, 0.056f, 0.058f,
  0.060f, 0.063f, 0.066f, 0.069f, 0.072f,
  0.075f, 0.078f, 0.081f, 0.084f, 0.087f
};

inline int clamp_int(int x, int lo, int hi)
{
  return std::max(lo, std::min(x, hi));
}

enum class RowDecision { NONE = 0, STRAIGHT, TURN_LEFT, TURN_RIGHT };
enum ColorId { COLOR_NONE = 0, COLOR_YELLOW = 1, COLOR_GREEN = 2 };

struct ColorRun
{
  int color{COLOR_NONE};
  int left{0};
  int right{0};
  int len() const { return right - left + 1; }
};

struct RowResult
{
  RowDecision decision{RowDecision::NONE};
  int left_x{-1};
  int right_x{-1};
  int center_x{-1};
};

/* Yolo检测框结构 */
struct YoloRect {
  int id;
  int xmin;
  int ymin;
  int xmax;
  int ymax;
  float confidence;
  std::string type;
};

class PixelPID
{
public:
  PixelPID(double kp, double ki, double kd, double low, double high)
  : kp_(kp), ki_(ki), kd_(kd), low_(low), high_(high),
    prev_err_(0.0), integral_(0.0),
    last_time_(std::chrono::high_resolution_clock::now()) {}

  void set_gains(double kp, double ki, double kd) { kp_ = kp; ki_ = ki; kd_ = kd; }
  void set_limits(double low, double high) { low_ = low; high_ = high; }

  void reset() {
    prev_err_ = 0.0;
    integral_ = 0.0;
    last_time_ = std::chrono::high_resolution_clock::now();
  }

  double update(double err) {
    auto now = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(now - last_time_).count();
    dt = std::clamp(dt, 1e-4, 0.3);

    integral_ += err * dt;
    const double derivative = (err - prev_err_) / dt;
    const double out = kp_ * err + ki_ * integral_ + kd_ * derivative;

    prev_err_ = err;
    last_time_ = now;
    return std::clamp(out, low_, high_);
  }

private:
  double kp_, ki_, kd_, low_, high_, prev_err_, integral_;
  std::chrono::high_resolution_clock::time_point last_time_;
};

static double weighted_average(const std::vector<double>& values, const std::vector<double>& weights)
{
  double ws = 0.0;
  double wsum = 0.0;

  for (size_t i = 0; i < values.size(); ++i) {
    ws += values[i] * weights[i];
    wsum += weights[i];
  }

  return ws / std::max(1e-6, wsum);
}

void wait_for_any_key()
{
  std::cout << "请按下任意键继续..." << std::flush;

#ifndef _WIN32
  if (isatty(STDIN_FILENO)) {
    termios oldt{}, newt{};
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char ch = 0;
    (void)read(STDIN_FILENO, &ch, 1);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    std::cout << std::endl;
  } else {
    std::cin.get();
    std::cout << std::endl;
  }
#else
  std::cin.get();
  std::cout << std::endl;
#endif
}
}  // namespace

class LabTrackLaneNode : public rclcpp::Node
{
public:
  LabTrackLaneNode() : Node("lab_track_lane_node")
  {
    declare_parameter<std::string>("image_topic", image_topic_);
    declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
    declare_parameter<std::string>("stop_command_topic", stop_command_topic_);
    declare_parameter<std::string>("direct_start_topic", direct_start_topic_);
    declare_parameter<int>("second_line_confirm_frames", second_line_confirm_frames_);
    declare_parameter<double>("second_line_min_confidence", second_line_min_confidence_);
    declare_parameter<double>("second_line_min_area_ratio", second_line_min_area_ratio_);
    declare_parameter<bool>("show_debug", show_debug_);
    declare_parameter<bool>("track_clockwise", track_clockwise_);

    declare_parameter<double>("target_center_px", target_center_px_param_);
    declare_parameter<double>("target_x_ratio", target_x_ratio_);
    declare_parameter<double>("roi_top_ratio", static_cast<double>(roi_top_ratio_));
    declare_parameter<double>("roi_bottom_ratio", static_cast<double>(roi_bottom_ratio_));
    declare_parameter<int>("control_hz", control_hz_);

    declare_parameter<double>("linear_speed", linear_speed_);
    declare_parameter<double>("cautious_linear_speed", cautious_linear_speed_);
    declare_parameter<double>("lost_speed", lost_speed_);
    declare_parameter<double>("fixed_turn_speed", fixed_turn_speed_);
    declare_parameter<double>("fixed_turn_wz", fixed_turn_wz_);

    declare_parameter<double>("pid_kp", pid_kp_);
    declare_parameter<double>("pid_ki", pid_ki_);
    declare_parameter<double>("pid_kd", pid_kd_);
    declare_parameter<double>("max_wz", max_wz_);
    declare_parameter<double>("straight_max_wz", straight_max_wz_);

    declare_parameter<double>("heading_weight", heading_weight_);
    declare_parameter<double>("error_deadband_px", error_deadband_px_);
    declare_parameter<double>("wz_smooth_alpha", wz_smooth_alpha_);

    declare_parameter<int>("straight_enter_min_lines", straight_enter_min_lines_);
    declare_parameter<int>("min_valid_lines", min_valid_lines_);
    declare_parameter<int>("turn_detect_min_lines", turn_detect_min_lines_);
    declare_parameter<int>("turn_confirm_frames", turn_confirm_frames_);
    declare_parameter<int>("lost_line_limit", lost_line_limit_);
    declare_parameter<int>("min_color_run", min_color_run_);

    // CIELab 参数声明
    declare_parameter<int>("yellow_l_low", yellow_l_low_);
    declare_parameter<int>("yellow_l_high", yellow_l_high_);
    declare_parameter<int>("yellow_a_low", yellow_a_low_);
    declare_parameter<int>("yellow_a_high", yellow_a_high_);
    declare_parameter<int>("yellow_b_low", yellow_b_low_);
    declare_parameter<int>("yellow_b_high", yellow_b_high_);

    declare_parameter<int>("green_l_low", green_l_low_);
    declare_parameter<int>("green_l_high", green_l_high_);
    declare_parameter<int>("green_a_low", green_a_low_);
    declare_parameter<int>("green_a_high", green_a_high_);
    declare_parameter<int>("green_b_low", green_b_low_);
    declare_parameter<int>("green_b_high", green_b_high_);

    image_topic_ = get_parameter("image_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    stop_command_topic_ = get_parameter("stop_command_topic").as_string();
    direct_start_topic_ = get_parameter("direct_start_topic").as_string();
    second_line_confirm_frames_ = get_parameter("second_line_confirm_frames").as_int();
    second_line_min_confidence_ = get_parameter("second_line_min_confidence").as_double();
    second_line_min_area_ratio_ = get_parameter("second_line_min_area_ratio").as_double();
    show_debug_ = get_parameter("show_debug").as_bool();
    track_clockwise_ = get_parameter("track_clockwise").as_bool();

    target_center_px_param_ = get_parameter("target_center_px").as_double();
    target_x_ratio_ = get_parameter("target_x_ratio").as_double();
    roi_top_ratio_ = static_cast<float>(get_parameter("roi_top_ratio").as_double());
    roi_bottom_ratio_ = static_cast<float>(get_parameter("roi_bottom_ratio").as_double());
    control_hz_ = get_parameter("control_hz").as_int();

    linear_speed_ = get_parameter("linear_speed").as_double();
    cautious_linear_speed_ = get_parameter("cautious_linear_speed").as_double();
    lost_speed_ = get_parameter("lost_speed").as_double();
    fixed_turn_speed_ = get_parameter("fixed_turn_speed").as_double();
    fixed_turn_wz_ = get_parameter("fixed_turn_wz").as_double();

    pid_kp_ = get_parameter("pid_kp").as_double();
    pid_ki_ = get_parameter("pid_ki").as_double();
    pid_kd_ = get_parameter("pid_kd").as_double();
    max_wz_ = get_parameter("max_wz").as_double();
    straight_max_wz_ = get_parameter("straight_max_wz").as_double();

    heading_weight_ = get_parameter("heading_weight").as_double();
    error_deadband_px_ = get_parameter("error_deadband_px").as_double();
    wz_smooth_alpha_ = get_parameter("wz_smooth_alpha").as_double();

    straight_enter_min_lines_ = get_parameter("straight_enter_min_lines").as_int();
    min_valid_lines_ = get_parameter("min_valid_lines").as_int();
    turn_detect_min_lines_ = get_parameter("turn_detect_min_lines").as_int();
    turn_confirm_frames_ = get_parameter("turn_confirm_frames").as_int();
    lost_line_limit_ = get_parameter("lost_line_limit").as_int();
    min_color_run_ = get_parameter("min_color_run").as_int();

    // CIELab 参数获取
    yellow_l_low_ = get_parameter("yellow_l_low").as_int();
    yellow_l_high_ = get_parameter("yellow_l_high").as_int();
    yellow_a_low_ = get_parameter("yellow_a_low").as_int();
    yellow_a_high_ = get_parameter("yellow_a_high").as_int();
    yellow_b_low_ = get_parameter("yellow_b_low").as_int();
    yellow_b_high_ = get_parameter("yellow_b_high").as_int();

    green_l_low_ = get_parameter("green_l_low").as_int();
    green_l_high_ = get_parameter("green_l_high").as_int();
    green_a_low_ = get_parameter("green_a_low").as_int();
    green_a_high_ = get_parameter("green_a_high").as_int();
    green_b_low_ = get_parameter("green_b_low").as_int();
    green_b_high_ = get_parameter("green_b_high").as_int();

    pid_.set_gains(pid_kp_, pid_ki_, pid_kd_);
    pid_.set_limits(-max_wz_, max_wz_);

    // 构造 Lab 空间下的双限 Scalar
    yellow_low_ = cv::Scalar(yellow_l_low_, yellow_a_low_, yellow_b_low_);
    yellow_high_ = cv::Scalar(yellow_l_high_, yellow_a_high_, yellow_b_high_);
    green_low_ = cv::Scalar(green_l_low_, green_a_low_, green_b_low_);
    green_high_ = cv::Scalar(green_l_high_, green_a_high_, green_b_high_);

    auto image_qos = rclcpp::SensorDataQoS();
    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);

    direct_start_pub_ = create_publisher<std_msgs::msg::Bool>(direct_start_topic_, cmd_qos);

    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      stop_command_topic_, cmd_qos,
      std::bind(&LabTrackLaneNode::on_stop_command, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&LabTrackLaneNode::on_image, this, std::placeholders::_1));

    ai_subscription_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "win_detection", 10,
      std::bind(&LabTrackLaneNode::yolo_callback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / std::max(1, control_hz_)),
      std::bind(&LabTrackLaneNode::on_control_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "【信息】CIELab赛道巡线节点已启动，运行方向=%s，扫描方式=30条扫描线，全宽扫描，ROI=图像高度%.0f%%~%.0f%%，直线轻微中线纠偏，图像话题=%s，速度话题=%s，停车控制话题=%s，YOLO避障话题=win_detection，DirectStart话题=%s，避障与自动切二线机制已启用",
      track_clockwise_ ? "顺时针" : "逆时针",
      static_cast<double>(roi_top_ratio_ * 100.0F),
      static_cast<double>(roi_bottom_ratio_ * 100.0F),
      image_topic_.c_str(),
      cmd_vel_topic_.c_str(),
      stop_command_topic_.c_str(),
      direct_start_topic_.c_str());
  }

  ~LabTrackLaneNode() override
  {
    if (show_debug_) cv::destroyAllWindows();
  }

private:
  // ==== 极简避障参数 ====
  bool   is_exe_obs_     = false;
  bool   is_left_        = true;
  int    obs_turn_frames_= 0;
  float  revision_       = 0.0;

  const double IMAGE_AREA = 1920.0 * 1080.0; 
  const float LINE_REAL_MIDX = 960.0;        

  void trigger_second_line_handoff(float line_area_ratio, float line_confidence)
  {
    if (handoff_triggered_.exchange(true, std::memory_order_relaxed)) {
      return;
    }

    external_stop_requested_.store(true, std::memory_order_relaxed);

    geometry_msgs::msg::Twist stop_cmd{};
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      latest_cmd_ = stop_cmd;
    }

    std_msgs::msg::Bool start_msg{};
    start_msg.data = true;

    RCLCPP_WARN(
      get_logger(),
      "【自动切换】连续识别到黑线 line，准备发布 DirectStart=true 并退出 cure_line | 连续帧=%d 面积占比=%.6f 置信度=%.3f",
      second_line_detect_count_, line_area_ratio, line_confidence);

    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(stop_cmd);
      direct_start_pub_->publish(start_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    RCLCPP_WARN(
      get_logger(),
      "【自动切换】已向 %s 发布 true，second_line 将接管，cure_line 现在退出",
      direct_start_topic_.c_str());

    rclcpp::shutdown();
  }

  void yolo_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
      if (handoff_triggered_.load(std::memory_order_relaxed)) {
        return;
      }

      bool saw_second_line = false;
      float max_second_line_area = 0.0F;
      float max_second_line_confidence = 0.0F;

      for (const auto& target : msg->targets) {
        if (target.type != "line") {
          continue;
        }

        for (const auto& roi : target.rois) {
          const float area = static_cast<float>(roi.rect.width * roi.rect.height);
          const float area_ratio = static_cast<float>(area / IMAGE_AREA);
          const float confidence = roi.confidence;

          if (confidence >= second_line_min_confidence_ &&
              area_ratio >= second_line_min_area_ratio_) {
            saw_second_line = true;
            if (area_ratio > max_second_line_area) {
              max_second_line_area = area_ratio;
              max_second_line_confidence = confidence;
            }
          }
        }
      }

      if (saw_second_line) {
        second_line_detect_count_++;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "【自动切换】检测到 second_line 黑线 line | 连续帧=%d/%d 最大面积占比=%.6f 最大置信度=%.3f",
          second_line_detect_count_, second_line_confirm_frames_,
          max_second_line_area, max_second_line_confidence);

        if (second_line_detect_count_ >= second_line_confirm_frames_) {
          trigger_second_line_handoff(max_second_line_area, max_second_line_confidence);
          return;
        }
      } else {
        second_line_detect_count_ = 0;
      }

      std::vector<YoloRect> results_hat;
      for (const auto& target : msg->targets) {
        if(target.type == "hat"){
          for (const auto& roi : target.rois){
            YoloRect y;
            y.xmin = roi.rect.x_offset;
            y.ymin = roi.rect.y_offset;
            y.xmax = roi.rect.x_offset + roi.rect.width;
            y.ymax = roi.rect.y_offset + roi.rect.height;
            y.confidence = roi.confidence;
            y.type = "hat";
            results_hat.push_back(y);
          }
        }
      }

      if(is_exe_obs_)
      {
          geometry_msgs::msg::Twist obs_cmd{};
          obs_cmd.linear.x = bizhang_speed_;
          obs_turn_frames_++;

          if(is_left_) {
              obs_cmd.angular.z = -std::abs(fixed_turn_wz_);
          } else {
              obs_cmd.angular.z = std::abs(fixed_turn_wz_);
          }

          if (obs_turn_frames_ > 28 && obs_turn_frames_ <= 84) {
              obs_cmd.angular.z = -obs_cmd.angular.z;
          }
          if (obs_turn_frames_ > 84 && obs_turn_frames_ <= 112) {
              obs_cmd.angular.z = obs_cmd.angular.z;
          }

          if (obs_turn_frames_ > 112) {
             obs_turn_frames_ = 0;
             obs_cmd.linear.x = 0;
             obs_cmd.angular.z = 0;
             is_exe_obs_ = false;
          
             RCLCPP_WARN(get_logger(), "【避障】避障动作完成，退出避障状态，控制权交还给巡线");
          }

          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 800,
            "【避障】正在执行盲转避障 | 避让方向=%s 当前帧数=%d 线速度=%.3f 角速度=%.3f",
            is_left_ ? "向右避让" : "向左避让", obs_turn_frames_,
            obs_cmd.linear.x, obs_cmd.angular.z);

          {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            latest_cmd_ = obs_cmd;
          }
          return;
      }

      float max_rect_area = 0.0;
      float max_rect_width = 0.0;   
      float target_x = LINE_REAL_MIDX;

      for (auto& rect : results_hat) {
         float rect_width = rect.xmax - rect.xmin;
         float rect_height = rect.ymax - rect.ymin;
         float now_rect_area = rect_width * rect_height;

         if (max_rect_area < now_rect_area) {
            max_rect_area = now_rect_area;
            max_rect_width = rect_width;   
            target_x = (rect.xmax + rect.xmin) / 2.0;
         }
     }

     double area_ratio = max_rect_area / IMAGE_AREA;

     if (area_ratio >= 0.08 && max_rect_width > 380.0 && !is_exe_obs_)
     {
         is_exe_obs_ = true;
         obs_turn_frames_ = 0;

         if (target_x < LINE_REAL_MIDX) {
             is_left_ = true;
         } else {
             is_left_ = false;
         }

         RCLCPP_WARN(
           get_logger(),
           "【避障】触发盲转避障 | 障碍中心x=%.1f x宽度=%.1f 面积占比=%.4f 复用巡线转弯参数：线速度=%.3f 角速度=%.3f",
           target_x, max_rect_width, area_ratio, fixed_turn_speed_, fixed_turn_wz_);
     }
  }

  void on_control_timer()
  {
    if (handoff_triggered_.load(std::memory_order_relaxed)) {
      geometry_msgs::msg::Twist stop_cmd{};
      cmd_pub_->publish(stop_cmd);
      return;
    }

    if (external_stop_requested_.load(std::memory_order_relaxed)) {
      geometry_msgs::msg::Twist stop_cmd{};
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_cmd_ = stop_cmd;
      }
      cmd_pub_->publish(stop_cmd);
      return;
    }

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_pub_->publish(latest_cmd_);
  }

  void on_stop_command(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool need_stop = msg->data;
    external_stop_requested_.store(need_stop, std::memory_order_relaxed);

    geometry_msgs::msg::Twist stop_cmd{};
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      latest_cmd_ = stop_cmd;
    }

    last_wz_ = 0.0;
    lost_count_ = 0;
    left_turn_streak_ = 0;
    right_turn_streak_ = 0;
    pid_.reset();

    cmd_pub_->publish(stop_cmd);

    if (need_stop) {
      RCLCPP_WARN(get_logger(), "【停车控制】收到 take_photo 停车命令 true，cure_line 立即停车");
    } else {
      RCLCPP_WARN(get_logger(), "【停车控制】收到 take_photo 恢复命令 false，cure_line 恢复巡线");
    }
  }

  // ===================================
  // 图像巡线回调 (CIELab 重构版本)
  // ===================================
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "【错误】cv_bridge 转换异常：%s", e.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    if (handoff_triggered_.load(std::memory_order_relaxed)) {
      return;
    }

    if (external_stop_requested_.load(std::memory_order_relaxed)) {
      geometry_msgs::msg::Twist stop_cmd{};
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        latest_cmd_ = stop_cmd;
      }
      last_wz_ = 0.0;
      return;
    }

    if (is_exe_obs_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 800,
        "【避障】YOLO避障正在接管 cmd_vel，当前图像帧不执行巡线计算");
      return;
    }

    const int img_w = bgr.cols;
    const int img_h = bgr.rows;

    const double target_x =
      (target_x_ratio_ > 0.0 && target_x_ratio_ < 1.0)
        ? target_x_ratio_ * static_cast<double>(img_w)
        : target_center_px_param_;

    const int roi_top = clamp_int(static_cast<int>(img_h * roi_top_ratio_), 0, img_h - 2);
    const int roi_bottom = clamp_int(static_cast<int>(img_h * roi_bottom_ratio_), roi_top + 1, img_h - 1);

    cv::Rect roi_rect(0, roi_top, img_w, roi_bottom - roi_top);
    cv::Mat roi = bgr(roi_rect);

    // 更换为 CIELab 色彩空间进行处理
    cv::Mat lab, yellow_mask, green_mask;
    cv::cvtColor(roi, lab, cv::COLOR_BGR2Lab);

    // 基于 Lab 进行二值化通道解耦
    cv::inRange(lab, yellow_low_, yellow_high_, yellow_mask);
    cv::inRange(lab, green_low_, green_high_, green_mask);

    const cv::Mat k_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    const cv::Mat k_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));

    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, k_open);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, k_close);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, k_open);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, k_close);

    std::vector<double> center_values;
    std::vector<double> center_weights;
    std::vector<cv::Point> debug_centers;

    center_values.reserve(kNumScanLines);
    center_weights.reserve(kNumScanLines);
    debug_centers.reserve(kNumScanLines);

    int straight_votes = 0;
    int left_turn_votes = 0;
    int right_turn_votes = 0;

    cv::Mat debug = show_debug_ ? roi.clone() : cv::Mat{};
    std::string debug_mode_text = "【警告】丢线";

    for (int i = 0; i < kNumScanLines; ++i) {
      const int y = clamp_int(static_cast<int>(roi.rows * kScanRatios[i]), 0, roi.rows - 1);
      const RowResult row_result = analyze_row_pattern(yellow_mask, green_mask, y);

      if (row_result.decision == RowDecision::STRAIGHT) {
        straight_votes++;
        center_values.push_back(static_cast<double>(row_result.center_x));
        center_weights.push_back(static_cast<double>(kScanWeights[i]));
        debug_centers.emplace_back(row_result.center_x, y);

        if (show_debug_) {
          cv::circle(debug, cv::Point(row_result.left_x, y), 4,
                     cv::Scalar(255, 0, 0), -1);
          cv::circle(debug, cv::Point(row_result.right_x, y), 4,
                     cv::Scalar(0, 255, 0), -1);
          cv::circle(debug, cv::Point(row_result.center_x, y), 5,
                     cv::Scalar(0, 0, 255), -1);
        }
      } else if (row_result.decision == RowDecision::TURN_LEFT) {
        left_turn_votes++;

        if (show_debug_) {
          cv::circle(debug, cv::Point(20, y), 4,
                     cv::Scalar(255, 255, 0), -1);
        }
      } else if (row_result.decision == RowDecision::TURN_RIGHT) {
        right_turn_votes++;

        if (show_debug_) {
          cv::circle(debug, cv::Point(roi.cols - 20, y), 4,
                     cv::Scalar(0, 255, 255), -1);
        }
      }
    }

    geometry_msgs::msg::Twist cmd{};

    if (straight_votes >= straight_enter_min_lines_) {
      const double fused_center_x = weighted_average(center_values, center_weights);
      const double near_x = center_values.back();
      const double far_x = center_values.front();
      const double heading_px = (center_values.size() >= 2) ? (near_x - far_x) : 0.0;
      const double pos_err_px = target_x - fused_center_x;
      double control_err_px = pos_err_px + heading_weight_ * heading_px;

      if (std::abs(pos_err_px) < error_deadband_px_ && std::abs(heading_px) < error_deadband_px_) {
        control_err_px = 0.0;
      }

      double wz = pid_.update(control_err_px);
      wz = wz_smooth_alpha_ * wz + (1.0 - wz_smooth_alpha_) * last_wz_;

      cmd.linear.x = (straight_votes >= min_valid_lines_) ? linear_speed_ : cautious_linear_speed_;
      cmd.angular.z = std::clamp(wz, -straight_max_wz_, straight_max_wz_);

      last_wz_ = cmd.angular.z;
      lost_count_ = 0;
      left_turn_streak_ = 0;
      right_turn_streak_ = 0;

      if (straight_votes >= min_valid_lines_) {
        debug_mode_text = "【信息】直线巡线-轻微中线纠偏";
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 800,
          "【信息】直线巡线 | 直线票数=%d 左转票数=%d 右转票数=%d 中线=%.1f 横向误差=%.1f 航向误差=%.1f 控制误差=%.1f 角速度=%.3f",
          straight_votes, left_turn_votes, right_turn_votes,
          fused_center_x, pos_err_px, heading_px, control_err_px, cmd.angular.z);
      } else {
        debug_mode_text = "【信息】谨慎直线巡线-轻微中线纠偏";
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 800,
          "【信息】谨慎直线巡线 | 直线票数=%d 左转票数=%d 右转票数=%d 中线=%.1f 横向误差=%.1f 航向误差=%.1f 控制误差=%.1f 角速度=%.3f",
          straight_votes, left_turn_votes, right_turn_votes,
          fused_center_x, pos_err_px, heading_px, control_err_px, cmd.angular.z);
      }

      if (show_debug_) {
        cv::line(debug,
                 cv::Point(static_cast<int>(target_x), 0),
                 cv::Point(static_cast<int>(target_x), roi.rows),
                 cv::Scalar(255, 0, 255), 2);

        for (size_t i = 1; i < debug_centers.size(); ++i) {
          cv::line(debug, debug_centers[i - 1], debug_centers[i],
                   cv::Scalar(0, 0, 255), 2);
        }

        cv::circle(debug,
                   cv::Point(static_cast<int>(fused_center_x), roi.rows - 20),
                   6, cv::Scalar(255, 255, 0), -1);
      }

    } else {
      const bool right_turn_candidate = (right_turn_votes >= turn_detect_min_lines_) && (right_turn_votes > left_turn_votes);
      const bool left_turn_candidate = (left_turn_votes >= turn_detect_min_lines_) && (left_turn_votes > right_turn_votes);

      if (right_turn_candidate) {
        right_turn_streak_++;
        left_turn_streak_ = 0;

        if (right_turn_streak_ >= turn_confirm_frames_) {
          cmd.linear.x = fixed_turn_speed_;
          cmd.angular.z = -std::abs(fixed_turn_wz_);
          last_wz_ = cmd.angular.z;
          lost_count_ = 0;
          pid_.reset();

          debug_mode_text = "【信息】进入右转";
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 800,
            "【信息】进入右转 | 右转票数=%d 左转票数=%d 连续确认帧=%d 角速度=%.3f",
            right_turn_votes, left_turn_votes, right_turn_streak_, cmd.angular.z);
        } else {
          cmd.linear.x = cautious_linear_speed_;
          cmd.angular.z = 0.0;
          last_wz_ = 0.0;

          debug_mode_text = "【信息】右转确认中";
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 800,
            "【信息】右转确认中 | 右转票数=%d 左转票数=%d 当前确认帧=%d/%d 角速度=%.3f",
            right_turn_votes, left_turn_votes,
            right_turn_streak_, turn_confirm_frames_, cmd.angular.z);
        }
      } else if (left_turn_candidate) {
        left_turn_streak_++;
        right_turn_streak_ = 0;

        if (left_turn_streak_ >= turn_confirm_frames_) {
          cmd.linear.x = fixed_turn_speed_;
          cmd.angular.z = std::abs(fixed_turn_wz_);
          last_wz_ = cmd.angular.z;
          lost_count_ = 0;
          pid_.reset();

          debug_mode_text = "【信息】进入左转";
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 800,
            "【信息】进入左转 | 左转票数=%d 右转票数=%d 连续确认帧=%d 角速度=%.3f",
            left_turn_votes, right_turn_votes, left_turn_streak_, cmd.angular.z);
        } else {
          cmd.linear.x = cautious_linear_speed_;
          cmd.angular.z = 0.0;
          last_wz_ = 0.0;

          debug_mode_text = "【信息】左转确认中";
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 800,
            "【信息】左转确认中 | 左转票数=%d 右转票数=%d 当前确认帧=%d/%d 角速度=%.3f",
            left_turn_votes, right_turn_votes,
            left_turn_streak_, turn_confirm_frames_, cmd.angular.z);
        }
      } else {
        left_turn_streak_ = 0;
        right_turn_streak_ = 0;
        lost_count_++;

        cmd.linear.x = (lost_count_ >= lost_line_limit_) ? 0.1 : lost_speed_;
        cmd.angular.z = 0.0;
        last_wz_ = 0.0;
        pid_.reset();

        debug_mode_text = "【警告】丢线";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 800,
          "【警告】丢线 | 直线票数=%d 左转票数=%d 右转票数=%d 连续丢线次数=%d 角速度=%.3f",
          straight_votes, left_turn_votes, right_turn_votes,
          lost_count_, cmd.angular.z);
      }
    }

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      latest_cmd_ = cmd;
    }

    if (show_debug_) {
      cv::putText(debug,
                  track_clockwise_ ? "CW" : "CCW",
                  cv::Point(20, 35),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 255), 2);

      cv::putText(debug,
                  "30 LINES ROI 60%-98% (CIELab)",
                  cv::Point(20, 70),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 255), 2);

      cv::putText(debug,
                  debug_mode_text,
                  cv::Point(20, 105),
                  cv::FONT_HERSHEY_SIMPLEX, 0.75,
                  cv::Scalar(0, 0, 255), 2);
      cv::waitKey(1);
    }
  }

  int get_pixel_color(const cv::Mat& yellow_mask, const cv::Mat& green_mask, int y, int x) const
  {
    const bool is_yellow = (yellow_mask.at<uchar>(y, x) == 255);
    const bool is_green = (green_mask.at<uchar>(y, x) == 255);

    if (is_yellow && !is_green) return COLOR_YELLOW;
    if (is_green && !is_yellow) return COLOR_GREEN;
    return COLOR_NONE;
  }

  std::vector<ColorRun> build_color_runs_on_row(const cv::Mat& yellow_mask, const cv::Mat& green_mask, int y) const
  {
    std::vector<ColorRun> runs;
    const int w = yellow_mask.cols;
    int x = 0;

    while (x < w) {
      const int color = get_pixel_color(yellow_mask, green_mask, y, x);
      if (color == COLOR_NONE) { ++x; continue; }

      const int start_x = x;
      int end_x = x;
      ++x;

      while (x < w) {
        const int next_color = get_pixel_color(yellow_mask, green_mask, y, x);
        if (next_color != color) break;
        end_x = x;
        ++x;
      }

      ColorRun run;
      run.color = color;
      run.left = start_x;
      run.right = end_x;

      if (run.len() >= min_color_run_) runs.push_back(run);
    }

    return runs;
  }

  static bool has_green_before(const std::vector<ColorRun>& runs, size_t index) {
    for (size_t k = 0; k < index; ++k) {
      if (runs[k].color == COLOR_GREEN) return true;
    }
    return false;
  }

  static bool has_green_after(const std::vector<ColorRun>& runs, size_t index) {
    for (size_t k = index + 1; k < runs.size(); ++k) {
      if (runs[k].color == COLOR_GREEN) return true;
    }
    return false;
  }

  RowResult analyze_row_pattern(const cv::Mat& yellow_mask, const cv::Mat& green_mask, int y) const
  {
    RowResult result;
    const std::vector<ColorRun> runs = build_color_runs_on_row(yellow_mask, green_mask, y);

    if (runs.empty()) return result;

    for (size_t i = 0; i + 2 < runs.size(); ++i) {
      if (runs[i].color == COLOR_GREEN && runs[i + 1].color == COLOR_YELLOW && runs[i + 2].color == COLOR_GREEN) {
        result.decision = RowDecision::STRAIGHT;
        result.left_x = runs[i + 1].left;
        result.right_x = runs[i + 1].right;
        result.center_x = (runs[i + 1].left + runs[i + 1].right) / 2;
        return result;
      }
    }

    for (size_t i = 0; i + 1 < runs.size(); ++i) {
      if (runs[i].color == COLOR_GREEN && runs[i + 1].color == COLOR_YELLOW && !has_green_after(runs, i + 1)) {
        result.decision = track_clockwise_ ? RowDecision::TURN_LEFT : RowDecision::TURN_RIGHT;
        return result;
      }
    }

    for (size_t i = 0; i + 1 < runs.size(); ++i) {
      if (runs[i].color == COLOR_YELLOW && runs[i + 1].color == COLOR_GREEN && !has_green_before(runs, i)) {
        result.decision = track_clockwise_ ? RowDecision::TURN_RIGHT : RowDecision::TURN_LEFT;
        return result;
      }
    }

    return result;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr direct_start_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_subscription_; 

  std::string image_topic_{"/image_raw"};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string stop_command_topic_{"/take_photo/stop_cure_line"};
  std::string direct_start_topic_{"/direct_start"};

  bool show_debug_{true};
  bool track_clockwise_{false};

  double target_center_px_param_{640.0};
  double target_x_ratio_{0.50};
  float roi_top_ratio_{0.60F};
  float roi_bottom_ratio_{0.98F};

  int control_hz_{20};
  double linear_speed_{0.5};
  double cautious_linear_speed_{0.5};
  double lost_speed_{0.08};
  double fixed_turn_speed_{0.18};
  double fixed_turn_wz_{1.0};
  
  double bizhang_speed_{0.2};

  double pid_kp_{0.0030};
  double pid_ki_{0.0};
  double pid_kd_{0.0008};
  double max_wz_{1.20};
  double straight_max_wz_{0.38};

  double heading_weight_{0.16};
  double error_deadband_px_{5.0};
  double wz_smooth_alpha_{0.30};

  int straight_enter_min_lines_{6};
  int min_valid_lines_{6};
  int turn_detect_min_lines_{10};
  int turn_confirm_frames_{2};
  int lost_line_limit_{10};
  int min_color_run_{8};

  // 初始化调参设定的 CIELab 默认阈值边界
  int yellow_l_low_{120};
  int yellow_l_high_{255};
  int yellow_a_low_{125};
  int yellow_a_high_{145};
  int yellow_b_low_{138};
  int yellow_b_high_{255};

  int green_l_low_{120};
  int green_l_high_{255};
  int green_a_low_{95};
  int green_a_high_{125};
  int green_b_low_{0};
  int green_b_high_{125};

  cv::Scalar yellow_low_, yellow_high_, green_low_, green_high_;

  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist latest_cmd_{};
  std::atomic<bool> external_stop_requested_{false};
  std::atomic<bool> handoff_triggered_{false};

  int second_line_confirm_frames_{2};
  int second_line_detect_count_{0};
  double second_line_min_confidence_{0.90};
  double second_line_min_area_ratio_{0.0002};

  double last_wz_{0.0};
  int lost_count_{0};
  int left_turn_streak_{0};
  int right_turn_streak_{0};

  // 根据参数范围同步初始化 PID 边界
  PixelPID pid_{0.0030, 0.0, 0.0008, -1.20, 1.20};
};

int main(int argc, char* argv[])
{
  wait_for_any_key();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabTrackLaneNode>());
  rclcpp::shutdown();
 
  return 0;
}