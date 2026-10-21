#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "lio_ob/math_utils.hpp"

namespace lio_ob
{

class SubmapLioNode : public rclcpp::Node
{
public:
  SubmapLioNode()
  : Node("submap_lio")
  {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lio_ob/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "lio_ob_odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    publish_odom_bridge_tf_ = declare_parameter<bool>("publish_odom_bridge_tf", true);
    wheel_odom_frame_ = declare_parameter<std::string>("wheel_odom_frame", "odom");
    enable_ring_y_correction_ = declare_parameter<bool>("enable_ring_y_correction", true);
    ring_y_correction_topic_ = declare_parameter<std::string>(
      "ring_y_correction_topic", "/lio_ob/ring_y_correction");
    ring_y_correction_gain_ = std::clamp(
      declare_parameter<double>("ring_y_correction_gain", 0.90), 0.0, 1.0);
    ring_y_correction_max_innovation_ = std::max(
      0.05, declare_parameter<double>("ring_y_correction_max_innovation", 0.35));
    ring_y_correction_min_std_ = std::max(
      0.005, declare_parameter<double>("ring_y_correction_min_std", 0.015));
    ring_y_correction_max_std_ = std::max(
      ring_y_correction_min_std_,
      declare_parameter<double>("ring_y_correction_max_std", 0.10));

    laser_extrinsic_.x = declare_parameter<double>("laser_x", 0.0);
    laser_extrinsic_.y = declare_parameter<double>("laser_y", 0.0);
    laser_extrinsic_.yaw = declare_parameter<double>("laser_yaw", 0.0);
    deskew_scan_ = declare_parameter<bool>("deskew_scan", true);
    scan_index_time_reversed_ =
      declare_parameter<bool>("scan_index_time_reversed", true);
    scan_stamp_is_end_ = declare_parameter<bool>("scan_stamp_is_end", true);
    scan_stamp_offset_ = declare_parameter<double>("scan_stamp_offset", 0.0);
    motion_buffer_duration_ = std::max(
      0.5, declare_parameter<double>("motion_buffer_duration", 2.0));
    motion_max_sample_gap_ = std::max(
      0.05, declare_parameter<double>("motion_max_sample_gap", 0.12));
    max_pending_scans_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("max_pending_scans", 2)));
    scan_min_range_ = declare_parameter<double>("scan_min_range", 0.25);
    scan_max_range_ = declare_parameter<double>("scan_max_range", 12.0);
    scan_stride_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("scan_stride", 4)));

    icp_max_iterations_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("icp_max_iterations", 8)));
    icp_max_correspondence_ = declare_parameter<double>("icp_max_correspondence", 0.35);
    icp_normal_max_gap_ = declare_parameter<double>("icp_normal_max_gap", 0.30);
    icp_huber_delta_ = declare_parameter<double>("icp_huber_delta", 0.08);
    icp_min_correspondences_ = std::max(
      10, static_cast<int>(declare_parameter<std::int64_t>("icp_min_correspondences", 30)));
    icp_max_rmse_ = declare_parameter<double>("icp_max_rmse", 0.10);
    icp_translation_convergence_ = declare_parameter<double>("icp_translation_convergence", 1e-4);
    icp_rotation_convergence_ = declare_parameter<double>("icp_rotation_convergence", 1e-4);
    icp_max_translation_jump_ = declare_parameter<double>("icp_max_translation_jump", 0.20);
    icp_max_rotation_jump_ = declare_parameter<double>("icp_max_rotation_jump", 0.35);
    icp_max_prediction_translation_error_ =
      declare_parameter<double>("icp_max_prediction_translation_error", 0.10);
    icp_max_prediction_rotation_error_ =
      declare_parameter<double>("icp_max_prediction_rotation_error", 0.10);

    keyframe_distance_ = std::max(0.05, declare_parameter<double>("keyframe_distance", 0.20));
    keyframe_angle_ = std::max(
      1.0, declare_parameter<double>("keyframe_angle_deg", 30.0)) * kPi / 180.0;
    submap_max_keyframes_ = std::max(
      3, static_cast<int>(declare_parameter<std::int64_t>("submap_max_keyframes", 50)));
    ndt_max_iterations_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("ndt_max_iterations", 20)));
    ndt_voxel_size_ = std::max(0.05, declare_parameter<double>("ndt_voxel_size", 0.20));
    ndt_min_effective_points_ = std::max(
      5, static_cast<int>(declare_parameter<std::int64_t>("ndt_min_effective_points", 10)));
    ndt_min_points_per_voxel_ = std::max(
      2, static_cast<int>(declare_parameter<std::int64_t>("ndt_min_points_per_voxel", 3)));
    ndt_max_points_per_voxel_ = std::max(
      ndt_min_points_per_voxel_,
      static_cast<int>(declare_parameter<std::int64_t>("ndt_max_points_per_voxel", 50)));
    ndt_outlier_threshold_ = std::max(
      1.0, declare_parameter<double>("ndt_outlier_threshold", 5.991));
    ndt_voxel_capacity_ = std::max(
      100, static_cast<int>(declare_parameter<std::int64_t>("ndt_voxel_capacity", 10000)));
    ndt_regularization_ = std::max(
      1e-6, declare_parameter<double>("ndt_regularization", 1e-3));
    ndt_prior_information_ = std::max(
      1.0, declare_parameter<double>("ndt_prior_information", 1e3));
    ndt_information_ratio_ = std::max(
      1e-5, declare_parameter<double>("ndt_information_ratio", 0.01));
    ndt_convergence_epsilon_ = std::max(
      1e-5, declare_parameter<double>("ndt_convergence_epsilon", 1e-3));
    ndt_nearby_type_ = std::clamp(
      static_cast<int>(declare_parameter<std::int64_t>("ndt_nearby_type", 2)), 0, 2);
    ndt_max_translation_correction_ = std::max(
      0.05, declare_parameter<double>("ndt_max_translation_correction", 0.25));
    ndt_max_rotation_correction_ = std::max(
      0.05, declare_parameter<double>("ndt_max_rotation_correction", 0.20));

    occupancy_resolution_ = std::max(
      1.0, declare_parameter<double>("occupancy_pixels_per_meter", 20.0));
    occupancy_image_size_ = std::max(
      20, static_cast<int>(declare_parameter<std::int64_t>("occupancy_image_size", 200)));
    occupancy_origin_x_ = declare_parameter<double>("occupancy_origin_x", -2.5);
    occupancy_origin_y_ = declare_parameter<double>("occupancy_origin_y", -2.5);
    path_publish_period_ = std::max(
      0.05, declare_parameter<double>("path_publish_period", 0.20));
    path_pose_min_distance_ = std::max(
      0.001, declare_parameter<double>("path_pose_min_distance", 0.01));
    path_pose_min_angle_ = std::max(
      0.001, declare_parameter<double>("path_pose_min_angle", 0.01));
    path_max_poses_ = std::max(
      100, static_cast<int>(declare_parameter<std::int64_t>("path_max_poses", 2000)));
    static_motion_velocity_threshold_ =
      declare_parameter<double>("static_motion_velocity_threshold", 0.03);
    static_motion_gyro_threshold_ =
      declare_parameter<double>("static_motion_gyro_threshold", 0.01);

    imu_min_dt_ = std::max(1e-3, declare_parameter<double>("imu_min_dt", 0.01));
    imu_max_dt_ = std::max(
      imu_min_dt_, declare_parameter<double>("imu_max_dt", 0.15));
    imu_integration_step_ = std::max(
      1e-3, declare_parameter<double>("imu_integration_step", 0.05));
    stationary_bias_alpha_ = std::clamp(
      declare_parameter<double>("stationary_bias_alpha", 0.05), 0.0, 1.0);
    use_imu_orientation_ = declare_parameter<bool>("use_imu_orientation", false);
    imu_yaw_std_ = std::max(0.005, declare_parameter<double>("imu_yaw_std", 0.05));

    gyro_noise_std_ = declare_parameter<double>("gyro_noise_std", 0.003);
    gyro_bias_walk_std_ = declare_parameter<double>("gyro_bias_walk_std", 0.0002);
    velocity_walk_std_ = declare_parameter<double>("velocity_walk_std", 0.20);
    wheel_velocity_std_ = declare_parameter<double>("wheel_velocity_std", 0.06);
    lidar_position_std_ = declare_parameter<double>("lidar_position_std", 0.04);
    lidar_yaw_std_ = declare_parameter<double>("lidar_yaw_std", 0.05);
    const double initial_x = declare_parameter<double>("initial_x", 0.54);
    const double initial_y = declare_parameter<double>("initial_y", 0.30);
    const double initial_yaw = declare_parameter<double>("initial_yaw", 0.0);

    state_.setZero();
    state_(0) = initial_x;
    state_(1) = initial_y;
    state_(2) = initial_yaw;
    covariance_.setIdentity();
    covariance_(0, 0) = 1e-4;
    covariance_(1, 1) = 1e-4;
    covariance_(2, 2) = 1e-4;
    covariance_(3, 3) = 0.02;
    covariance_(4, 4) = 1e-5;

    // Small queues prefer current motion over replaying stale callbacks after
    // ICP briefly occupies the single-threaded executor.
    auto motion_qos = rclcpp::SensorDataQoS().keep_last(5);
    auto scan_qos = rclcpp::SensorDataQoS().keep_last(2);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, motion_qos,
      std::bind(&SubmapLioNode::imu_callback, this, std::placeholders::_1));
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, motion_qos,
      std::bind(&SubmapLioNode::wheel_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, scan_qos,
      std::bind(&SubmapLioNode::scan_callback, this, std::placeholders::_1));
    ring_y_correction_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      ring_y_correction_topic_, rclcpp::QoS(5),
      std::bind(&SubmapLioNode::ring_y_correction_callback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/lio_ob/path", rclcpp::QoS(1).transient_local().reliable());
    submap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/lio_ob/submap", rclcpp::QoS(1).transient_local().reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    path_.header.frame_id = odom_frame_;

    RCLCPP_INFO(
      get_logger(),
      "二维迭代 NDT-EKF LIO：IMU=%s 轮速=%s 雷达=%s 输出=%s 坐标系=%s "
      "初始位姿=(%.2f %.2f %.2f) 扫描顺序=%s 时间戳=%s",
      imu_topic_.c_str(), wheel_odom_topic_.c_str(), scan_topic_.c_str(), odom_topic_.c_str(),
      odom_frame_.c_str(), initial_x, initial_y, initial_yaw,
      scan_index_time_reversed_ ? "顺时针/数组反序" : "数组正序",
      scan_stamp_is_end_ ? "整圈末端" : "首点");
    RCLCPP_INFO(
      get_logger(),
      "NDT 子地图：体素=%.2f m，关键帧距离=%.2f m，关键帧角度=%.1f deg，最多=%d 帧",
      ndt_voxel_size_, keyframe_distance_, keyframe_angle_ * 180.0 / kPi,
      submap_max_keyframes_);
    if (enable_ring_y_correction_) {
      RCLCPP_INFO(
        get_logger(), "环形区 Y 坐标纠正已启用：话题=%s 增益=%.2f 上限=%.2f m",
        ring_y_correction_topic_.c_str(), ring_y_correction_gain_,
        ring_y_correction_max_innovation_);
    }
  }

private:
  using State = Eigen::Matrix<double, 5, 1>;  // x, y, yaw, longitudinal velocity, gyro z bias
  using Covariance = Eigen::Matrix<double, 5, 5>;

  struct IcpResult
  {
    bool valid{false};
    Pose2d relative_pose;
    int correspondences{0};
    double rmse{std::numeric_limits<double>::infinity()};
  };

  struct NdtResult
  {
    bool valid{false};
    Pose2d pose;
    int effective_points{0};
    int iterations{0};
    double mean_mahalanobis{std::numeric_limits<double>::infinity()};
  };

  struct VoxelAccumulator
  {
    int count{0};
    Eigen::Vector2d sum{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d sum_outer{Eigen::Matrix2d::Zero()};
  };

  struct NdtVoxel
  {
    int count{0};
    Eigen::Vector2d mean{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d information{Eigen::Matrix2d::Identity()};
  };

  struct TimedSample
  {
    rclcpp::Time stamp;
    double value{0.0};
  };

  struct ScanTiming
  {
    rclcpp::Time first_point;
    rclcpp::Time last_point;
    rclcpp::Time scan_end;
    double point_period{0.0};
    double index_zero_to_end{0.0};
  };

  Pose2d state_pose() const
  {
    return {state_(0), state_(1), state_(2)};
  }

  void ring_y_correction_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!enable_ring_y_correction_) {
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != odom_frame_) {
      RCLCPP_WARN(
        get_logger(), "拒绝环形区 Y 坐标纠正：坐标系为 '%s'，期望 '%s'",
        msg->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    const double measured_y = msg->pose.pose.position.y;
    const double supplied_variance = msg->pose.covariance[7];
    if (!std::isfinite(measured_y) || !std::isfinite(supplied_variance) ||
      supplied_variance <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "拒绝格式错误的环形区 Y 坐标纠正");
      return;
    }
    const double measurement_std = std::clamp(
      std::sqrt(supplied_variance), ring_y_correction_min_std_,
      ring_y_correction_max_std_);
    const double innovation = measured_y - state_(1);
    if (std::abs(innovation) > ring_y_correction_max_innovation_) {
      RCLCPP_WARN(
        get_logger(), "拒绝环形区 Y 坐标纠正 %.3f m：上限为 %.3f m",
        innovation, ring_y_correction_max_innovation_);
      return;
    }

    // This is a one-shot map registration at a known semantic landmark, not a
    // continuous lane-following observation. Shift both the filter state and
    // the scan-to-scan ICP anchor so the following lidar update cannot undo it.
    const double applied_correction = ring_y_correction_gain_ * innovation;
    state_(1) += applied_correction;
    if (have_reference_scan_) {
      lidar_pose_.y += applied_correction;
    }
    if (!submap_keyframes_.empty()) {
      for (auto & keyframe : submap_keyframes_) {
        for (auto & point : keyframe) {
          point.y() += applied_correction;
        }
      }
      if (have_keyframe_pose_) {
        last_keyframe_pose_.y += applied_correction;
      }
      rebuild_ndt_submap();
    }
    for (Eigen::Index index = 0; index < covariance_.rows(); ++index) {
      covariance_(1, index) = 0.0;
      covariance_(index, 1) = 0.0;
    }
    covariance_(1, 1) = measurement_std * measurement_std;
    RCLCPP_WARN(
      get_logger(),
      "已应用环形区 Y 坐标纠正：测量=%.3f 偏差=%.3f 实施=%.3f 新Y=%.3f",
      measured_y, innovation, applied_correction, state_(1));
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    const double gyro_z = msg->angular_velocity.z;
    push_sample(imu_history_, stamp, gyro_z);

    if (!have_imu_stamp_) {
      last_imu_stamp_ = stamp;
      last_gyro_z_ = gyro_z;
      have_imu_stamp_ = true;
      observe_imu_orientation(*msg);
      try_process_pending_scans();
      publish_state(stamp);
      return;
    }

      double dt = (stamp - last_imu_stamp_).seconds();
      last_imu_stamp_ = stamp;
      if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "忽略非正数 IMU 时间间隔 %.6f s", dt);
        last_gyro_z_ = gyro_z;
        try_process_pending_scans();
        publish_state(stamp);
        return;
      }
      if (dt < imu_min_dt_ || dt > imu_max_dt_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU 时间间隔异常 %.6f s：仍按真实时长预测，雷达去畸变将拒绝稀疏覆盖",
          dt);
      }

    if (robot_is_static(gyro_z)) {
      // Wheel speed and gyro jointly indicate no physical motion. Holding the
      // pose here prevents residual gyro bias and degenerate ICP from walking
      // the estimate while the vehicle is parked.
      state_(3) = 0.0;
      state_(4) =
        (1.0 - stationary_bias_alpha_) * state_(4) + stationary_bias_alpha_ * gyro_z;
      covariance_(3, 3) = std::min(covariance_(3, 3), 1e-4);
      covariance_(4, 4) = std::min(covariance_(4, 4), gyro_noise_std_ * gyro_noise_std_);
    } else {
      const double interval_gyro = 0.5 * (last_gyro_z_ + gyro_z);
      double remaining = dt;
      while (remaining > 0.0) {
        const double step = std::min(remaining, imu_integration_step_);
        predict(interval_gyro, step);
        remaining -= step;
      }
    }

    observe_imu_orientation(*msg);

    last_gyro_z_ = gyro_z;
    try_process_pending_scans();
    publish_state(stamp);
  }

  void observe_imu_orientation(const sensor_msgs::msg::Imu & msg)
  {
    if (!use_imu_orientation_ || msg.orientation_covariance[0] < 0.0) {
      return;
    }
    const auto & q = msg.orientation;
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_sq) || norm_sq < 0.5 || norm_sq > 1.5) {
      return;
    }
    const double scale = 1.0 / norm_sq;
    const double imu_yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y) * scale,
      1.0 - 2.0 * (q.y * q.y + q.z * q.z) * scale);
    if (!have_imu_orientation_reference_) {
      initial_imu_orientation_yaw_ = imu_yaw;
      imu_orientation_anchor_yaw_ = state_(2);
      have_imu_orientation_reference_ = true;
      RCLCPP_INFO(
        get_logger(), "已建立 IMU 航向基准：IMU=%.3f rad，场地航向=%.3f rad",
        imu_yaw, imu_orientation_anchor_yaw_);
      return;
    }

    const double measured_yaw = normalize_angle(
      imu_orientation_anchor_yaw_ + normalize_angle(imu_yaw - initial_imu_orientation_yaw_));
    const double innovation = normalize_angle(measured_yaw - state_(2));
    const double variance = imu_yaw_std_ * imu_yaw_std_;
    const double innovation_variance = covariance_(2, 2) + variance;
    if (!(innovation_variance > 0.0) || !std::isfinite(innovation)) {
      return;
    }
    const Eigen::Matrix<double, 5, 1> gain = covariance_.col(2) / innovation_variance;
    const double yaw_before = state_(2);
    state_ += gain * innovation;
    state_(2) = normalize_angle(state_(2));
    Eigen::Matrix<double, 1, 5> observation = Eigen::Matrix<double, 1, 5>::Zero();
    observation(0, 2) = 1.0;
    const Covariance identity = Covariance::Identity();
    const Covariance correction = identity - gain * observation;
    covariance_ = correction * covariance_ * correction.transpose() +
      gain * variance * gain.transpose();

    // ICP stores a global scan-to-scan anchor. Keep that anchor on the same
    // absolute heading as the IMU-constrained filter, otherwise the next scan
    // would immediately inject the old yaw drift again.
    if (have_reference_scan_) {
      Pose2d scan_to_filter;
      if (motion_from_scan_end_to_filter(last_scan_end_stamp_, scan_to_filter)) {
        lidar_pose_.yaw = normalize_angle(measured_yaw - scan_to_filter.yaw);
      } else {
        lidar_pose_.yaw = normalize_angle(
          lidar_pose_.yaw + normalize_angle(state_(2) - yaw_before));
      }
    }
  }

  bool robot_is_static(double gyro_z) const
  {
    return have_wheel_speed_ &&
      std::abs(latest_wheel_speed_) <= static_motion_velocity_threshold_ &&
      std::abs(gyro_z - state_(4)) <= static_motion_gyro_threshold_;
  }

  void predict(double gyro_z, double dt)
  {
    const double yaw = state_(2);
    const double velocity = state_(3);
    const double unbiased_gyro = gyro_z - state_(4);

    state_(0) += velocity * std::cos(yaw) * dt;
    state_(1) += velocity * std::sin(yaw) * dt;
    state_(2) = normalize_angle(state_(2) + unbiased_gyro * dt);

    Covariance transition = Covariance::Identity();
    transition(0, 2) = -velocity * std::sin(yaw) * dt;
    transition(0, 3) = std::cos(yaw) * dt;
    transition(1, 2) = velocity * std::cos(yaw) * dt;
    transition(1, 3) = std::sin(yaw) * dt;
    transition(2, 4) = -dt;

    Covariance process_noise = Covariance::Zero();
    const double velocity_var = velocity_walk_std_ * velocity_walk_std_;
    const double gyro_var = gyro_noise_std_ * gyro_noise_std_;
    const double bias_var = gyro_bias_walk_std_ * gyro_bias_walk_std_;
    process_noise(0, 0) = 0.25 * velocity_var * dt * dt;
    process_noise(1, 1) = 0.25 * velocity_var * dt * dt;
    process_noise(2, 2) = gyro_var * dt * dt;
    process_noise(3, 3) = velocity_var * dt;
    process_noise(4, 4) = bias_var * dt;
    covariance_ = transition * covariance_ * transition.transpose() + process_noise;
  }

  void wheel_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double measured_velocity = msg->twist.twist.linear.x;
    const rclcpp::Time stamp(msg->header.stamp);
    push_sample(wheel_history_, stamp, measured_velocity);
    latest_wheel_speed_ = measured_velocity;
    have_wheel_speed_ = true;
    latest_wheel_odom_pose_.x = msg->pose.pose.position.x;
    latest_wheel_odom_pose_.y = msg->pose.pose.position.y;
    tf2::Quaternion wheel_quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double wheel_roll = 0.0;
    double wheel_pitch = 0.0;
    tf2::Matrix3x3(wheel_quaternion).getRPY(
      wheel_roll, wheel_pitch, latest_wheel_odom_pose_.yaw);
    have_wheel_odom_pose_ = true;

    Eigen::Matrix<double, 1, 5> observation;
    observation.setZero();
    observation(0, 3) = 1.0;
    const double innovation = measured_velocity - state_(3);
    const double variance = wheel_velocity_std_ * wheel_velocity_std_;
    const double innovation_variance =
      (observation * covariance_ * observation.transpose())(0, 0) + variance;
    const Eigen::Matrix<double, 5, 1> gain =
      covariance_ * observation.transpose() / innovation_variance;
    state_ += gain * innovation;
    covariance_ =
      (Covariance::Identity() - gain * observation) * covariance_;
    if (try_process_pending_scans()) {
      publish_state(have_imu_stamp_ ? last_imu_stamp_ : stamp);
    }
  }

  void push_sample(
    std::deque<TimedSample> & history, const rclcpp::Time & stamp, double value)
  {
    if (!history.empty() && stamp < history.back().stamp) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "丢弃时间戳乱序的运动数据");
      return;
    }
    if (!history.empty() && stamp == history.back().stamp) {
      history.back().value = value;
    } else {
      history.push_back({stamp, value});
    }
    const int64_t keep_after = stamp.nanoseconds() -
      static_cast<int64_t>(motion_buffer_duration_ * 1e9);
    while (history.size() > 2U && history[1].stamp.nanoseconds() < keep_after) {
      history.pop_front();
    }
  }

  bool interpolate_sample(
    const std::deque<TimedSample> & history, const rclcpp::Time & stamp,
    double & value) const
  {
    if (history.empty() || stamp < history.front().stamp || stamp > history.back().stamp) {
      return false;
    }
    const auto upper = std::lower_bound(
      history.begin(), history.end(), stamp,
      [](const TimedSample & sample, const rclcpp::Time & time) {
        return sample.stamp < time;
      });
    if (upper == history.begin()) {
      value = upper->value;
      return true;
    }
    if (upper == history.end()) {
      value = history.back().value;
      return true;
    }
    if (upper->stamp == stamp) {
      value = upper->value;
      return true;
    }
    const auto lower = std::prev(upper);
    const double interval = (upper->stamp - lower->stamp).seconds();
    if (interval <= 0.0) {
      return false;
    }
    const double alpha = (stamp - lower->stamp).seconds() / interval;
    value = lower->value + alpha * (upper->value - lower->value);
    return true;
  }

  bool motion_coverage(const rclcpp::Time & from, const rclcpp::Time & to) const
  {
    return !imu_history_.empty() && !wheel_history_.empty() &&
      imu_history_.front().stamp <= from && imu_history_.back().stamp >= to &&
      wheel_history_.front().stamp <= from && wheel_history_.back().stamp >= to &&
      sample_gaps_acceptable(imu_history_, from, to) &&
      sample_gaps_acceptable(wheel_history_, from, to);
  }

  bool sample_gaps_acceptable(
    const std::deque<TimedSample> & history,
    const rclcpp::Time & from, const rclcpp::Time & to) const
  {
    if (history.size() < 2U) {
      return false;
    }
    auto next = std::upper_bound(
      history.begin(), history.end(), from,
      [](const rclcpp::Time & time, const TimedSample & sample) {
        return time < sample.stamp;
      });
    if (next == history.begin() || next == history.end()) {
      return false;
    }
    auto previous = std::prev(next);
    while (previous->stamp < to) {
      if (next == history.end() ||
        (next->stamp - previous->stamp).seconds() > motion_max_sample_gap_)
      {
        return false;
      }
      previous = next;
      ++next;
    }
    return true;
  }

  bool integrate_motion(
    const rclcpp::Time & from, const rclcpp::Time & to, Pose2d & motion) const
  {
    motion = Pose2d{};
    if (to <= from) {
      return true;
    }
    if (!motion_coverage(from, to)) {
      return false;
    }

    std::vector<rclcpp::Time> knots;
    knots.reserve(imu_history_.size() + wheel_history_.size() + 2U);
    knots.push_back(from);
    for (const auto & sample : imu_history_) {
      if (sample.stamp > from && sample.stamp < to) {
        knots.push_back(sample.stamp);
      }
    }
    for (const auto & sample : wheel_history_) {
      if (sample.stamp > from && sample.stamp < to) {
        knots.push_back(sample.stamp);
      }
    }
    knots.push_back(to);
    std::sort(
      knots.begin(), knots.end(),
      [](const rclcpp::Time & lhs, const rclcpp::Time & rhs) {return lhs < rhs;});
    knots.erase(
      std::unique(
        knots.begin(), knots.end(),
        [](const rclcpp::Time & lhs, const rclcpp::Time & rhs) {return lhs == rhs;}),
      knots.end());

    for (std::size_t i = 1U; i < knots.size(); ++i) {
      const double dt = (knots[i] - knots[i - 1U]).seconds();
      double gyro_begin = 0.0;
      double gyro_end = 0.0;
      double velocity_begin = 0.0;
      double velocity_end = 0.0;
      if (dt <= 0.0 ||
        !interpolate_sample(imu_history_, knots[i - 1U], gyro_begin) ||
        !interpolate_sample(imu_history_, knots[i], gyro_end) ||
        !interpolate_sample(wheel_history_, knots[i - 1U], velocity_begin) ||
        !interpolate_sample(wheel_history_, knots[i], velocity_end))
      {
        return false;
      }

      const double angular_velocity = 0.5 * (gyro_begin + gyro_end) - state_(4);
      const double velocity = 0.5 * (velocity_begin + velocity_end);
      const double delta_yaw = angular_velocity * dt;
      Pose2d step;
      if (std::abs(angular_velocity) > 1e-6) {
        step.x = velocity * std::sin(delta_yaw) / angular_velocity;
        step.y = velocity * (1.0 - std::cos(delta_yaw)) / angular_velocity;
      } else {
        step.x = velocity * dt;
        step.y = 0.0;
      }
      step.yaw = delta_yaw;
      motion = compose(motion, step);
    }
    return true;
  }

  ScanTiming get_scan_timing(const sensor_msgs::msg::LaserScan & scan) const
  {
    const rclcpp::Time message_stamp(scan.header.stamp);
    const double point_period = scan.time_increment > 0.0 ?
      scan.time_increment :
      (scan.ranges.size() > 1U ? scan.scan_time / static_cast<double>(scan.ranges.size() - 1U) : 0.0);
    const int64_t stamp_offset_ns = static_cast<int64_t>(scan_stamp_offset_ * 1e9);
    const rclcpp::Duration stamp_offset = rclcpp::Duration::from_nanoseconds(stamp_offset_ns);

    ScanTiming timing{message_stamp, message_stamp, message_stamp, point_period, 0.0};
    if (scan_index_time_reversed_) {
      double normalized_min_angle = std::fmod(static_cast<double>(scan.angle_min), 2.0 * kPi);
      if (normalized_min_angle < 0.0) {
        normalized_min_angle += 2.0 * kPi;
      }
      timing.index_zero_to_end = scan.scan_time > 0.0 ?
        normalized_min_angle * scan.scan_time / (2.0 * kPi) : 0.0;
      timing.scan_end = message_stamp + stamp_offset;
      timing.first_point = timing.scan_end - rclcpp::Duration::from_seconds(
        timing.index_zero_to_end +
        point_period * static_cast<double>(scan.ranges.size() > 0U ? scan.ranges.size() - 1U : 0U));
      timing.last_point = timing.scan_end -
        rclcpp::Duration::from_seconds(timing.index_zero_to_end);
    } else {
      timing.first_point = message_stamp + stamp_offset;
      timing.last_point = timing.first_point + rclcpp::Duration::from_seconds(
        point_period * static_cast<double>(scan.ranges.size() > 0U ? scan.ranges.size() - 1U : 0U));
      timing.scan_end = scan_stamp_is_end_ ? message_stamp + stamp_offset : timing.last_point;
    }
    return timing;
  }

  rclcpp::Time point_stamp(const ScanTiming & timing, std::size_t index) const
  {
    if (scan_index_time_reversed_) {
      return timing.scan_end - rclcpp::Duration::from_seconds(
        timing.index_zero_to_end + timing.point_period * static_cast<double>(index));
    }
    return timing.first_point +
      rclcpp::Duration::from_seconds(timing.point_period * static_cast<double>(index));
  }

  std::vector<Eigen::Vector2d> scan_to_points(
    const sensor_msgs::msg::LaserScan & scan, const ScanTiming & timing) const
  {
    std::vector<Eigen::Vector2d> points;
    points.reserve(scan.ranges.size() / static_cast<std::size_t>(scan_stride_) + 1U);

    double deskew_velocity = 0.0;
    double deskew_angular_velocity = 0.0;
    const double scan_duration = (timing.scan_end - timing.first_point).seconds();
    if (deskew_scan_ && timing.point_period > 0.0 && scan_duration > 0.0) {
      Pose2d full_scan_motion;
      if (!integrate_motion(timing.first_point, timing.scan_end, full_scan_motion)) {
        return points;
      }
      deskew_angular_velocity = full_scan_motion.yaw / scan_duration;
      const double chord = std::hypot(full_scan_motion.x, full_scan_motion.y);
      double arc = chord;
      const double half_yaw = 0.5 * std::abs(full_scan_motion.yaw);
      if (half_yaw > 1e-5 && std::abs(std::sin(half_yaw)) > 1e-6) {
        arc *= half_yaw / std::abs(std::sin(half_yaw));
      }
      deskew_velocity = std::copysign(arc / scan_duration, full_scan_motion.x);
    }

    for (std::size_t i = 0; i < scan.ranges.size(); i += static_cast<std::size_t>(scan_stride_)) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan_min_range_ || range > scan_max_range_) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      Eigen::Vector2d point_laser(range * std::cos(angle), range * std::sin(angle));
      Eigen::Vector2d point_base = transform_point(laser_extrinsic_, point_laser);

      if (deskew_scan_ && timing.point_period > 0.0) {
        const double dt = std::max(0.0, (timing.scan_end - point_stamp(timing, i)).seconds());
        Pose2d point_to_end;
        const double delta_yaw = deskew_angular_velocity * dt;
        if (std::abs(deskew_angular_velocity) > 1e-6) {
          point_to_end.x = deskew_velocity * std::sin(delta_yaw) / deskew_angular_velocity;
          point_to_end.y = deskew_velocity * (1.0 - std::cos(delta_yaw)) /
            deskew_angular_velocity;
        } else {
          point_to_end.x = deskew_velocity * dt;
        }
        point_to_end.yaw = delta_yaw;
        point_base = transform_point(inverse(point_to_end), point_base);
      }
      points.push_back(point_base);
    }
    return points;
  }

  IcpResult align_scan(
    const std::vector<Eigen::Vector2d> & source,
    const std::vector<Eigen::Vector2d> & target,
    Pose2d initial_guess) const
  {
    IcpResult result;
    if (source.size() < 20U || target.size() < 20U) {
      return result;
    }

    Pose2d estimate = initial_guess;
    const double max_distance_sq = icp_max_correspondence_ * icp_max_correspondence_;

    for (int iteration = 0; iteration < icp_max_iterations_; ++iteration) {
      Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      double squared_error_sum = 0.0;
      int correspondences = 0;

      const double c = std::cos(estimate.yaw);
      const double s = std::sin(estimate.yaw);
      for (const auto & source_point : source) {
        const Eigen::Vector2d rotated(
          c * source_point.x() - s * source_point.y(),
          s * source_point.x() + c * source_point.y());
        const Eigen::Vector2d transformed =
          rotated + Eigen::Vector2d(estimate.x, estimate.y);

        std::size_t nearest_index = 0U;
        double nearest_distance_sq = max_distance_sq;
        for (std::size_t j = 1U; j + 1U < target.size(); ++j) {
          const double distance_sq = (target[j] - transformed).squaredNorm();
          if (distance_sq < nearest_distance_sq) {
            nearest_distance_sq = distance_sq;
            nearest_index = j;
          }
        }
        if (nearest_index == 0U) {
          continue;
        }

        const Eigen::Vector2d before = target[nearest_index - 1U];
        const Eigen::Vector2d after = target[nearest_index + 1U];
        if ((target[nearest_index] - before).norm() > icp_normal_max_gap_ ||
          (after - target[nearest_index]).norm() > icp_normal_max_gap_)
        {
          continue;
        }
        Eigen::Vector2d tangent = after - before;
        const double tangent_norm = tangent.norm();
        if (tangent_norm < 1e-6) {
          continue;
        }
        tangent /= tangent_norm;
        const Eigen::Vector2d normal(-tangent.y(), tangent.x());
        const double residual = normal.dot(transformed - target[nearest_index]);
        const double absolute_residual = std::abs(residual);
        const double weight = absolute_residual <= icp_huber_delta_ ?
          1.0 : icp_huber_delta_ / absolute_residual;

        Eigen::Vector3d jacobian;
        jacobian << normal.x(), normal.y(), normal.dot(Eigen::Vector2d(-rotated.y(), rotated.x()));
        hessian.noalias() += weight * jacobian * jacobian.transpose();
        gradient.noalias() += weight * jacobian * residual;
        squared_error_sum += residual * residual;
        ++correspondences;
      }

      result.correspondences = correspondences;
      if (correspondences < icp_min_correspondences_) {
        return result;
      }

      const Eigen::LDLT<Eigen::Matrix3d> decomposition(hessian);
      if (decomposition.info() != Eigen::Success) {
        return result;
      }
      const Eigen::Vector3d increment = decomposition.solve(-gradient);
      if (!increment.allFinite()) {
        return result;
      }
      estimate.x += increment.x();
      estimate.y += increment.y();
      estimate.yaw = normalize_angle(estimate.yaw + increment.z());
      result.rmse = std::sqrt(squared_error_sum / static_cast<double>(correspondences));

      if (increment.head<2>().norm() < icp_translation_convergence_ &&
        std::abs(increment.z()) < icp_rotation_convergence_)
      {
        break;
      }
    }

    result.valid = result.correspondences >= icp_min_correspondences_ &&
      std::isfinite(result.rmse) && result.rmse <= icp_max_rmse_;
    result.relative_pose = estimate;
    return result;
  }

  static std::int64_t voxel_key(int x, int y)
  {
    const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
    const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
    return static_cast<std::int64_t>((ux << 32U) | uy);
  }

  std::pair<int, int> voxel_index(const Eigen::Vector2d & point) const
  {
    return {
      static_cast<int>(std::floor(point.x() / ndt_voxel_size_)),
      static_cast<int>(std::floor(point.y() / ndt_voxel_size_))};
  }

  void rebuild_ndt_submap()
  {
    std::unordered_map<std::int64_t, VoxelAccumulator> accumulators;
    for (const auto & keyframe : submap_keyframes_) {
      for (const auto & point : keyframe) {
        const auto [ix, iy] = voxel_index(point);
        const std::int64_t key = voxel_key(ix, iy);
        auto found = accumulators.find(key);
        if (found == accumulators.end()) {
          if (static_cast<int>(accumulators.size()) >= ndt_voxel_capacity_) {
            continue;
          }
          found = accumulators.emplace(key, VoxelAccumulator{}).first;
        }
        auto & accumulator = found->second;
        if (accumulator.count >= ndt_max_points_per_voxel_) {
          continue;
        }
        ++accumulator.count;
        accumulator.sum += point;
        accumulator.sum_outer.noalias() += point * point.transpose();
      }
    }

    ndt_voxels_.clear();
    ndt_voxels_.reserve(accumulators.size());
    for (const auto & [key, accumulator] : accumulators) {
      if (accumulator.count < ndt_min_points_per_voxel_) {
        continue;
      }
      NdtVoxel voxel;
      voxel.count = accumulator.count;
      voxel.mean = accumulator.sum / static_cast<double>(accumulator.count);
      Eigen::Matrix2d covariance =
        accumulator.sum_outer / static_cast<double>(accumulator.count) -
        voxel.mean * voxel.mean.transpose();
      covariance += ndt_regularization_ * Eigen::Matrix2d::Identity();
      const Eigen::LDLT<Eigen::Matrix2d> decomposition(covariance);
      if (decomposition.info() != Eigen::Success) {
        continue;
      }
      voxel.information = decomposition.solve(Eigen::Matrix2d::Identity());
      if (voxel.information.allFinite()) {
        ndt_voxels_.emplace(key, voxel);
      }
    }
    publish_submap();
  }

  void add_keyframe(const std::vector<Eigen::Vector2d> & scan, const Pose2d & pose)
  {
    std::vector<Eigen::Vector2d> global_points;
    global_points.reserve(scan.size());
    for (const auto & point : scan) {
      global_points.push_back(transform_point(pose, point));
    }
    submap_keyframes_.push_back(std::move(global_points));
    while (static_cast<int>(submap_keyframes_.size()) > submap_max_keyframes_) {
      submap_keyframes_.pop_front();
    }
    last_keyframe_pose_ = pose;
    have_keyframe_pose_ = true;
    rebuild_ndt_submap();
    RCLCPP_INFO(
      get_logger(), "新增关键帧：总数=%zu，有效 NDT 体素=%zu",
      submap_keyframes_.size(), ndt_voxels_.size());
  }

  bool should_add_keyframe(const Pose2d & pose) const
  {
    if (!have_keyframe_pose_) {
      return true;
    }
    const Pose2d delta = compose(inverse(last_keyframe_pose_), pose);
    return std::hypot(delta.x, delta.y) >= keyframe_distance_ ||
      std::abs(delta.yaw) >= keyframe_angle_;
  }

  const NdtVoxel * nearest_ndt_voxel(
    const Eigen::Vector2d & point, double & mahalanobis) const
  {
    const auto [center_x, center_y] = voxel_index(point);
    const NdtVoxel * best = nullptr;
    mahalanobis = std::numeric_limits<double>::infinity();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (ndt_nearby_type_ == 0 && (dx != 0 || dy != 0)) {
          continue;
        }
        if (ndt_nearby_type_ == 1 && std::abs(dx) + std::abs(dy) > 1) {
          continue;
        }
        const auto found = ndt_voxels_.find(voxel_key(center_x + dx, center_y + dy));
        if (found == ndt_voxels_.end()) {
          continue;
        }
        const Eigen::Vector2d residual = point - found->second.mean;
        const double score = residual.dot(found->second.information * residual);
        if (score < mahalanobis) {
          mahalanobis = score;
          best = &found->second;
        }
      }
    }
    return best;
  }

  NdtResult align_scan_to_submap(
    const std::vector<Eigen::Vector2d> & scan, const Pose2d & prediction) const
  {
    NdtResult result;
    result.pose = prediction;
    if (ndt_voxels_.empty() || scan.size() < 20U) {
      return result;
    }

    Pose2d estimate = prediction;
    for (int iteration = 0; iteration < ndt_max_iterations_; ++iteration) {
      Eigen::Matrix3d hessian = ndt_prior_information_ * Eigen::Matrix3d::Identity();
      Eigen::Vector3d gradient;
      gradient.x() = ndt_prior_information_ * (estimate.x - prediction.x);
      gradient.y() = ndt_prior_information_ * (estimate.y - prediction.y);
      gradient.z() = ndt_prior_information_ *
        normalize_angle(estimate.yaw - prediction.yaw);
      int effective_points = 0;
      double score_sum = 0.0;
      const double c = std::cos(estimate.yaw);
      const double s = std::sin(estimate.yaw);

      for (const auto & point : scan) {
        const Eigen::Vector2d rotated(
          c * point.x() - s * point.y(), s * point.x() + c * point.y());
        const Eigen::Vector2d global = rotated + Eigen::Vector2d(estimate.x, estimate.y);
        double mahalanobis = 0.0;
        const NdtVoxel * voxel = nearest_ndt_voxel(global, mahalanobis);
        if (voxel == nullptr || mahalanobis > ndt_outlier_threshold_) {
          continue;
        }
        const Eigen::Vector2d residual = global - voxel->mean;
        Eigen::Matrix<double, 2, 3> jacobian;
        jacobian << 1.0, 0.0, -rotated.y(), 0.0, 1.0, rotated.x();
        hessian.noalias() += ndt_information_ratio_ *
          jacobian.transpose() * voxel->information * jacobian;
        gradient.noalias() += ndt_information_ratio_ *
          jacobian.transpose() * voxel->information * residual;
        score_sum += mahalanobis;
        ++effective_points;
      }

      result.effective_points = effective_points;
      result.iterations = iteration + 1;
      if (effective_points < ndt_min_effective_points_) {
        return result;
      }
      const Eigen::LDLT<Eigen::Matrix3d> decomposition(hessian);
      if (decomposition.info() != Eigen::Success) {
        return result;
      }
      Eigen::Vector3d increment = decomposition.solve(-gradient);
      if (!increment.allFinite()) {
        return result;
      }
      const double translation_norm = increment.head<2>().norm();
      if (translation_norm > 0.10) {
        increment.head<2>() *= 0.10 / translation_norm;
      }
      increment.z() = std::clamp(increment.z(), -0.10, 0.10);
      estimate.x += increment.x();
      estimate.y += increment.y();
      estimate.yaw = normalize_angle(estimate.yaw + increment.z());
      result.mean_mahalanobis = score_sum / static_cast<double>(effective_points);
      if (increment.norm() <= ndt_convergence_epsilon_) {
        break;
      }
    }
    result.pose = estimate;
    result.valid = result.effective_points >= ndt_min_effective_points_ &&
      std::isfinite(result.mean_mahalanobis);
    return result;
  }

  void publish_submap()
  {
    if (!submap_pub_) {
      return;
    }
    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp = now();
    map.header.frame_id = odom_frame_;
    map.info.resolution = static_cast<float>(1.0 / occupancy_resolution_);
    map.info.width = static_cast<std::uint32_t>(occupancy_image_size_);
    map.info.height = static_cast<std::uint32_t>(occupancy_image_size_);
    map.info.origin.position.x = occupancy_origin_x_;
    map.info.origin.position.y = occupancy_origin_y_;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
      static_cast<std::size_t>(occupancy_image_size_ * occupancy_image_size_), -1);
    for (const auto & [key, voxel] : ndt_voxels_) {
      (void)key;
      const int x = static_cast<int>(
        std::floor((voxel.mean.x() - occupancy_origin_x_) * occupancy_resolution_));
      const int y = static_cast<int>(
        std::floor((voxel.mean.y() - occupancy_origin_y_) * occupancy_resolution_));
      if (x >= 0 && y >= 0 && x < occupancy_image_size_ && y < occupancy_image_size_) {
        map.data[static_cast<std::size_t>(y * occupancy_image_size_ + x)] = 100;
      }
    }
    submap_pub_->publish(map);
  }

  Pose2d pose_at_scan_end(const rclcpp::Time & scan_end) const
  {
    Pose2d scan_to_filter;
    if (motion_from_scan_end_to_filter(scan_end, scan_to_filter))
    {
      return compose(state_pose(), inverse(scan_to_filter));
    }
    return state_pose();
  }

  bool motion_from_scan_end_to_filter(
    const rclcpp::Time & scan_end, Pose2d & motion) const
  {
    motion = Pose2d{};
    if (!have_imu_stamp_ || last_imu_stamp_ <= scan_end) {
      return true;
    }
    if (integrate_motion(scan_end, last_imu_stamp_, motion)) {
      return true;
    }

    // Wheel and IMU callbacks are not phase locked. For the short tail after
    // scan_end, use the current fused velocity and gyro instead of making the
    // pending-scan target move forward on every IMU callback.
    const double dt = (last_imu_stamp_ - scan_end).seconds();
    if (!(dt > 0.0 && dt <= motion_max_sample_gap_)) {
      return false;
    }
    const double angular_velocity = last_gyro_z_ - state_(4);
    const double delta_yaw = angular_velocity * dt;
    if (std::abs(angular_velocity) > 1e-6) {
      motion.x = state_(3) * std::sin(delta_yaw) / angular_velocity;
      motion.y = state_(3) * (1.0 - std::cos(delta_yaw)) / angular_velocity;
    } else {
      motion.x = state_(3) * dt;
    }
    motion.yaw = delta_yaw;
    return true;
  }

  void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr & msg, const ScanTiming & timing)
  {
    const auto matching_start = std::chrono::steady_clock::now();
    const auto points = scan_to_points(*msg, timing);
    if (points.size() < 20U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "有效雷达点过少：%zu", points.size());
      return;
    }

    if (!have_reference_scan_) {
      reference_scan_ = points;
      lidar_pose_ = pose_at_scan_end(timing.scan_end);
      last_scan_end_stamp_ = timing.scan_end;
      have_reference_scan_ = true;
      add_keyframe(points, lidar_pose_);
      RCLCPP_INFO(get_logger(), "NDT-LIO 初始化完成，有效点数：%zu", points.size());
      return;
    }

    const bool is_static = robot_is_static(last_gyro_z_);
    if (is_static) {
      reference_scan_ = points;
      lidar_pose_ = pose_at_scan_end(timing.scan_end);
      last_scan_end_stamp_ = timing.scan_end;
      return;
    }

    const Pose2d predicted_scan_pose = pose_at_scan_end(timing.scan_end);
    const NdtResult ndt = align_scan_to_submap(points, predicted_scan_pose);
    const Pose2d ndt_correction = compose(inverse(predicted_scan_pose), ndt.pose);
    const bool ndt_plausible = ndt.valid &&
      std::hypot(ndt_correction.x, ndt_correction.y) <= ndt_max_translation_correction_ &&
      std::abs(ndt_correction.yaw) <= ndt_max_rotation_correction_;
    bool lidar_update_accepted = false;
    if (ndt_plausible) {
      Pose2d scan_to_filter;
      Pose2d current_measurement = ndt.pose;
      if (motion_from_scan_end_to_filter(timing.scan_end, scan_to_filter)) {
        current_measurement = compose(ndt.pose, scan_to_filter);
      }
      observe_lidar_pose(current_measurement);
      // Mapping must use the filter posterior, not the raw scan-matcher
      // measurement. Otherwise the EKF state and its own submap gradually
      // become two different trajectories whenever the Kalman gain is below 1.
      lidar_pose_ = pose_at_scan_end(timing.scan_end);
      lidar_update_accepted = true;
      ++ndt_accepted_count_;
      RCLCPP_DEBUG(
        get_logger(),
        "NDT 接受：有效点=%d 迭代=%d 平均马氏距离=%.3f 修正=(%.3f %.3f %.3f)",
        ndt.effective_points, ndt.iterations, ndt.mean_mahalanobis,
        ndt_correction.x, ndt_correction.y, ndt_correction.yaw);
    }

    // The first few keyframes may not yet form enough populated NDT voxels.
    // Keep the old scan-to-scan ICP only as a bootstrap/failure fallback.
    Pose2d predicted_relative;
    if (!lidar_update_accepted &&
      !integrate_motion(last_scan_end_stamp_, timing.scan_end, predicted_relative))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "相邻雷达帧之间缺少完整运动数据，使用纯预测位姿");
      reference_scan_ = points;
      lidar_pose_ = predicted_scan_pose;
      last_scan_end_stamp_ = timing.scan_end;
      return;
    }

    if (!lidar_update_accepted) {
      const IcpResult icp = align_scan(points, reference_scan_, predicted_relative);
      const Pose2d prediction_error = compose(inverse(predicted_relative), icp.relative_pose);
      const bool motion_plausible = icp.valid &&
        std::hypot(icp.relative_pose.x, icp.relative_pose.y) <= icp_max_translation_jump_ &&
        std::abs(icp.relative_pose.yaw) <= icp_max_rotation_jump_ &&
        std::hypot(prediction_error.x, prediction_error.y) <=
        icp_max_prediction_translation_error_ &&
        std::abs(prediction_error.yaw) <= icp_max_prediction_rotation_error_;
      if (motion_plausible) {
        const Pose2d lidar_measurement = compose(lidar_pose_, icp.relative_pose);
        Pose2d scan_to_filter;
        Pose2d current_measurement = lidar_measurement;
        if (motion_from_scan_end_to_filter(timing.scan_end, scan_to_filter)) {
          current_measurement = compose(lidar_measurement, scan_to_filter);
        }
        observe_lidar_pose(current_measurement);
        lidar_pose_ = pose_at_scan_end(timing.scan_end);
        lidar_update_accepted = true;
        ++icp_accepted_count_;
        RCLCPP_DEBUG(
          get_logger(), "启动兜底 ICP 接受：点=%d rmse=%.4f", icp.correspondences, icp.rmse);
      } else {
        lidar_pose_ = predicted_scan_pose;
        ++lidar_rejected_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NDT/ICP 均拒绝：NDT点=%d NDT修正=(%.3f %.3f %.3f) ICP点=%d rmse=%.4f",
          ndt.effective_points, ndt_correction.x, ndt_correction.y, ndt_correction.yaw,
          icp.correspondences, icp.rmse);
      }
    }

    reference_scan_ = points;
    last_scan_end_stamp_ = timing.scan_end;
    if (lidar_update_accepted && should_add_keyframe(lidar_pose_)) {
      add_keyframe(points, lidar_pose_);
    }
    const double matching_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - matching_start).count();
    matching_time_sum_ms_ += matching_ms;
    ++matching_count_;
    matching_time_max_ms_ = std::max(matching_time_max_ms_, matching_ms);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "定位统计：NDT接受=%zu ICP兜底=%zu 双重拒绝=%zu "
      "当前NDT点=%d 迭代=%d 修正=(%.3f %.3f %.3f) "
      "匹配耗时=%.1fms 平均=%.1fms 最大=%.1fms",
      ndt_accepted_count_, icp_accepted_count_, lidar_rejected_count_,
      ndt.effective_points, ndt.iterations,
      ndt_correction.x, ndt_correction.y, ndt_correction.yaw,
      matching_ms, matching_time_sum_ms_ / static_cast<double>(matching_count_),
      matching_time_max_ms_);
  }

  bool try_process_pending_scans()
  {
    bool processed = false;
    while (!pending_scans_.empty()) {
      const auto & scan = pending_scans_.front();
      const ScanTiming timing = get_scan_timing(*scan);
      if (imu_history_.empty() || wheel_history_.empty()) {
        return processed;
      }
      if (imu_history_.front().stamp > timing.first_point ||
        wheel_history_.front().stamp > timing.first_point)
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "丢弃扫描：扫描起始时刻缺少运动数据");
        pending_scans_.pop_front();
        continue;
      }
      const rclcpp::Time required_end = timing.scan_end;
      if (imu_history_.back().stamp < required_end ||
        wheel_history_.back().stamp < required_end)
      {
        return processed;
      }
      if (!motion_coverage(timing.first_point, required_end)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "丢弃扫描：扫描期间 IMU 或轮速数据间隔过大");
        pending_scans_.pop_front();
        continue;
      }
      process_scan(scan, timing);
      pending_scans_.pop_front();
      processed = true;
    }
    return processed;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    pending_scans_.push_back(msg);
    while (static_cast<int>(pending_scans_.size()) > max_pending_scans_) {
      pending_scans_.pop_front();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "等待 IMU/轮速覆盖时队列已满，丢弃最旧扫描");
    }
    if (try_process_pending_scans() && have_imu_stamp_) {
      publish_state(last_imu_stamp_);
    }
  }

  void observe_lidar_pose(const Pose2d & measurement)
  {
    Eigen::Matrix<double, 3, 5> observation = Eigen::Matrix<double, 3, 5>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;
    observation(2, 2) = 1.0;

    Eigen::Vector3d innovation;
    innovation << measurement.x - state_(0), measurement.y - state_(1),
      normalize_angle(measurement.yaw - state_(2));

    Eigen::Matrix3d measurement_noise = Eigen::Matrix3d::Zero();
    measurement_noise(0, 0) = lidar_position_std_ * lidar_position_std_;
    measurement_noise(1, 1) = lidar_position_std_ * lidar_position_std_;
    measurement_noise(2, 2) = lidar_yaw_std_ * lidar_yaw_std_;

    const Eigen::Matrix3d innovation_covariance =
      observation * covariance_ * observation.transpose() + measurement_noise;
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
    if (decomposition.info() != Eigen::Success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "拒绝雷达更新：新息协方差矩阵奇异");
      return;
    }
    const Eigen::Matrix<double, 5, 3> cross_covariance =
      covariance_ * observation.transpose();
    const Eigen::Matrix<double, 5, 3> gain =
      decomposition.solve(cross_covariance.transpose()).transpose();
    state_ += gain * innovation;
    state_(2) = normalize_angle(state_(2));

    const Covariance identity = Covariance::Identity();
    const Covariance correction = identity - gain * observation;
    covariance_ = correction * covariance_ * correction.transpose() +
      gain * measurement_noise * gain.transpose();
  }

  void publish_state(const rclcpp::Time & stamp)
  {
    if (have_published_stamp_ && stamp <= last_published_stamp_) {
      return;
    }
    last_published_stamp_ = stamp;
    have_published_stamp_ = true;
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = state_(0);
    odometry.pose.pose.position.y = state_(1);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, state_(2));
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    odometry.twist.twist.linear.x = state_(3);
    odometry.twist.twist.angular.z = last_gyro_z_ - state_(4);

    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[0] = covariance_(0, 0);
    odometry.pose.covariance[1] = covariance_(0, 1);
    odometry.pose.covariance[5] = covariance_(0, 2);
    odometry.pose.covariance[6] = covariance_(1, 0);
    odometry.pose.covariance[7] = covariance_(1, 1);
    odometry.pose.covariance[11] = covariance_(1, 2);
    odometry.pose.covariance[30] = covariance_(2, 0);
    odometry.pose.covariance[31] = covariance_(2, 1);
    odometry.pose.covariance[35] = covariance_(2, 2);
    odometry.pose.covariance[14] = 1e6;
    odometry.pose.covariance[21] = 1e6;
    odometry.pose.covariance[28] = 1e6;
    odometry.twist.covariance.fill(0.0);
    odometry.twist.covariance[0] = covariance_(3, 3);
    odometry.twist.covariance[35] = gyro_noise_std_ * gyro_noise_std_ + covariance_(4, 4);
    odom_pub_->publish(odometry);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = state_(0);
      transform.transform.translation.y = state_(1);
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
    if (publish_odom_bridge_tf_ && have_wheel_odom_pose_) {
      // Preserve the chassis' existing odom->base_footprint transform. The
      // bridge makes lio_ob_odom->odom such that their composition equals
      // the LIO estimate lio_ob_odom->base_footprint.
      const Pose2d lio_base{state_(0), state_(1), state_(2)};
      const Pose2d lio_odom = compose(lio_base, inverse(latest_wheel_odom_pose_));
      geometry_msgs::msg::TransformStamped bridge;
      bridge.header.stamp = stamp;
      bridge.header.frame_id = odom_frame_;
      bridge.child_frame_id = wheel_odom_frame_;
      bridge.transform.translation.x = lio_odom.x;
      bridge.transform.translation.y = lio_odom.y;
      tf2::Quaternion bridge_quaternion;
      bridge_quaternion.setRPY(0.0, 0.0, lio_odom.yaw);
      bridge.transform.rotation.x = bridge_quaternion.x();
      bridge.transform.rotation.y = bridge_quaternion.y();
      bridge.transform.rotation.z = bridge_quaternion.z();
      bridge.transform.rotation.w = bridge_quaternion.w();
      tf_broadcaster_->sendTransform(bridge);
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = odometry.header;
    pose.pose = odometry.pose.pose;
    bool append_path_pose = path_.poses.empty();
    if (!append_path_pose) {
      const auto & previous = path_.poses.back().pose;
      const double position_delta = std::hypot(
        pose.pose.position.x - previous.position.x,
        pose.pose.position.y - previous.position.y);
      const double previous_yaw = std::atan2(
        2.0 * (previous.orientation.w * previous.orientation.z +
        previous.orientation.x * previous.orientation.y),
        1.0 - 2.0 * (previous.orientation.y * previous.orientation.y +
        previous.orientation.z * previous.orientation.z));
      const double angle_delta = std::abs(normalize_angle(state_(2) - previous_yaw));
      append_path_pose = position_delta >= path_pose_min_distance_ ||
        angle_delta >= path_pose_min_angle_;
    }
    if (append_path_pose) {
      path_.poses.push_back(pose);
      path_dirty_ = true;
      while (static_cast<int>(path_.poses.size()) > path_max_poses_) {
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + path_max_poses_ / 4);
      }
    }
    if (path_dirty_ &&
      (!have_path_publish_stamp_ ||
      (stamp - last_path_publish_stamp_).seconds() >= path_publish_period_))
    {
      path_.header.stamp = stamp;
      path_pub_->publish(path_);
      last_path_publish_stamp_ = stamp;
      have_path_publish_stamp_ = true;
      path_dirty_ = false;
    }
  }

  std::string imu_topic_;
  std::string wheel_odom_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{false};
  bool publish_odom_bridge_tf_{true};
  std::string wheel_odom_frame_{"odom"};
  bool enable_ring_y_correction_{true};
  std::string ring_y_correction_topic_;
  double ring_y_correction_gain_{0.90};
  double ring_y_correction_max_innovation_{0.35};
  double ring_y_correction_min_std_{0.015};
  double ring_y_correction_max_std_{0.10};

    Pose2d laser_extrinsic_;
    bool deskew_scan_{true};
    bool scan_index_time_reversed_{true};
    bool scan_stamp_is_end_{true};
    double scan_stamp_offset_{0.0};
    double motion_buffer_duration_{2.0};
    double motion_max_sample_gap_{0.12};
    int max_pending_scans_{2};
  double scan_min_range_{0.25};
  double scan_max_range_{12.0};
  int scan_stride_{4};
  int icp_max_iterations_{8};
  double icp_max_correspondence_{0.35};
  double icp_normal_max_gap_{0.30};
  double icp_huber_delta_{0.08};
  int icp_min_correspondences_{30};
  double icp_max_rmse_{0.10};
  double icp_translation_convergence_{1e-4};
  double icp_rotation_convergence_{1e-4};
  double icp_max_translation_jump_{0.20};
  double icp_max_rotation_jump_{0.35};
  double icp_max_prediction_translation_error_{0.10};
  double icp_max_prediction_rotation_error_{0.10};
  double keyframe_distance_{0.20};
  double keyframe_angle_{kPi / 6.0};
  int submap_max_keyframes_{50};
  int ndt_max_iterations_{20};
  double ndt_voxel_size_{0.20};
  int ndt_min_effective_points_{10};
  int ndt_min_points_per_voxel_{3};
  int ndt_max_points_per_voxel_{50};
  double ndt_outlier_threshold_{5.991};
  int ndt_voxel_capacity_{10000};
  double ndt_regularization_{1e-3};
  double ndt_prior_information_{1e3};
  double ndt_information_ratio_{0.01};
  double ndt_convergence_epsilon_{1e-3};
  int ndt_nearby_type_{2};
  double ndt_max_translation_correction_{0.25};
  double ndt_max_rotation_correction_{0.20};
  double occupancy_resolution_{20.0};
  int occupancy_image_size_{200};
  double occupancy_origin_x_{-2.5};
  double occupancy_origin_y_{-2.5};
  double path_publish_period_{0.20};
  double path_pose_min_distance_{0.01};
  double path_pose_min_angle_{0.01};
  int path_max_poses_{2000};
  double static_motion_velocity_threshold_{0.03};
  double static_motion_gyro_threshold_{0.01};

    double imu_min_dt_{0.01};
  double imu_max_dt_{0.15};
  double imu_integration_step_{0.05};
  double stationary_bias_alpha_{0.05};
  bool use_imu_orientation_{false};
  double imu_yaw_std_{0.05};
  bool have_imu_orientation_reference_{false};
  double initial_imu_orientation_yaw_{0.0};
  double imu_orientation_anchor_yaw_{0.0};

  double gyro_noise_std_{0.003};
  double gyro_bias_walk_std_{0.0002};
  double velocity_walk_std_{0.20};
  double wheel_velocity_std_{0.06};
  double lidar_position_std_{0.04};
  double lidar_yaw_std_{0.05};

  State state_;
  Covariance covariance_;
  double last_gyro_z_{0.0};
  double latest_wheel_speed_{0.0};
  bool have_wheel_speed_{false};
  Pose2d latest_wheel_odom_pose_;
  bool have_wheel_odom_pose_{false};
  bool have_imu_stamp_{false};
    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_published_stamp_{0, 0, RCL_ROS_TIME};
    bool have_published_stamp_{false};
    std::deque<TimedSample> imu_history_;
    std::deque<TimedSample> wheel_history_;
    std::deque<sensor_msgs::msg::LaserScan::SharedPtr> pending_scans_;

    bool have_reference_scan_{false};
    std::vector<Eigen::Vector2d> reference_scan_;
    Pose2d lidar_pose_;
    rclcpp::Time last_scan_end_stamp_{0, 0, RCL_ROS_TIME};
    std::deque<std::vector<Eigen::Vector2d>> submap_keyframes_;
    std::unordered_map<std::int64_t, NdtVoxel> ndt_voxels_;
    Pose2d last_keyframe_pose_;
    bool have_keyframe_pose_{false};
    std::size_t ndt_accepted_count_{0U};
    std::size_t icp_accepted_count_{0U};
    std::size_t lidar_rejected_count_{0U};
    std::size_t matching_count_{0U};
    double matching_time_sum_ms_{0.0};
    double matching_time_max_ms_{0.0};

  nav_msgs::msg::Path path_;
  bool path_dirty_{false};
  bool have_path_publish_stamp_{false};
  rclcpp::Time last_path_publish_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ring_y_correction_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr submap_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace lio_ob

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lio_ob::SubmapLioNode>());
  rclcpp::shutdown();
  return 0;
}
