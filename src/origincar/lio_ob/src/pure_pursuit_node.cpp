#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "origincar_msg/msg/sign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "lio_ob/math_utils.hpp"

namespace lio_ob
{

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode()
  : Node("lio_ob_controller")
  {
    path_file_ = declare_parameter<std::string>("path_file", "");
    enable_qr_mission_ = declare_parameter<bool>("enable_qr_mission", true);
    qr_sprint_path_file_ = declare_parameter<std::string>("qr_sprint_path_file", "");
    qr_clockwise_path_file_ = declare_parameter<std::string>("qr_clockwise_path_file", "");
    qr_counterclockwise_path_file_ =
      declare_parameter<std::string>("qr_counterclockwise_path_file", "");
    qr_sign_topic_ = declare_parameter<std::string>("qr_sign_topic", "/sign_switch");
    qr_allow_keepout_ = declare_parameter<bool>("qr_allow_keepout", false);
    qr_detection_arm_delay_ = std::max(
      0.0, declare_parameter<double>("qr_detection_arm_delay", 0.50));
    qr_switch_stop_speed_ = std::max(
      0.01, declare_parameter<double>("qr_switch_stop_speed", 0.04));
    qr_switch_max_brake_time_ = std::max(
      0.5, declare_parameter<double>("qr_switch_max_brake_time", 2.0));
    first_line_mode_ = declare_parameter<std::string>("first_line_mode", "lio");
    control_started_topic_ = declare_parameter<std::string>(
      "control_started_topic", "/origincar_control/started");
    control_released_topic_ = declare_parameter<std::string>(
      "control_released_topic", "/origincar_control/control_released");
    visual_first_line_cmd_topic_ = declare_parameter<std::string>(
      "visual_first_line_cmd_topic", "/lio_ob/visual_first_line_cmd");
    visual_first_line_cmd_timeout_ = std::max(
      0.05, declare_parameter<double>("visual_first_line_cmd_timeout", 0.30));
    if (first_line_mode_ != "lio" && first_line_mode_ != "vision" &&
      first_line_mode_ != "external")
    {
      RCLCPP_FATAL(
        get_logger(), "first_line_mode must be 'lio', 'vision' or 'external', got '%s'",
        first_line_mode_.c_str());
      throw std::runtime_error("invalid first_line_mode");
    }
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lio_ob/odom");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    vision_topic_ = declare_parameter<std::string>("vision_topic", "/win_detection");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    path_frame_ = declare_parameter<std::string>("path_frame", "lio_ob_odom");
    align_path_to_start_ = declare_parameter<bool>("align_path_to_start", true);
    ring_path_start_point_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("ring_path_start_point", 78)));
    ring_path_end_point_ = std::max(
      ring_path_start_point_,
      static_cast<int>(declare_parameter<std::int64_t>("ring_path_end_point", 159)));
    ring_path_y_offset_ = declare_parameter<double>("ring_path_y_offset", 0.0);
    ring_path_blend_points_ = std::max(
      0, static_cast<int>(declare_parameter<std::int64_t>("ring_path_blend_points", 4)));
    enable_ring_photo_trigger_ =
      declare_parameter<bool>("enable_ring_photo_trigger", true);
    ring_photo_trigger_topic_ = declare_parameter<std::string>(
      "ring_photo_trigger_topic", "/take_photo/trigger");
    ring_photo_forward_path_point_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_photo_forward_path_point", 125)));
    ring_photo_reverse_path_point_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_photo_reverse_path_point", 120)));
    ring_photo_max_abs_yaw_rate_ = std::max(
      0.0, declare_parameter<double>("ring_photo_max_abs_yaw_rate", 0.25));
    ring_photo_force_after_points_ = std::max(
      0, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_photo_force_after_points", 5)));
    use_keepout_mask_ = declare_parameter<bool>("use_keepout_mask", false);
    keepout_mask_file_ = declare_parameter<std::string>("keepout_mask_file", "");
    keepout_resolution_ = declare_parameter<double>("keepout_resolution", 0.0);
    keepout_origin_.x = declare_parameter<double>("keepout_origin_x", 0.0);
    keepout_origin_.y = declare_parameter<double>("keepout_origin_y", 0.0);
    keepout_origin_.yaw = declare_parameter<double>("keepout_origin_yaw", 0.0);
    keepout_white_threshold_ = static_cast<int>(
      declare_parameter<std::int64_t>("keepout_white_threshold", 128));
    max_path_mask_violation_ratio_ =
      declare_parameter<double>("max_path_mask_violation_ratio", 0.10);

    control_frequency_ = declare_parameter<double>("control_frequency", 30.0);
    min_lookahead_ = declare_parameter<double>("min_lookahead", 0.42);
    max_lookahead_ = declare_parameter<double>("max_lookahead", 0.95);
    lookahead_speed_gain_ = declare_parameter<double>("lookahead_speed_gain", 0.42);
    max_forward_speed_ = declare_parameter<double>("max_forward_speed", 0.50);
    max_reverse_speed_ = declare_parameter<double>("max_reverse_speed", 0.30);
    post_qr_transition_forward_speed_ = std::max(
      0.05, declare_parameter<double>("post_qr_transition_forward_speed", 0.28));
    post_qr_transition_reverse_speed_ = std::max(
      0.05, declare_parameter<double>("post_qr_transition_reverse_speed", 0.28));
    post_qr_transition_end_y_ = declare_parameter<double>(
      "post_qr_transition_end_y", 1.90);
    auto_select_post_qr_route_ = declare_parameter<bool>(
      "auto_select_post_qr_route", false);
    auto_route_obstacle_influence_radius_ = std::max(
      0.10, declare_parameter<double>("auto_route_obstacle_influence_radius", 0.55));
    auto_route_clockwise_path_files_ = declare_parameter<std::vector<std::string>>(
      "auto_route_clockwise_path_files", std::vector<std::string>{});
    auto_route_counterclockwise_path_files_ = declare_parameter<std::vector<std::string>>(
      "auto_route_counterclockwise_path_files", std::vector<std::string>{});
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 2.50);
    max_lateral_acceleration_ = declare_parameter<double>("max_lateral_acceleration", 2.20);
    max_acceleration_ = declare_parameter<double>("max_acceleration", 2.00);
    max_deceleration_ = declare_parameter<double>("max_deceleration", 0.35);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.12);
    cross_track_slowdown_distance_ =
      declare_parameter<double>("cross_track_slowdown_distance", 0.75);
    max_reverse_lookahead_ = declare_parameter<double>("max_reverse_lookahead", 0.35);
    direction_switch_tolerance_ =
      declare_parameter<double>("direction_switch_tolerance", 0.16);
    direction_switch_speed_threshold_ =
      declare_parameter<double>("direction_switch_speed_threshold", 0.04);
    direction_switch_reaction_time_ =
      declare_parameter<double>("direction_switch_reaction_time", 0.75);
    allow_early_reverse_completion_ =
      declare_parameter<bool>("allow_early_reverse_completion", true);
    early_reverse_completion_distance_ =
      declare_parameter<double>("early_reverse_completion_distance", 0.45);

    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);
    max_odom_message_age_ = declare_parameter<double>("max_odom_message_age", 0.30);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.30);
    require_fresh_scan_ = declare_parameter<bool>("require_fresh_scan", true);
    enable_obstacle_avoidance_ =
      declare_parameter<bool>("enable_obstacle_avoidance", true);

    laser_extrinsic_.x = declare_parameter<double>("laser_x", 0.0);
    laser_extrinsic_.y = declare_parameter<double>("laser_y", 0.0);
    laser_extrinsic_.yaw = declare_parameter<double>("laser_yaw", 0.0);
    robot_half_length_ = declare_parameter<double>("robot_half_length", 0.16);
    robot_half_width_ = declare_parameter<double>("robot_half_width", 0.11);
    enable_ring_obstacle_path_filter_ = declare_parameter<bool>(
      "enable_ring_obstacle_path_filter", true);
    ring_obstacle_path_half_width_ = std::max(
      robot_half_width_, declare_parameter<double>("ring_obstacle_path_half_width", 0.30));
    obstacle_corridor_margin_ = declare_parameter<double>("obstacle_corridor_margin", 0.08);
    obstacle_min_trigger_distance_ =
      declare_parameter<double>("obstacle_min_trigger_distance", 0.45);
    obstacle_max_trigger_distance_ =
      declare_parameter<double>("obstacle_max_trigger_distance", 2.20);
    obstacle_reaction_time_ = declare_parameter<double>("obstacle_reaction_time", 0.75);
    obstacle_stop_clearance_ = declare_parameter<double>("obstacle_stop_clearance", 0.10);
    obstacle_avoid_speed_ = declare_parameter<double>("obstacle_avoid_speed", 0.90);
    obstacle_avoid_yaw_rate_ = declare_parameter<double>("obstacle_avoid_yaw_rate", 1.80);
    obstacle_min_avoid_duration_ = declare_parameter<double>("obstacle_min_avoid_duration", 0.70);
    obstacle_clear_duration_ = declare_parameter<double>("obstacle_clear_duration", 0.40);
    enable_reverse_escape_ = declare_parameter<bool>("enable_reverse_escape", true);
    reverse_escape_speed_ = std::max(
      0.05, declare_parameter<double>("reverse_escape_speed", 0.18));
    reverse_escape_distance_ = std::max(
      0.10, declare_parameter<double>("reverse_escape_distance", 0.38));
    reverse_escape_max_duration_ = std::max(
      0.5, declare_parameter<double>("reverse_escape_max_duration", 4.0));
    reverse_escape_max_curvature_ = std::max(
      0.0, declare_parameter<double>("reverse_escape_max_curvature", 2.0));
    reverse_escape_curvature_samples_ = std::max(
      3, static_cast<int>(
        declare_parameter<std::int64_t>("reverse_escape_curvature_samples", 11)));
    reverse_escape_max_attempts_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("reverse_escape_max_attempts", 2)));
    reverse_escape_path_half_width_ = std::max(
      robot_half_width_, declare_parameter<double>("reverse_escape_path_half_width", 0.70));
    reverse_escape_stop_speed_ = std::max(
      0.01, declare_parameter<double>("reverse_escape_stop_speed", 0.04));
    reverse_escape_reset_progress_ = std::max(
      0.05, declare_parameter<double>("reverse_escape_reset_progress", 0.30));
    reverse_escape_reaction_time_ = std::max(
      0.0, declare_parameter<double>("reverse_escape_reaction_time", 0.75));
    reverse_escape_settle_time_ = std::max(
      0.0, declare_parameter<double>("reverse_escape_settle_time", 0.20));
    enable_reverse_side_rear_maneuver_ = declare_parameter<bool>(
      "enable_reverse_side_rear_maneuver", true);
    reverse_side_rear_min_rearward_ = std::max(
      0.0, declare_parameter<double>("reverse_side_rear_min_rearward", 0.02));
    reverse_side_rear_min_lateral_ = std::max(
      0.0, declare_parameter<double>("reverse_side_rear_min_lateral", 0.06));
    reverse_side_rear_max_lateral_ = std::max(
      reverse_side_rear_min_lateral_,
      declare_parameter<double>("reverse_side_rear_max_lateral", 0.60));
    reverse_side_rear_forward_distance_ = std::max(
      0.10, declare_parameter<double>("reverse_side_rear_forward_distance", 0.30));
    reverse_side_rear_forward_min_curvature_ = std::max(
      0.0, declare_parameter<double>("reverse_side_rear_forward_min_curvature", 0.60));
    deskew_obstacle_scan_ = declare_parameter<bool>("deskew_obstacle_scan", true);
    scan_index_time_reversed_ =
      declare_parameter<bool>("scan_index_time_reversed", true);
    scan_distortion_margin_ = declare_parameter<double>("scan_distortion_margin", 0.04);

    local_plan_horizon_ = declare_parameter<double>("local_plan_horizon", 1.80);
    local_plan_step_distance_ =
      std::max(0.02, declare_parameter<double>("local_plan_step_distance", 0.05));
    local_plan_turn_fraction_ = clamp(
      declare_parameter<double>("local_plan_turn_fraction", 0.40), 0.20, 0.48);
    local_plan_curvature_samples_ =
      std::max(5, static_cast<int>(
        declare_parameter<std::int64_t>("local_plan_curvature_samples", 15)));
    local_plan_speed_samples_ =
      std::max(1, static_cast<int>(
        declare_parameter<std::int64_t>("local_plan_speed_samples", 1)));
    max_curvature_ = declare_parameter<double>("max_curvature", 2.5);
    preferred_path_half_width_ = declare_parameter<double>("preferred_path_half_width", 0.32);
    emergency_path_half_width_ = declare_parameter<double>("emergency_path_half_width", 0.80);
    local_path_window_ = declare_parameter<double>("local_path_window", 2.5);
    progress_search_window_ = declare_parameter<double>("progress_search_window", 1.5);
    nominal_mask_check_distance_ =
      std::max(0.10, declare_parameter<double>("nominal_mask_check_distance", 0.45));
    enable_ring_vision_ = declare_parameter<bool>("enable_ring_vision", true);
    ring_binary_topic_ = declare_parameter<std::string>(
      "ring_binary_topic", "/image_yellow_binary");
    ring_start_path_point_ = std::max(
      1, static_cast<int>(declare_parameter<std::int64_t>("ring_start_path_point", 72)));
    ring_end_path_point_ = std::max(
      ring_start_path_point_,
      static_cast<int>(declare_parameter<std::int64_t>("ring_end_path_point", 160)));
    ring_vision_timeout_ = std::max(
      0.05, declare_parameter<double>("ring_vision_timeout", 0.25));
    ring_vision_min_confidence_ = clamp(
      declare_parameter<double>("ring_vision_min_confidence", 0.35), 0.0, 1.0);
    ring_image_threshold_ = static_cast<int>(clamp(
      static_cast<double>(declare_parameter<std::int64_t>("ring_image_threshold", 127)),
      0.0, 255.0));
    ring_image_row_step_ = std::max(
      2, static_cast<int>(declare_parameter<std::int64_t>("ring_image_row_step", 8)));
    ring_image_scan_rows_ = std::max(
      40, static_cast<int>(declare_parameter<std::int64_t>("ring_image_scan_rows", 240)));
    ring_image_min_valid_rows_ = std::max(
      3, static_cast<int>(declare_parameter<std::int64_t>("ring_image_min_valid_rows", 10)));
    ring_image_min_run_width_ = std::max(
      2, static_cast<int>(declare_parameter<std::int64_t>("ring_image_min_run_width", 20)));
    ring_image_max_run_width_ratio_ = clamp(
      declare_parameter<double>("ring_image_max_run_width_ratio", 0.85), 0.1, 1.0);
    ring_image_lookahead_px_ = std::max(
      20.0, declare_parameter<double>("ring_image_lookahead_px", 180.0));
    enable_ring_localization_correction_ =
      declare_parameter<bool>("enable_ring_localization_correction", true);
    ring_localization_correction_topic_ = declare_parameter<std::string>(
      "ring_localization_correction_topic", "/lio_ob/ring_y_correction");
    ring_localization_start_path_point_ = std::max(
      ring_start_path_point_, static_cast<int>(
        declare_parameter<std::int64_t>("ring_localization_start_path_point", 78)));
    ring_localization_end_path_point_ = std::max(
      ring_localization_start_path_point_, static_cast<int>(
        declare_parameter<std::int64_t>("ring_localization_end_path_point", 86)));
    ring_localization_lane_width_ = std::max(
      0.10, declare_parameter<double>("ring_localization_lane_width", 0.50));
    ring_localization_use_depth_ =
      declare_parameter<bool>("ring_localization_use_depth", true);
    ring_localization_require_depth_ =
      declare_parameter<bool>("ring_localization_require_depth", true);
    ring_depth_topic_ = declare_parameter<std::string>(
      "ring_depth_topic", "/aurora/depth/image_raw");
    ring_camera_info_topic_ = declare_parameter<std::string>(
      "ring_camera_info_topic", "/aurora/rgb/camera_info");
    ring_depth_sync_tolerance_ = std::max(
      0.01, declare_parameter<double>("ring_depth_sync_tolerance", 0.12));
    ring_depth_scale_ = std::max(
      1e-6, declare_parameter<double>("ring_depth_scale", 0.001));
    ring_depth_min_range_ = std::max(
      0.05, declare_parameter<double>("ring_depth_min_range", 0.20));
    ring_depth_max_range_ = std::max(
      ring_depth_min_range_ + 0.10,
      declare_parameter<double>("ring_depth_max_range", 4.0));
    ring_depth_search_radius_ = std::max(
      0, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_depth_search_radius", 3)));
    ring_depth_min_center_points_ = std::max(
      3, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_depth_min_center_points", 6)));
    ring_depth_min_forward_span_ = std::max(
      0.10, declare_parameter<double>("ring_depth_min_forward_span", 0.35));
    ring_depth_max_lane_width_error_ = std::max(
      0.05, declare_parameter<double>("ring_depth_max_lane_width_error", 0.18));
    ring_depth_max_fit_rmse_ = std::max(
      0.01, declare_parameter<double>("ring_depth_max_fit_rmse", 0.06));
    ring_camera_x_ = declare_parameter<double>("ring_camera_x", 0.0);
    ring_camera_y_ = declare_parameter<double>("ring_camera_y", 0.0);
    ring_localization_camera_center_x_ratio_ = clamp(
      declare_parameter<double>("ring_localization_camera_center_x_ratio", 0.50), 0.30, 0.70);
    ring_localization_min_y_observability_ = clamp(
      declare_parameter<double>("ring_localization_min_y_observability", 0.90), 0.50, 1.0);
    ring_localization_vehicle_row_ratio_ = clamp(
      declare_parameter<double>("ring_localization_vehicle_row_ratio", 0.96), 0.60, 1.0);
    ring_localization_min_row_ratio_ = clamp(
      declare_parameter<double>("ring_localization_min_row_ratio", 0.52), 0.20,
      ring_localization_vehicle_row_ratio_ - 0.05);
    ring_localization_horizon_row_ratio_ = clamp(
      declare_parameter<double>("ring_localization_horizon_row_ratio", 0.25), 0.0,
      ring_localization_min_row_ratio_ - 0.05);
    ring_localization_min_lane_width_px_ = std::max(
      20.0, declare_parameter<double>("ring_localization_min_lane_width_px", 80.0));
    ring_localization_expected_width_px_ = std::max(
      0.0, declare_parameter<double>("ring_localization_expected_width_px", 0.0));
    ring_localization_max_fit_rmse_ratio_ = std::max(
      0.01, declare_parameter<double>("ring_localization_max_fit_rmse_ratio", 0.10));
    ring_localization_min_solid_fraction_ = clamp(
      declare_parameter<double>("ring_localization_min_solid_fraction", 0.78), 0.0, 1.0);
    ring_localization_max_heading_error_ = std::max(
      0.05, declare_parameter<double>("ring_localization_max_heading_error", 0.40));
    ring_localization_sample_count_ = std::max(
      3, static_cast<int>(declare_parameter<std::int64_t>(
        "ring_localization_sample_count", 5)));
    ring_localization_max_sample_spread_ = std::max(
      0.005, declare_parameter<double>("ring_localization_max_sample_spread", 0.05));
    ring_localization_measurement_std_ = std::max(
      0.005, declare_parameter<double>("ring_localization_measurement_std", 0.025));
    ring_localization_max_correction_ = std::max(
      0.05, declare_parameter<double>("ring_localization_max_correction", 0.35));
    ring_recovery_speed_ = std::max(
      0.05, declare_parameter<double>("ring_recovery_speed", 0.28));
    ring_recovery_horizon_ = std::max(
      0.20, declare_parameter<double>("ring_recovery_horizon", 0.65));
    ring_recovery_path_half_width_ = std::max(
      robot_half_width_, declare_parameter<double>("ring_recovery_path_half_width", 0.55));
    ring_recovery_min_mask_improvement_ = clamp(
      declare_parameter<double>("ring_recovery_min_mask_improvement", 0.05), 0.0, 1.0);
    ring_field_boundary_margin_ = std::max(
      0.0, declare_parameter<double>("ring_field_boundary_margin", 0.05));
    obstacle_safety_margin_ = declare_parameter<double>("obstacle_safety_margin", 0.05);
    obstacle_memory_duration_ =
      std::max(0.0, declare_parameter<double>("obstacle_memory_duration", 1.0));
    obstacle_memory_radius_ =
      std::max(0.5, declare_parameter<double>("obstacle_memory_radius", 2.5));
    obstacle_memory_voxel_size_ =
      std::max(0.01, declare_parameter<double>("obstacle_memory_voxel_size", 0.06));
    enable_lidar_noise_filter_ =
      declare_parameter<bool>("enable_lidar_noise_filter", true);
    scan_cluster_max_beam_gap_ = std::max(
      1, static_cast<int>(
        declare_parameter<std::int64_t>("scan_cluster_max_beam_gap", 2)));
    scan_cluster_max_distance_ = std::max(
      0.02, declare_parameter<double>("scan_cluster_max_distance", 0.16));
    scan_cluster_min_points_ = std::max(
      1, static_cast<int>(
        declare_parameter<std::int64_t>("scan_cluster_min_points", 3)));
    obstacle_confirmation_hits_ = std::max(
      1, static_cast<int>(
        declare_parameter<std::int64_t>("obstacle_confirmation_hits", 2)));
    obstacle_confirmation_window_ = std::max(
      0.05, declare_parameter<double>("obstacle_confirmation_window", 0.30));
    obstacle_confirmation_radius_ = std::max(
      obstacle_memory_voxel_size_,
      declare_parameter<double>("obstacle_confirmation_radius", 0.10));
    unconfirmed_obstacle_ttl_ = std::max(
      obstacle_confirmation_window_,
      declare_parameter<double>("unconfirmed_obstacle_ttl", 0.35));
    enable_speed_weighted_obstacle_memory_ = declare_parameter<bool>(
      "enable_speed_weighted_obstacle_memory", true);
    obstacle_low_speed_threshold_ = std::max(
      0.05, declare_parameter<double>("obstacle_low_speed_threshold", 0.35));
    obstacle_high_speed_measurement_weight_ = clamp(
      declare_parameter<double>("obstacle_high_speed_measurement_weight", 0.20),
      0.01, 1.0);
    obstacle_stopped_measurement_weight_ = clamp(
      declare_parameter<double>("obstacle_stopped_measurement_weight", 0.75),
      obstacle_high_speed_measurement_weight_, 1.0);
    filter_field_boundary_returns_ =
      declare_parameter<bool>("filter_field_boundary_returns", true);
    field_min_x_ = declare_parameter<double>("field_min_x", 0.0);
    field_max_x_ = declare_parameter<double>("field_max_x", 5.0);
    field_min_y_ = declare_parameter<double>("field_min_y", 0.0);
    field_max_y_ = declare_parameter<double>("field_max_y", 5.0);
    field_boundary_filter_band_ =
      std::max(0.0, declare_parameter<double>("field_boundary_filter_band", 0.20));
    filter_wall_line_clusters_ =
      declare_parameter<bool>("filter_wall_line_clusters", true);
    wall_cluster_min_points_ = std::max(
      3, static_cast<int>(
        declare_parameter<std::int64_t>("wall_cluster_min_points", 8)));
    wall_cluster_min_length_ =
      std::max(0.10, declare_parameter<double>("wall_cluster_min_length", 0.45));
    wall_cluster_max_thickness_ =
      std::max(0.01, declare_parameter<double>("wall_cluster_max_thickness", 0.07));
    wall_cluster_min_linearity_ = clamp(
      declare_parameter<double>("wall_cluster_min_linearity", 0.97), 0.5, 0.9999);
    weight_path_distance_ = declare_parameter<double>("weight_path_distance", 18.0);
    weight_path_heading_ = declare_parameter<double>("weight_path_heading", 2.0);
    weight_path_return_ = declare_parameter<double>("weight_path_return", 24.0);
    weight_black_area_ = declare_parameter<double>("weight_black_area", 80.0);
    weight_obstacle_clearance_ = declare_parameter<double>("weight_obstacle_clearance", 0.8);
    weight_steering_change_ = declare_parameter<double>("weight_steering_change", 0.8);
    weight_preferred_side_ = declare_parameter<double>("weight_preferred_side", 10.0);
    weight_progress_ = declare_parameter<double>("weight_progress", 7.0);
    weight_speed_ = declare_parameter<double>("weight_speed", 8.0);
    weight_visual_command_ = declare_parameter<double>("weight_visual_command", 24.0);

    image_width_ = declare_parameter<double>("image_width", 1920.0);
    image_height_ = declare_parameter<double>("image_height", 1080.0);
    image_center_x_ = declare_parameter<double>("image_center_x", 920.0);
    hat_trigger_ratio_ = declare_parameter<double>("hat_trigger_ratio", 0.22);
    hat_force_trigger_ratio_ = declare_parameter<double>("hat_force_trigger_ratio", 0.50);
    hat_horizontal_limit_ = declare_parameter<double>("hat_horizontal_limit", 500.0);
    hat_width_scale_ = declare_parameter<double>("hat_width_scale", 1.55);
    hat_avoid_target_scale_ = declare_parameter<double>("hat_avoid_target_scale", 1.60);
    hat_direction_bias_px_ = declare_parameter<double>("hat_direction_bias_px", 180.0);
    hat_kp_ = declare_parameter<double>("hat_kp", 0.002);
    hat_ki_ = declare_parameter<double>("hat_ki", 0.00007);
    hat_kd_ = declare_parameter<double>("hat_kd", 0.00007);
    vision_timeout_ = declare_parameter<double>("vision_timeout", 0.35);
    use_vision_when_reversing_ = declare_parameter<bool>("use_vision_when_reversing", false);

    // Visual parking is a terminal controller adapted from second_line's P-target policy.
    // The recorded path brings the car into the parking area; vision owns /cmd_vel only
    // after the controller is close to the end of the post-QR path.
    enable_visual_parking_ = declare_parameter<bool>("enable_visual_parking", true);
    visual_parking_only_post_qr_ =
      declare_parameter<bool>("visual_parking_only_post_qr", true);
    parking_target_type_ = declare_parameter<std::string>("parking_target_type", "P");
    parking_arm_distance_ = std::max(
      0.0, declare_parameter<double>("parking_arm_distance", 2.00));
    parking_min_confidence_ = clamp(
      declare_parameter<double>("parking_min_confidence", 0.0), 0.0, 1.0);
    parking_detection_timeout_ = std::max(
      0.05, declare_parameter<double>("parking_detection_timeout", 0.35));
    parking_obstacle_margin_px_ = std::max(
      0.0, declare_parameter<double>("parking_obstacle_margin_px", 150.0));
    parking_center_x_ = declare_parameter<double>(
      "parking_center_x", 0.5 * image_width_);
    parking_decel_start_y_ = declare_parameter<double>("parking_decel_start_y", 500.0);
    parking_stop_y_ = declare_parameter<double>("parking_stop_y", image_height_);
    parking_trigger_y_ = declare_parameter<double>(
      "parking_trigger_y", image_height_ - 50.0);
    if (parking_stop_y_ <= parking_decel_start_y_ ||
      parking_trigger_y_ < parking_decel_start_y_ || parking_trigger_y_ > parking_stop_y_)
    {
      RCLCPP_FATAL(
        get_logger(),
        "visual parking requires decel_start_y <= trigger_y <= stop_y");
      throw std::runtime_error("invalid visual parking pixel thresholds");
    }
    parking_max_speed_ = std::max(
      0.0, declare_parameter<double>("parking_max_speed", 0.80));
    parking_kp_ = declare_parameter<double>("parking_kp", 0.0025);
    parking_kd_ = declare_parameter<double>("parking_kd", 0.0010);
    parking_final_crawl_speed_ = std::max(
      0.0, declare_parameter<double>("parking_final_crawl_speed", 0.36));
    parking_final_crawl_duration_ = std::max(
      0.0, declare_parameter<double>("parking_final_crawl_duration", 0.60));

    if (enable_qr_mission_) {
      if (!load_path(qr_sprint_path_file_, qr_sprint_path_, false) ||
        !load_path(qr_clockwise_path_file_, qr_clockwise_path_, true) ||
        !load_path(qr_counterclockwise_path_file_, qr_counterclockwise_path_, true))
      {
        RCLCPP_FATAL(
          get_logger(), "二维码任务路径缺失或格式错误：冲刺=%s 顺时针=%s 逆时针=%s",
          qr_sprint_path_file_.c_str(), qr_clockwise_path_file_.c_str(),
          qr_counterclockwise_path_file_.c_str());
        throw std::runtime_error("QR mission paths are missing or malformed");
      }
      if (!validate_qr_mission_paths()) {
        throw std::runtime_error("QR mission path contract is invalid");
      }
      recorded_path_ = qr_sprint_path_;
      qr_mission_phase_ = QrMissionPhase::SPRINT;
      qr_mission_start_time_ = now();
      if (auto_select_post_qr_route_) {
        load_auto_route_candidates(
          auto_route_clockwise_path_files_, auto_route_clockwise_candidates_,
          auto_route_clockwise_candidate_names_);
        load_auto_route_candidates(
          auto_route_counterclockwise_path_files_, auto_route_counterclockwise_candidates_,
          auto_route_counterclockwise_candidate_names_);
        RCLCPP_INFO(
          get_logger(), "二维码后自动选路已启用：顺时针候选=%zu，逆时针候选=%zu，障碍影响半径=%.2f米",
          auto_route_clockwise_candidates_.size(),
          auto_route_counterclockwise_candidates_.size(),
          auto_route_obstacle_influence_radius_);
        if (auto_route_clockwise_candidates_.empty() ||
          auto_route_counterclockwise_candidates_.empty())
        {
          RCLCPP_WARN(
            get_logger(), "自动选路候选不完整，将在扫码后回退到固定二维码路线");
        }
      }
    } else if (!load_path(path_file_, recorded_path_, true)) {
      RCLCPP_FATAL(get_logger(), "记录路径加载失败：%s", path_file_.c_str());
      throw std::runtime_error("recorded path is missing or malformed");
    }
    std::size_t maximum_path_size = enable_qr_mission_ ?
      std::max(qr_clockwise_path_.size(), qr_counterclockwise_path_.size()) :
      recorded_path_.size();
    for (const auto & candidate : auto_route_clockwise_candidates_) {
      maximum_path_size = std::max(maximum_path_size, candidate.size());
    }
    for (const auto & candidate : auto_route_counterclockwise_candidates_) {
      maximum_path_size = std::max(maximum_path_size, candidate.size());
    }
    if (ring_end_path_point_ > static_cast<int>(maximum_path_size)) {
      RCLCPP_WARN(
        get_logger(),
        "环形视觉已关闭：配置的路径点范围%d-%d超过路径总数%zu",
        ring_start_path_point_, ring_end_path_point_, maximum_path_size);
      enable_ring_vision_ = false;
    }
    log_direction_segments();
    if (use_keepout_mask_) {
      if (keepout_resolution_ <= 0.0 || !load_keepout_mask(keepout_mask_file_)) {
        RCLCPP_FATAL(
          get_logger(), "禁行区掩码必须是可读的PGM文件，并且分辨率必须为正数");
        throw std::runtime_error("keepout mask calibration is incomplete");
      }
    }

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      std::bind(&PurePursuitNode::scan_callback, this, std::placeholders::_1));
    vision_sub_ = create_subscription<ai_msgs::msg::PerceptionTargets>(
      vision_topic_, rclcpp::QoS(10),
      std::bind(&PurePursuitNode::vision_callback, this, std::placeholders::_1));
    if (enable_qr_mission_) {
      // qr_code_detection publishes Sign with reliable/volatile QoS.
      // A transient-local subscriber is incompatible with that publisher.
      const auto qr_qos = rclcpp::QoS(10).reliable().durability_volatile();
      qr_sign_sub_ = create_subscription<origincar_msg::msg::Sign>(
        qr_sign_topic_, qr_qos,
        std::bind(&PurePursuitNode::qr_sign_callback, this, std::placeholders::_1));
      if (first_line_mode_ == "vision") {
        visual_first_line_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          visual_first_line_cmd_topic_, rclcpp::QoS(10),
          std::bind(
            &PurePursuitNode::visual_first_line_cmd_callback, this,
            std::placeholders::_1));
      } else if (first_line_mode_ == "external") {
        const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
        control_started_sub_ = create_subscription<std_msgs::msg::Bool>(
          control_started_topic_, state_qos,
          std::bind(
            &PurePursuitNode::control_started_callback, this,
            std::placeholders::_1));
        control_released_sub_ = create_subscription<std_msgs::msg::Bool>(
          control_released_topic_, state_qos,
          std::bind(
            &PurePursuitNode::control_released_callback, this,
            std::placeholders::_1));
      }
    }
    if (enable_ring_vision_) {
      ring_binary_sub_ = create_subscription<sensor_msgs::msg::Image>(
        ring_binary_topic_, sensor_qos,
        std::bind(&PurePursuitNode::ring_binary_callback, this, std::placeholders::_1));
      if (enable_ring_localization_correction_ && ring_localization_use_depth_) {
        ring_depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
          ring_depth_topic_, sensor_qos,
          std::bind(&PurePursuitNode::ring_depth_callback, this, std::placeholders::_1));
        ring_camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
          ring_camera_info_topic_, sensor_qos,
          std::bind(&PurePursuitNode::ring_camera_info_callback, this, std::placeholders::_1));
      }
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/lio_ob/tracking_path", rclcpp::QoS(1).transient_local());
    lookahead_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/lio_ob/lookahead", rclcpp::QoS(10));
    local_plan_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/lio_ob/local_plan", rclcpp::QoS(10));
    state_pub_ = create_publisher<std_msgs::msg::String>("/lio_ob/state", rclcpp::QoS(10));
    ring_photo_trigger_pub_ = create_publisher<std_msgs::msg::String>(
      ring_photo_trigger_topic_, rclcpp::QoS(1).reliable().durability_volatile());
    keepout_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/lio_ob/keepout_map", rclcpp::QoS(1).transient_local());
    obstacle_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio_ob/obstacles", rclcpp::SensorDataQoS().keep_last(2));
    debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/lio_ob/debug_markers", rclcpp::QoS(10));
    ring_vision_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/lio_ob/ring_vision", rclcpp::SensorDataQoS().keep_last(5));
    ring_localization_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      ring_localization_correction_topic_, rclcpp::QoS(5));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency_));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PurePursuitNode::control_step, this));
    last_control_time_ = now();

    RCLCPP_INFO(
      get_logger(), "纯追踪已加载 %zu 个路径点；定位=%s 雷达=%s 视觉=%s 速度输出=%s",
      recorded_path_.size(), odom_topic_.c_str(), scan_topic_.c_str(), vision_topic_.c_str(),
      cmd_vel_topic_.c_str());
    if (enable_qr_mission_) {
      RCLCPP_INFO(
        get_logger(),
        "二维码两段式任务：冲刺=%zu 点，终点=(%.2f, %.2f, %.1f deg)，话题=%s，"
        "sign=3表示顺时针，sign=4表示逆时针，第一赛段=%s",
        qr_sprint_path_.size(), qr_sprint_path_.back().pose.x,
        qr_sprint_path_.back().pose.y,
        180.0 * qr_sprint_path_.back().pose.yaw / std::acos(-1.0),
        qr_sign_topic_.c_str(),
        first_line_mode_ == "external" ? "外部origincar_control，LIO仅监控" :
        (first_line_mode_ == "vision" ? "origincar_control纯视觉转发" : "LIO路径跟踪"));
    }
    RCLCPP_INFO(
      get_logger(), "二维码后固定路线由内置纯追踪和局部避障控制，始终不启用外部规划器");
    RCLCPP_INFO(
      get_logger(), "二维码后自动选路：%s，候选评分范围=通道前，影响半径=%.2f米",
      auto_select_post_qr_route_ ? "启用" : "关闭", auto_route_obstacle_influence_radius_);
    RCLCPP_INFO(
      get_logger(),
      "倒车侧后方两阶段避障：%s，检测区[后向>=%.2f米 横向=%.2f-%.2f米]，"
      "前向腾挪=%.2f米 最小曲率=%.2f",
      enable_reverse_side_rear_maneuver_ ? "启用" : "关闭",
      reverse_side_rear_min_rearward_, reverse_side_rear_min_lateral_,
      reverse_side_rear_max_lateral_, reverse_side_rear_forward_distance_,
      reverse_side_rear_forward_min_curvature_);
    RCLCPP_INFO(
      get_logger(),
      "二维码后过渡段：进入通道Y=%.2f米前 正走上限=%.2f米/秒 倒车上限=%.2f米/秒；"
      "低速点云加权=%s 阈值=%.2f米/秒 权重[高速=%.2f 停车=%.2f]",
      post_qr_transition_end_y_, post_qr_transition_forward_speed_,
      post_qr_transition_reverse_speed_,
      enable_speed_weighted_obstacle_memory_ ? "启用" : "关闭",
      obstacle_low_speed_threshold_, obstacle_high_speed_measurement_weight_,
      obstacle_stopped_measurement_weight_);
    if (enable_ring_vision_) {
      RCLCPP_INFO(
        get_logger(), "环形视觉：二值图=%s 路径点=%d-%d",
        ring_binary_topic_.c_str(), ring_start_path_point_, ring_end_path_point_);
      RCLCPP_INFO(
        get_logger(), "环形区雷达占道过滤：%s，参考路径横向阈值=%.2f米",
        enable_ring_obstacle_path_filter_ ? "启用" : "关闭", ring_obstacle_path_half_width_);
      if (enable_ring_localization_correction_) {
        RCLCPP_INFO(
          get_logger(),
          "环形区Y坐标纠正：路径点=%d-%d 话题=%s 车道宽度=%.2f 深度=%s 相机信息=%s",
          ring_localization_start_path_point_, ring_localization_end_path_point_,
          ring_localization_correction_topic_.c_str(), ring_localization_lane_width_,
          ring_depth_topic_.c_str(), ring_camera_info_topic_.c_str());
      }
    }
    if (enable_visual_parking_) {
      RCLCPP_INFO(
        get_logger(),
        "Visual parking enabled: target=%s, arm distance=%.2f m, trigger y=%.0f px",
        parking_target_type_.c_str(), parking_arm_distance_, parking_trigger_y_);
    }
  }

  ~PurePursuitNode() override
  {
    if (!(enable_qr_mission_ && first_line_mode_ == "external" &&
      qr_mission_phase_ == QrMissionPhase::SPRINT))
    {
      publish_command(0.0, 0.0);
    }
  }

private:
    struct PathPoint
    {
      Pose2d pose;
      double arc_length{0.0};
      int direction{0};
  };

  struct LidarObstacle
  {
    bool detected{false};
    double clearance{std::numeric_limits<double>::infinity()};
    int preferred_side{1};  // +1 left, -1 right in base_link
    double point_x{std::numeric_limits<double>::quiet_NaN()};
    double point_y{std::numeric_limits<double>::quiet_NaN()};
    bool detected_on_arc{false};
    int ring_filtered_points{0};
  };

  struct PathMetric
  {
    double distance{std::numeric_limits<double>::infinity()};
    double heading_error{0.0};
    double arc_length{0.0};
  };

  struct LocalPlan
  {
    bool valid{false};
    double speed{0.0};
    double yaw_rate{0.0};
    double cost{std::numeric_limits<double>::infinity()};
    double max_path_deviation{0.0};
    double min_obstacle_clearance{std::numeric_limits<double>::infinity()};
    bool enters_black_area{false};
    double travel_distance{0.0};
    std::vector<Pose2d> trajectory;
  };

  struct CandidateRejections
  {
    int yaw_rate{0};
    int path_metric{0};
    int path_corridor{0};
    int keepout{0};
    int collision{0};
  };

  struct EscapeRejections
  {
    int yaw_rate{0};
    int field_boundary{0};
    int path_corridor{0};
    int keepout{0};
    int collision{0};
  };

  struct AutoRouteScore
  {
    std::size_t colliding_obstacles{0U};
    double proximity_cost{0.0};
    double start_distance{std::numeric_limits<double>::infinity()};
    double total_cost{std::numeric_limits<double>::infinity()};
  };

  struct TimedObstaclePoint
  {
    Eigen::Vector2d world_point;
    rclcpp::Time first_seen{0, 0, RCL_ROS_TIME};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::uint64_t last_scan_id{0U};
    int hit_count{0};
    bool confirmed{false};
  };

  struct RingRowRun
  {
    double x{0.0};
    double y{0.0};
    double width{0.0};
    int begin{0};
    int end{0};
  };

  struct RingMetricObservation
  {
    bool valid{false};
    double lateral_m{0.0};
    double heading_error{0.0};
    double lane_center_px{0.0};
    double lane_width_px{0.0};
    double fit_rmse_px{std::numeric_limits<double>::infinity()};
    double solid_fraction{0.0};
  };

  enum class EscapePhase
  {
    IDLE,
    BRAKING,
    REVERSING,
    SETTLING
  };

  enum class QrMissionPhase
  {
    DISABLED,
    SPRINT,
    BRAKING,
    POST_QR
  };

  enum class ParkingPhase
  {
    IDLE,
    APPROACH,
    FINAL_CRAWL,
    STOPPED
  };

  bool post_qr_transition_active() const
  {
    return enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::POST_QR &&
           !tracking_path_.empty() &&
           progress_index_ < post_qr_transition_end_index_;
  }

  double path_speed_limit(int direction) const
  {
    if (post_qr_transition_active()) {
      return direction > 0 ?
        post_qr_transition_forward_speed_ : post_qr_transition_reverse_speed_;
    }
    return direction > 0 ? max_forward_speed_ : max_reverse_speed_;
  }

  double obstacle_measurement_weight() const
  {
    if (!enable_speed_weighted_obstacle_memory_) {
      return 0.35;
    }
    const double speed_ratio = clamp(
      std::abs(measured_velocity_) / obstacle_low_speed_threshold_, 0.0, 1.0);
    return obstacle_stopped_measurement_weight_ - speed_ratio *
      (obstacle_stopped_measurement_weight_ - obstacle_high_speed_measurement_weight_);
  }

  static bool read_pgm_token(std::istream & stream, std::string & token)
  {
    while (stream >> token) {
      if (!token.empty() && token.front() == '#') {
        std::string ignored;
        std::getline(stream, ignored);
        continue;
      }
      return true;
    }
    return false;
  }

  bool load_keepout_mask(const std::string & filename)
  {
    std::ifstream stream(filename, std::ios::binary);
    std::string token;
    if (!stream.is_open() || !read_pgm_token(stream, token) || token != "P5") {
      return false;
    }
    if (!read_pgm_token(stream, token)) {
      return false;
    }
    keepout_width_ = std::stoi(token);
    if (!read_pgm_token(stream, token)) {
      return false;
    }
    keepout_height_ = std::stoi(token);
    if (!read_pgm_token(stream, token)) {
      return false;
    }
    const int maximum_value = std::stoi(token);
    if (keepout_width_ <= 0 || keepout_height_ <= 0 || maximum_value <= 0 || maximum_value > 255) {
      return false;
    }
    stream.get();
    keepout_pixels_.resize(
      static_cast<std::size_t>(keepout_width_) * static_cast<std::size_t>(keepout_height_));
    stream.read(
      reinterpret_cast<char *>(keepout_pixels_.data()),
      static_cast<std::streamsize>(keepout_pixels_.size()));
    if (stream.gcount() != static_cast<std::streamsize>(keepout_pixels_.size())) {
      keepout_pixels_.clear();
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "已加载禁行区掩码：%dx%d，分辨率=%.6f 米/像素",
      keepout_width_, keepout_height_, keepout_resolution_);
    return true;
  }

  bool is_drivable_mask_point(const Eigen::Vector2d & lio_point) const
  {
    if (!use_keepout_mask_ || qr_keepout_is_relaxed()) {
      return true;
    }
    const Eigen::Vector2d recorded_point = transform_point(inverse(path_alignment_), lio_point);
    const Eigen::Vector2d mask_point = transform_point(inverse(keepout_origin_), recorded_point);
    const int column = static_cast<int>(std::floor(mask_point.x() / keepout_resolution_));
    const int row_from_bottom = static_cast<int>(std::floor(mask_point.y() / keepout_resolution_));
    const int row = keepout_height_ - 1 - row_from_bottom;
    if (column < 0 || column >= keepout_width_ || row < 0 || row >= keepout_height_) {
      return false;
    }
    const std::size_t index = static_cast<std::size_t>(row) *
      static_cast<std::size_t>(keepout_width_) + static_cast<std::size_t>(column);
    return keepout_pixels_[index] >= keepout_white_threshold_;
  }

  double forbidden_footprint_fraction(const Pose2d & lio_pose) const
  {
    if (!use_keepout_mask_) {
      return 0.0;
    }
    const std::array<Eigen::Vector2d, 9> footprint_samples{{
      Eigen::Vector2d(0.0, 0.0),
      Eigen::Vector2d(robot_half_length_, robot_half_width_),
      Eigen::Vector2d(robot_half_length_, -robot_half_width_),
      Eigen::Vector2d(-robot_half_length_, robot_half_width_),
      Eigen::Vector2d(-robot_half_length_, -robot_half_width_),
      Eigen::Vector2d(robot_half_length_, 0.0),
      Eigen::Vector2d(-robot_half_length_, 0.0),
      Eigen::Vector2d(0.0, robot_half_width_),
      Eigen::Vector2d(0.0, -robot_half_width_)}};
    int forbidden_samples = 0;
    for (const auto & sample : footprint_samples) {
      if (!is_drivable_mask_point(transform_point(lio_pose, sample))) {
        ++forbidden_samples;
      }
    }
    return static_cast<double>(forbidden_samples) /
      static_cast<double>(footprint_samples.size());
  }

  static double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion_msg)
  {
    tf2::Quaternion quaternion(
      quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
  }

  bool load_path(
    const std::string & filename, std::vector<PathPoint> & output,
    bool apply_ring_offset)
  {
    output.clear();
    if (filename.empty()) {
      return false;
    }
    std::ifstream stream(filename);
    if (!stream.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(stream, line)) {
      if (line.empty() || line.front() == '#') {
        continue;
      }
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream values(line);
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double qx = 0.0;
      double qy = 0.0;
      double qz = 0.0;
      double qw = 1.0;
      if (!(values >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        RCLCPP_WARN(get_logger(), "跳过格式错误的路径行：%s", line.c_str());
        continue;
      }
      (void)z;
      (void)qx;
      (void)qy;
        geometry_msgs::msg::Quaternion quaternion;
        quaternion.z = qz;
        quaternion.w = qw;
        int explicit_direction = 0;
        if (values >> explicit_direction) {
          explicit_direction = explicit_direction == 0 ? 0 :
            (explicit_direction > 0 ? 1 : -1);
        }
        output.push_back(
          {{x, y, quaternion_to_yaw(quaternion)}, 0.0, explicit_direction});
    }
    if (output.size() < 2U) {
      return false;
    }

    if (apply_ring_offset && std::abs(ring_path_y_offset_) > 1e-9) {
      const std::size_t first = std::min(
        static_cast<std::size_t>(ring_path_start_point_ - 1), output.size());
      const std::size_t last = std::min(
        static_cast<std::size_t>(ring_path_end_point_), output.size());
      for (std::size_t i = first; i < last; ++i) {
        output[i].pose.y += ring_path_y_offset_;
      }
      for (int step = 1; step <= ring_path_blend_points_; ++step) {
        const double factor = static_cast<double>(ring_path_blend_points_ - step + 1) /
          static_cast<double>(ring_path_blend_points_ + 1);
        if (first >= static_cast<std::size_t>(step)) {
          output[first - static_cast<std::size_t>(step)].pose.y +=
            factor * ring_path_y_offset_;
        }
        const std::size_t after = last + static_cast<std::size_t>(step - 1);
        if (after < output.size()) {
          output[after].pose.y += factor * ring_path_y_offset_;
        }
      }
      RCLCPP_WARN(
        get_logger(), "临时路径偏移：环形区第 %d-%d 点整体沿 y 移动 %+.3f m，边界过渡 %d 点",
        ring_path_start_point_, ring_path_end_point_, ring_path_y_offset_,
        ring_path_blend_points_);
    }

    double arc_length = 0.0;
    for (std::size_t i = 0; i < output.size(); ++i) {
      if (i > 0U) {
        arc_length += std::hypot(
          output[i].pose.x - output[i - 1U].pose.x,
          output[i].pose.y - output[i - 1U].pose.y);
      }
      output[i].arc_length = arc_length;

        if (output[i].direction == 0) {
          const std::size_t next = std::min(i + 1U, output.size() - 1U);
          const std::size_t previous = i == output.size() - 1U ? i - 1U : i;
          const double dx = output[next].pose.x - output[previous].pose.x;
          const double dy = output[next].pose.y - output[previous].pose.y;
          const double heading_x = std::cos(output[i].pose.yaw);
          const double heading_y = std::sin(output[i].pose.yaw);
          output[i].direction = dx * heading_x + dy * heading_y >= 0.0 ? 1 : -1;
        }
    }
    return true;
  }

  static void recompute_arc_lengths(std::vector<PathPoint> & path)
  {
    double arc_length = 0.0;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i > 0U) {
        arc_length += std::hypot(
          path[i].pose.x - path[i - 1U].pose.x,
          path[i].pose.y - path[i - 1U].pose.y);
      }
      path[i].arc_length = arc_length;
    }
  }

  void load_auto_route_candidates(
    const std::vector<std::string> & filenames,
    std::vector<std::vector<PathPoint>> & candidates,
    std::vector<std::string> & candidate_names)
  {
    candidates.clear();
    candidate_names.clear();
    for (const auto & filename : filenames) {
      std::vector<PathPoint> candidate;
      if (!load_path(filename, candidate, true)) {
        RCLCPP_WARN(get_logger(), "跳过无效的自动选路候选路径：%s", filename.c_str());
        continue;
      }
      if (candidate.front().direction >= 0) {
        RCLCPP_WARN(
          get_logger(), "跳过首段不是倒车的自动选路候选：%s", filename.c_str());
        continue;
      }
      candidates.push_back(std::move(candidate));
      const std::size_t separator = filename.find_last_of("/\\");
      candidate_names.push_back(
        separator == std::string::npos ? filename : filename.substr(separator + 1U));
    }
  }

  double route_obstacle_clearance(
    const Pose2d & route_pose, const Eigen::Vector2d & obstacle_world) const
  {
    const double footprint_length =
      robot_half_length_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double footprint_width =
      robot_half_width_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double cos_yaw = std::cos(route_pose.yaw);
    const double sin_yaw = std::sin(route_pose.yaw);
    const double dx = obstacle_world.x() - route_pose.x;
    const double dy = obstacle_world.y() - route_pose.y;
    const double robot_x = cos_yaw * dx + sin_yaw * dy;
    const double robot_y = -sin_yaw * dx + cos_yaw * dy;
    const double penetration_x = footprint_length - std::abs(robot_x);
    const double penetration_y = footprint_width - std::abs(robot_y);
    if (penetration_x > 0.0 && penetration_y > 0.0) {
      return -std::min(penetration_x, penetration_y);
    }
    return std::hypot(
      std::max(0.0, -penetration_x), std::max(0.0, -penetration_y));
  }

  AutoRouteScore score_auto_route(const std::vector<PathPoint> & candidate) const
  {
    AutoRouteScore score;
    if (candidate.empty()) {
      return score;
    }

    const Pose2d start_pose = compose(path_alignment_, candidate.front().pose);
    score.start_distance = std::hypot(
      start_pose.x - current_pose_.x, start_pose.y - current_pose_.y);

    std::size_t nearest_index = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    std::size_t transition_end = candidate.size() - 1U;
    for (std::size_t index = 0U; index < candidate.size(); ++index) {
      const Pose2d pose = compose(path_alignment_, candidate[index].pose);
      const double distance = std::hypot(
        pose.x - current_pose_.x, pose.y - current_pose_.y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = index;
      }
      if (pose.y >= post_qr_transition_end_y_) {
        transition_end = index;
        break;
      }
    }
    if (nearest_index >= transition_end || laser_points_.empty()) {
      score.total_cost = score.start_distance;
      return score;
    }

    const double influence_radius = auto_route_obstacle_influence_radius_;
    for (const auto & local_obstacle : laser_points_) {
      const Eigen::Vector2d obstacle_world = transform_point(current_pose_, local_obstacle);
      double minimum_clearance = std::numeric_limits<double>::infinity();
      for (std::size_t index = nearest_index; index <= transition_end; ++index) {
        const Pose2d pose = compose(path_alignment_, candidate[index].pose);
        minimum_clearance = std::min(
          minimum_clearance, route_obstacle_clearance(pose, obstacle_world));
      }
      if (minimum_clearance > influence_radius) {
        continue;
      }
      if (minimum_clearance <= 0.0) {
        ++score.colliding_obstacles;
      }
      score.proximity_cost += influence_radius - std::max(0.0, minimum_clearance);
    }
    score.total_cost =
      1000.0 * static_cast<double>(score.colliding_obstacles) +
      10.0 * score.proximity_cost + 2.0 * score.start_distance;
    return score;
  }

  const std::vector<PathPoint> & select_auto_post_qr_path()
  {
    const bool clockwise = qr_selected_sign_ == 3;
    const auto & candidates = clockwise ?
      auto_route_clockwise_candidates_ : auto_route_counterclockwise_candidates_;
    const auto & names = clockwise ?
      auto_route_clockwise_candidate_names_ : auto_route_counterclockwise_candidate_names_;
    const auto & fallback = clockwise ? qr_clockwise_path_ : qr_counterclockwise_path_;
    if (!auto_select_post_qr_route_ || candidates.empty()) {
      return fallback;
    }

    std::size_t best_index = 0U;
    AutoRouteScore best_score;
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      const AutoRouteScore score = score_auto_route(candidates[index]);
      RCLCPP_INFO(
        get_logger(),
        "自动选路候选[%zu] %s：碰撞障碍点=%zu 近障代价=%.3f 起点偏差=%.3f米 总分=%.3f",
        index + 1U, names[index].c_str(), score.colliding_obstacles,
        score.proximity_cost, score.start_distance, score.total_cost);
      if (score.total_cost < best_score.total_cost) {
        best_score = score;
        best_index = index;
      }
    }
    RCLCPP_WARN(
      get_logger(), "自动选择二维码后%s倒车入环路线：%s（碰撞障碍点=%zu，总分=%.3f）",
      clockwise ? "顺时针" : "逆时针", names[best_index].c_str(),
      best_score.colliding_obstacles, best_score.total_cost);
    return candidates[best_index];
  }

  bool validate_qr_mission_paths()
  {
    if (qr_sprint_path_.size() < 8U ||
      !std::all_of(
        qr_sprint_path_.begin(), qr_sprint_path_.end(),
        [](const PathPoint & point) {return point.direction > 0;}))
    {
      RCLCPP_FATAL(get_logger(), "二维码冲刺路径必须至少 8 点且全部为前进");
      return false;
    }
    if (qr_clockwise_path_.size() != qr_counterclockwise_path_.size()) {
      RCLCPP_FATAL(get_logger(), "顺/逆时针二维码后路径点数不一致");
      return false;
    }
    for (std::size_t i = 0U; i < qr_clockwise_path_.size(); ++i) {
      if (qr_clockwise_path_[i].direction == 0 || qr_counterclockwise_path_[i].direction == 0) {
        RCLCPP_FATAL(get_logger(), "二维码后路径必须显式指定行驶方向：错误点=%zu", i + 1U);
        return false;
      }
    }
    return true;
  }

  bool qr_keepout_is_relaxed() const
  {
    if (!enable_qr_mission_ || !qr_allow_keepout_) {
      return false;
    }
    return qr_mission_phase_ == QrMissionPhase::SPRINT ||
           qr_mission_phase_ == QrMissionPhase::BRAKING;
  }

  void visual_first_line_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    visual_first_line_command_ = *msg;
    last_visual_first_line_cmd_ = now();
    have_visual_first_line_cmd_ = true;
    if (enable_qr_mission_ && first_line_mode_ == "vision" &&
      qr_mission_phase_ == QrMissionPhase::SPRINT)
    {
      // The LIO controller is only the single-writer command arbiter in this
      // phase. The visual controller's linear and angular commands pass
      // through unchanged; path tracking, keepout and lidar avoidance do not
      // participate until the QR route switch.
      commanded_speed_ = msg->linear.x;
      publish_command(msg->linear.x, msg->angular.z);
    }
  }

  void qr_sign_callback(const origincar_msg::msg::Sign::SharedPtr msg)
  {
    if (!enable_qr_mission_ || qr_mission_phase_ != QrMissionPhase::SPRINT) {
      return;
    }
    if (msg->sign_data != 3 && msg->sign_data != 4) {
      RCLCPP_WARN(get_logger(), "忽略未知二维码方向 sign_data=%d", msg->sign_data);
      return;
    }
    if (first_line_mode_ == "external" && !external_control_started_) {
      RCLCPP_WARN(get_logger(), "忽略视觉第一赛段启动前收到的二维码结果");
      return;
    }
    if ((!have_odom_ || !path_aligned_) && first_line_mode_ != "external") {
      RCLCPP_WARN(get_logger(), "忽略定位建立前收到的二维码结果，防止沿用上一轮缓存");
      return;
    }
    if ((now() - qr_mission_start_time_).seconds() < qr_detection_arm_delay_) {
      RCLCPP_WARN(get_logger(), "忽略任务刚启动时收到的二维码缓存结果");
      return;
    }

    qr_selected_sign_ = msg->sign_data;
    qr_detection_pose_ = current_pose_;
    qr_detection_progress_index_ = progress_index_;
    qr_switch_start_time_ = now();
    qr_mission_phase_ = QrMissionPhase::BRAKING;
    commanded_speed_ = 0.0;
    if (first_line_mode_ != "external") {
      publish_command(0.0, 0.0);
    }
    clear_local_plan();
    publish_state(
      first_line_mode_ == "external" ? "QR_WAIT_CONTROL_RELEASE" : "QR_BRAKING");
    RCLCPP_WARN(
      get_logger(),
      "二维码已识别：sign=%d（%s），立即终止冲刺；识别位姿=(%.3f, %.3f, %.1f deg)",
      qr_selected_sign_, qr_selected_sign_ == 3 ? "顺时针" : "逆时针",
      qr_detection_pose_.x, qr_detection_pose_.y,
      180.0 * qr_detection_pose_.yaw / std::acos(-1.0));
  }

  void control_started_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || external_control_started_) {
      return;
    }
    external_control_started_ = true;
    qr_mission_start_time_ = now();
    RCLCPP_INFO(get_logger(), "已收到 origincar_control 第一赛段启动确认");
  }

  void control_released_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data || external_control_released_) {
      return;
    }
    external_control_released_ = true;
    if (qr_mission_phase_ == QrMissionPhase::BRAKING) {
      qr_switch_start_time_ = now();
    }
    RCLCPP_INFO(get_logger(), "已收到 origincar_control 控制权释放确认");
  }

  void activate_qr_post_path()
  {
    const std::vector<PathPoint> & selected = select_auto_post_qr_path();
    recorded_path_ = selected;
    tracking_path_ = selected;
    for (auto & point : tracking_path_) {
      point.pose = compose(path_alignment_, point.pose);
    }
    recompute_arc_lengths(tracking_path_);
    post_qr_transition_end_index_ = tracking_path_.size();
    for (std::size_t index = 0U; index < tracking_path_.size(); ++index) {
      if (tracking_path_[index].pose.y >= post_qr_transition_end_y_) {
        post_qr_transition_end_index_ = index;
        break;
      }
    }

    progress_index_ = 0U;
    completed_ = false;
    direction_switch_pending_ = false;
    avoidance_active_ = false;
    reverse_side_rear_active_ = false;
    reverse_side_rear_forward_done_ = false;
    escape_phase_ = EscapePhase::IDLE;
    ring_photo_triggered_ = false;
    commanded_speed_ = 0.0;
    qr_mission_phase_ = QrMissionPhase::POST_QR;
    publish_tracking_path();
    publish_keepout_map();
    clear_local_plan();
    publish_command(0.0, 0.0);

    const Pose2d & head = tracking_path_.front().pose;
    publish_state("QR_FIXED_PATH_TRACK");
    RCLCPP_WARN(
      get_logger(),
      "二维码后切换固定路线：路径头=(%.2f, %.2f, %.1f deg)，初始方向=%s，随后%s绕环",
      head.x, head.y, 180.0 * head.yaw / std::acos(-1.0),
      tracking_path_.front().direction > 0 ? "前进" : "倒车",
      qr_selected_sign_ == 3 ? "顺时针" : "逆时针");
    if (post_qr_transition_end_index_ < tracking_path_.size()) {
      RCLCPP_INFO(
        get_logger(),
        "二维码后通道前限速区间：路径点1-%zu，入口Y阈值=%.2f米，正走=%.2f米/秒，倒车=%.2f米/秒",
        post_qr_transition_end_index_, post_qr_transition_end_y_,
        post_qr_transition_forward_speed_, post_qr_transition_reverse_speed_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "二维码后路线未找到Y>=%.2f米的通道入口点，将对整条路线应用通道前限速",
        post_qr_transition_end_y_);
    }
  }

  void maybe_trigger_ring_photo()
  {
    if (!enable_ring_photo_trigger_ || ring_photo_triggered_ ||
      qr_mission_phase_ != QrMissionPhase::POST_QR || tracking_path_.empty())
    {
      return;
    }

    const int ring_midpoint = std::clamp(
      (ring_start_path_point_ + ring_end_path_point_) / 2,
      1, static_cast<int>(tracking_path_.size()));
    const bool reverse_ring =
      tracking_path_[static_cast<std::size_t>(ring_midpoint - 1)].direction < 0;
    const int trigger_point = reverse_ring ?
      ring_photo_reverse_path_point_ : ring_photo_forward_path_point_;
    const int current_point = static_cast<int>(progress_index_) + 1;
    if (current_point < trigger_point) {
      return;
    }

    // Avoid photographing an avoidance/recovery pose that only happens to
    // have a low instantaneous yaw rate. Wait until normal path tracking resumes.
    if (avoidance_active_ || escape_phase_ != EscapePhase::IDLE) {
      return;
    }

    const bool straight_enough =
      std::abs(measured_yaw_rate_) <= ring_photo_max_abs_yaw_rate_;
    const bool trigger_deadline_reached =
      current_point >= trigger_point + ring_photo_force_after_points_;
    if (!straight_enough && !trigger_deadline_reached) {
      return;
    }

    std_msgs::msg::String trigger;
    trigger.data = reverse_ring ? "aurora" : "front";
    ring_photo_trigger_pub_->publish(trigger);
    ring_photo_triggered_ = true;
    RCLCPP_WARN(
      get_logger(),
      "触发环形区拍照：相机=%s 当前路径点=%d 配置点=%d 实测角速度=%.3f rad/s%s",
      reverse_ring ? "车尾Aurora（客户端旋转180度）" : "车头YOLO相机",
      current_point, trigger_point, measured_yaw_rate_,
      straight_enough ? "" : "（已到最迟触发点）");
  }


  void log_direction_segments() const
  {
    std::ostringstream summary;
    summary << "路径方向分段：";
    std::size_t begin = 0U;
    while (begin < recorded_path_.size()) {
      std::size_t end = begin;
      while (end + 1U < recorded_path_.size() &&
        recorded_path_[end + 1U].direction == recorded_path_[begin].direction)
      {
        ++end;
      }
      const double length = recorded_path_[end].arc_length - recorded_path_[begin].arc_length;
      summary << " [points " << begin + 1U << '-' << end + 1U << ' '
              << (recorded_path_[begin].direction > 0 ? "前进" : "倒车")
              << ' ' << std::fixed << std::setprecision(2) << length << "m]";
      begin = end + 1U;
    }
    RCLCPP_INFO(get_logger(), "%s", summary.str().c_str());
  }

  void align_path(const Pose2d & current_pose)
  {
    tracking_path_ = recorded_path_;
    const Pose2d alignment = align_path_to_start_ ?
      compose(current_pose, inverse(recorded_path_.front().pose)) : Pose2d{};
    path_alignment_ = alignment;
    for (auto & point : tracking_path_) {
      point.pose = compose(alignment, point.pose);
    }
    if (use_keepout_mask_) {
      std::size_t violating_points = 0U;
      for (const auto & point : tracking_path_) {
        if (forbidden_footprint_fraction(point.pose) > 0.0) {
          ++violating_points;
        }
      }
      const double violation_ratio = static_cast<double>(violating_points) /
        static_cast<double>(tracking_path_.size());
      mask_alignment_valid_ = violation_ratio <= max_path_mask_violation_ratio_;
      if (!mask_alignment_valid_) {
        RCLCPP_ERROR(
          get_logger(),
          "禁行区掩码对齐失败：%.1f%%的记录位姿落在黑区，请检查原点和旋转角",
          100.0 * violation_ratio);
      }
    }
    path_aligned_ = true;
    progress_index_ = 0U;
    if (enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::SPRINT) {
      qr_mission_start_time_ = now();
    }
    ring_localization_window_active_ = false;
    ring_localization_attempted_ = false;
    ring_localization_applied_ = false;
    ring_localization_y_samples_.clear();
    publish_tracking_path();
    publish_keepout_map();
    RCLCPP_INFO(
      get_logger(), "路径已对齐到位姿 (%.3f, %.3f, %.3f)，初始方向=%s",
      current_pose.x, current_pose.y, current_pose.yaw,
      tracking_path_.front().direction > 0 ? "前进" : "倒车");
  }

  void publish_tracking_path()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = path_frame_;
    for (const auto & point : tracking_path_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.pose.x;
      pose.pose.position.y = point.pose.y;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, point.pose.yaw);
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = quaternion_to_yaw(msg->pose.pose.orientation);
    measured_velocity_ = msg->twist.twist.linear.x;
    measured_yaw_rate_ = msg->twist.twist.angular.z;
    have_odom_ = true;
    last_odom_received_ = now();
    last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
    if (!path_aligned_ && !recorded_path_.empty()) {
      align_path(current_pose_);
    }
  }

  std::size_t remove_wall_line_clusters(
    std::vector<Eigen::Vector2d> & points, std::vector<std::size_t> & beam_indices) const
  {
    if (!filter_wall_line_clusters_ || points.size() <
      static_cast<std::size_t>(wall_cluster_min_points_))
    {
      return 0U;
    }

    std::vector<bool> reject(points.size(), false);
    std::size_t rejected_points = 0U;
    std::size_t begin = 0U;
    while (begin < points.size()) {
      std::size_t end = begin + 1U;
      while (end < points.size() &&
        beam_indices[end] - beam_indices[end - 1U] <=
        static_cast<std::size_t>(scan_cluster_max_beam_gap_) &&
        (points[end] - points[end - 1U]).norm() <= scan_cluster_max_distance_)
      {
        ++end;
      }

      const std::size_t count = end - begin;
      if (count >= static_cast<std::size_t>(wall_cluster_min_points_)) {
        Eigen::Vector2d mean = Eigen::Vector2d::Zero();
        for (std::size_t i = begin; i < end; ++i) {
          mean += points[i];
        }
        mean /= static_cast<double>(count);

        double covariance_xx = 0.0;
        double covariance_xy = 0.0;
        double covariance_yy = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
          const Eigen::Vector2d centered = points[i] - mean;
          covariance_xx += centered.x() * centered.x();
          covariance_xy += centered.x() * centered.y();
          covariance_yy += centered.y() * centered.y();
        }
        covariance_xx /= static_cast<double>(count);
        covariance_xy /= static_cast<double>(count);
        covariance_yy /= static_cast<double>(count);

        const double trace = covariance_xx + covariance_yy;
        const double discriminant = std::sqrt(
          std::max(0.0, (covariance_xx - covariance_yy) *
          (covariance_xx - covariance_yy) + 4.0 * covariance_xy * covariance_xy));
        const double major_variance = 0.5 * (trace + discriminant);
        const double minor_variance = std::max(0.0, 0.5 * (trace - discriminant));
        const double linearity = major_variance > 1e-9 ?
          1.0 - minor_variance / major_variance : 0.0;

        Eigen::Vector2d principal_axis;
        if (std::abs(covariance_xy) > 1e-9) {
          principal_axis = Eigen::Vector2d(major_variance - covariance_yy, covariance_xy);
          principal_axis.normalize();
        } else {
          principal_axis = covariance_xx >= covariance_yy ?
            Eigen::Vector2d(1.0, 0.0) : Eigen::Vector2d(0.0, 1.0);
        }
        double minimum_projection = std::numeric_limits<double>::infinity();
        double maximum_projection = -std::numeric_limits<double>::infinity();
        for (std::size_t i = begin; i < end; ++i) {
          const double projection = principal_axis.dot(points[i] - mean);
          minimum_projection = std::min(minimum_projection, projection);
          maximum_projection = std::max(maximum_projection, projection);
        }
        const double length = maximum_projection - minimum_projection;
        const double thickness = std::sqrt(minor_variance);
        if (length >= wall_cluster_min_length_ &&
          thickness <= wall_cluster_max_thickness_ &&
          linearity >= wall_cluster_min_linearity_)
        {
          for (std::size_t i = begin; i < end; ++i) {
            reject[i] = true;
          }
          rejected_points += count;
        }
      }
      begin = end;
    }

    if (rejected_points == 0U) {
      return 0U;
    }
    std::vector<Eigen::Vector2d> retained_points;
    std::vector<std::size_t> retained_indices;
    retained_points.reserve(points.size() - rejected_points);
    retained_indices.reserve(beam_indices.size() - rejected_points);
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (!reject[i]) {
        retained_points.push_back(points[i]);
        retained_indices.push_back(beam_indices[i]);
      }
    }
    points = std::move(retained_points);
    beam_indices = std::move(retained_indices);
    return rejected_points;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<Eigen::Vector2d> current_scan_points;
    std::vector<std::size_t> current_scan_indices;
    current_scan_points.reserve(msg->ranges.size());
    current_scan_indices.reserve(msg->ranges.size());
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range) || range < std::max(0.20f, msg->range_min) ||
        range > std::min(8.0f, msg->range_max))
      {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      Eigen::Vector2d point = transform_point(
        laser_extrinsic_, Eigen::Vector2d(range * std::cos(angle), range * std::sin(angle)));
      if (deskew_obstacle_scan_) {
        const double point_period = msg->time_increment > 0.0 ?
          msg->time_increment :
          (msg->ranges.size() > 1U ?
          msg->scan_time / static_cast<double>(msg->ranges.size() - 1U) : 0.0);
        double index_zero_to_end = 0.0;
        if (scan_index_time_reversed_ && msg->scan_time > 0.0) {
          double normalized_min_angle = std::fmod(static_cast<double>(msg->angle_min), 2.0 * kPi);
          if (normalized_min_angle < 0.0) {
            normalized_min_angle += 2.0 * kPi;
          }
          index_zero_to_end = normalized_min_angle * msg->scan_time / (2.0 * kPi);
        }
        const double scan_duration =
          point_period * static_cast<double>(msg->ranges.size() > 0U ? msg->ranges.size() - 1U : 0U);
        const double point_time = point_period * static_cast<double>(i);
        const double dt_to_end = scan_index_time_reversed_ ?
          index_zero_to_end + point_time : std::max(0.0, scan_duration - point_time);
        const double delta_yaw = measured_yaw_rate_ * dt_to_end;
        const double deskew_c = std::cos(-delta_yaw);
        const double deskew_s = std::sin(-delta_yaw);
        point.x() -= measured_velocity_ * dt_to_end;
        point = Eigen::Vector2d(
          deskew_c * point.x() - deskew_s * point.y(),
          deskew_s * point.x() + deskew_c * point.y());
      }
      current_scan_points.push_back(point);
      current_scan_indices.push_back(i);
    }

    const std::size_t valid_point_count = current_scan_points.size();
    if (enable_lidar_noise_filter_ && scan_cluster_min_points_ > 1) {
      std::vector<Eigen::Vector2d> clustered_points;
      std::vector<std::size_t> clustered_indices;
      clustered_points.reserve(current_scan_points.size());
      clustered_indices.reserve(current_scan_indices.size());
      for (std::size_t i = 0; i < current_scan_points.size(); ++i) {
        int support = 1;
        for (std::size_t j = i; j > 0U;) {
          --j;
          if (current_scan_indices[i] - current_scan_indices[j] >
            static_cast<std::size_t>(scan_cluster_max_beam_gap_))
          {
            break;
          }
          if ((current_scan_points[i] - current_scan_points[j]).norm() <=
            scan_cluster_max_distance_)
          {
            ++support;
          }
        }
        for (std::size_t j = i + 1U; j < current_scan_points.size(); ++j) {
          if (current_scan_indices[j] - current_scan_indices[i] >
            static_cast<std::size_t>(scan_cluster_max_beam_gap_))
          {
            break;
          }
          if ((current_scan_points[i] - current_scan_points[j]).norm() <=
            scan_cluster_max_distance_)
          {
            ++support;
          }
        }
        if (support >= scan_cluster_min_points_) {
          clustered_points.push_back(current_scan_points[i]);
          clustered_indices.push_back(current_scan_indices[i]);
        }
      }
      current_scan_points = std::move(clustered_points);
      current_scan_indices = std::move(clustered_indices);
    }

    const std::size_t wall_filtered_count =
      remove_wall_line_clusters(current_scan_points, current_scan_indices);

    if (have_odom_) {
      const rclcpp::Time received_time = now();
      ++scan_sequence_;
      for (const auto & point : current_scan_points) {
        const Eigen::Vector2d world_point = transform_point(current_pose_, point);
        if (is_field_boundary_return(world_point)) {
          continue;
        }
        update_obstacle_memory(world_point, received_time);
      }
      refresh_obstacle_points(received_time);
      publish_obstacle_cloud(received_time);
      if (enable_lidar_noise_filter_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "雷达过滤：有效=%zu 保留=%zu 墙体=%zu 已确认障碍=%zu 待确认=%zu "
          "车速=%.2f米/秒 点云融合权重=%.2f",
          valid_point_count, current_scan_points.size(), wall_filtered_count, laser_points_.size(),
          obstacle_memory_.size() - laser_points_.size(), measured_velocity_,
          obstacle_measurement_weight());
      }
    } else {
      laser_points_ = std::move(current_scan_points);
    }
    have_scan_ = true;
    last_scan_received_ = now();
  }

  std::uint64_t obstacle_voxel_key(int voxel_x, int voxel_y) const
  {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(voxel_x)) << 32U) |
           static_cast<std::uint32_t>(voxel_y);
  }

  void update_obstacle_memory(
    const Eigen::Vector2d & world_point, const rclcpp::Time & received_time)
  {
    const int voxel_x = static_cast<int>(
      std::floor(world_point.x() / obstacle_memory_voxel_size_));
    const int voxel_y = static_cast<int>(
      std::floor(world_point.y() / obstacle_memory_voxel_size_));

    TimedObstaclePoint * best_match = nullptr;
    double best_distance = obstacle_confirmation_radius_;
    const int neighbor_cells = std::max(
      1, static_cast<int>(
        std::ceil(obstacle_confirmation_radius_ / obstacle_memory_voxel_size_)));
    for (int dx = -neighbor_cells; dx <= neighbor_cells; ++dx) {
      for (int dy = -neighbor_cells; dy <= neighbor_cells; ++dy) {
        const auto iterator = obstacle_memory_.find(obstacle_voxel_key(voxel_x + dx, voxel_y + dy));
        if (iterator == obstacle_memory_.end() ||
          iterator->second.last_scan_id == scan_sequence_)
        {
          continue;
        }
        const double age = (received_time - iterator->second.stamp).seconds();
        const double allowed_age = iterator->second.confirmed ?
          obstacle_memory_duration_ : obstacle_confirmation_window_;
        const double distance = (world_point - iterator->second.world_point).norm();
        if (age >= 0.0 && age <= allowed_age && distance <= best_distance) {
          best_distance = distance;
          best_match = &iterator->second;
        }
      }
    }

    if (best_match != nullptr) {
      if (!best_match->confirmed &&
        (received_time - best_match->first_seen).seconds() > obstacle_confirmation_window_)
      {
        best_match->world_point = world_point;
        best_match->first_seen = received_time;
        best_match->stamp = received_time;
        best_match->last_scan_id = scan_sequence_;
        best_match->hit_count = 1;
        return;
      }
      const double measurement_weight = obstacle_measurement_weight();
      best_match->world_point =
        (1.0 - measurement_weight) * best_match->world_point +
        measurement_weight * world_point;
      best_match->stamp = received_time;
      best_match->last_scan_id = scan_sequence_;
      best_match->hit_count = std::min(best_match->hit_count + 1, obstacle_confirmation_hits_);
      best_match->confirmed = best_match->confirmed ||
        best_match->hit_count >= obstacle_confirmation_hits_;
      return;
    }

    const std::uint64_t key = obstacle_voxel_key(voxel_x, voxel_y);
    auto existing = obstacle_memory_.find(key);
    if (existing != obstacle_memory_.end() && existing->second.last_scan_id == scan_sequence_) {
      // Points from the same scan have identical motion reliability. Average
      // them geometrically; speed weighting belongs only to cross-scan fusion.
      existing->second.world_point = 0.5 * (existing->second.world_point + world_point);
      return;
    }

    TimedObstaclePoint candidate;
    candidate.world_point = world_point;
    candidate.first_seen = received_time;
    candidate.stamp = received_time;
    candidate.last_scan_id = scan_sequence_;
    candidate.hit_count = 1;
    candidate.confirmed = !enable_lidar_noise_filter_ || obstacle_confirmation_hits_ <= 1;
    obstacle_memory_[key] = candidate;
  }

  bool is_field_boundary_return(const Eigen::Vector2d & world_point) const
  {
    if (!filter_field_boundary_returns_) {
      return false;
    }
    return world_point.x() <= field_min_x_ + field_boundary_filter_band_ ||
           world_point.x() >= field_max_x_ - field_boundary_filter_band_ ||
           world_point.y() <= field_min_y_ + field_boundary_filter_band_ ||
           world_point.y() >= field_max_y_ - field_boundary_filter_band_;
  }

  void publish_obstacle_cloud(const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = path_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    const std::size_t confirmed_count = static_cast<std::size_t>(std::count_if(
      obstacle_memory_.begin(), obstacle_memory_.end(),
      [](const auto & entry) {return entry.second.confirmed;}));
    modifier.resize(confirmed_count);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & entry : obstacle_memory_) {
      if (!entry.second.confirmed) {
        continue;
      }
      *x = static_cast<float>(entry.second.world_point.x());
      *y = static_cast<float>(entry.second.world_point.y());
      *z = 0.04F;
      ++x;
      ++y;
      ++z;
    }
    obstacle_cloud_pub_->publish(cloud);
  }

  void publish_keepout_map()
  {
    if (!use_keepout_mask_ || keepout_pixels_.empty()) {
      return;
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = path_frame_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(keepout_resolution_);
    grid.info.width = static_cast<std::uint32_t>(keepout_width_);
    grid.info.height = static_cast<std::uint32_t>(keepout_height_);
    const Pose2d map_origin = compose(path_alignment_, keepout_origin_);
    grid.info.origin.position.x = map_origin.x;
    grid.info.origin.position.y = map_origin.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, map_origin.yaw);
    grid.info.origin.orientation.x = quaternion.x();
    grid.info.origin.orientation.y = quaternion.y();
    grid.info.origin.orientation.z = quaternion.z();
    grid.info.origin.orientation.w = quaternion.w();
    grid.data.resize(keepout_pixels_.size());
    for (int row_from_bottom = 0; row_from_bottom < keepout_height_; ++row_from_bottom) {
      const int pgm_row = keepout_height_ - 1 - row_from_bottom;
      for (int column = 0; column < keepout_width_; ++column) {
        const std::size_t pgm_index = static_cast<std::size_t>(pgm_row) *
          static_cast<std::size_t>(keepout_width_) + static_cast<std::size_t>(column);
        const std::size_t grid_index = static_cast<std::size_t>(row_from_bottom) *
          static_cast<std::size_t>(keepout_width_) + static_cast<std::size_t>(column);
        grid.data[grid_index] =
          keepout_pixels_[pgm_index] >= keepout_white_threshold_ ? 0 : 100;
      }
    }
    keepout_map_pub_->publish(grid);
  }

  void refresh_obstacle_points(const rclcpp::Time & current_time)
  {
    laser_points_.clear();
    laser_points_.reserve(obstacle_memory_.size());
    const Pose2d world_to_base = inverse(current_pose_);
    for (auto iterator = obstacle_memory_.begin(); iterator != obstacle_memory_.end();) {
      const double age = (current_time - iterator->second.stamp).seconds();
      const Eigen::Vector2d local_point =
        transform_point(world_to_base, iterator->second.world_point);
      const double maximum_age = iterator->second.confirmed ?
        obstacle_memory_duration_ : unconfirmed_obstacle_ttl_;
      if (age > maximum_age || local_point.norm() > obstacle_memory_radius_) {
        iterator = obstacle_memory_.erase(iterator);
        continue;
      }
      if (iterator->second.confirmed) {
        laser_points_.push_back(local_point);
      }
      ++iterator;
    }
  }

  void vision_callback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    bool found = false;
    double largest_height = 0.0;
    double selected_center = image_center_x_;
    double selected_width = 0.0;
    bool parking_found = false;
    double largest_parking_area = 0.0;
    double parking_center = parking_center_x_;
    double parking_ymax = 0.0;
    std::vector<std::pair<double, double>> hat_intervals;
    std::vector<std::pair<double, double>> parking_intervals;

    for (const auto & target : msg->targets) {
      if (target.type == parking_target_type_) {
        for (const auto & roi : target.rois) {
          if (static_cast<double>(roi.confidence) < parking_min_confidence_) {
            continue;
          }
          const double xmin = static_cast<double>(roi.rect.x_offset);
          const double ymin = static_cast<double>(roi.rect.y_offset);
          const double width = static_cast<double>(roi.rect.width);
          const double height = static_cast<double>(roi.rect.height);
          const double area = width * height;
          parking_intervals.emplace_back(xmin, xmin + width);
          if (area > largest_parking_area) {
            largest_parking_area = area;
            parking_center = xmin + 0.5 * width;
            parking_ymax = ymin + height;
            parking_found = true;
          }
        }
      }
      if (target.type != "hat") {
        continue;
      }
      for (const auto & roi : target.rois) {
        const double xmin = static_cast<double>(roi.rect.x_offset);
        const double ymin = static_cast<double>(roi.rect.y_offset);
        const double width = static_cast<double>(roi.rect.width);
        const double height = static_cast<double>(roi.rect.height);
        const double xmax = xmin + width;
        hat_intervals.emplace_back(xmin, xmax);
        const double left_image_limit = image_width_ * (400.0 / 1920.0);
        const double right_image_limit = image_width_ * (1600.0 / 1920.0);
        if (xmax < left_image_limit || xmin > right_image_limit) {
          continue;
        }
        if (height > largest_height) {
          largest_height = height;
          selected_center = xmin + 0.5 * width;
          selected_width = width;
          found = true;
        }
        (void)ymin;
      }
    }

    visual_triggered_ = false;
    if (found) {
      const double ratio = largest_height / image_height_;
      visual_height_ratio_ = ratio;
      visual_selected_center_ = selected_center;
      visual_selected_width_ = selected_width;
      const double horizontal_gate = std::min(hat_horizontal_limit_, hat_width_scale_ * selected_width);
      visual_triggered_ = ratio >= hat_force_trigger_ratio_ ||
        (ratio >= hat_trigger_ratio_ && std::abs(selected_center - image_center_x_) < horizontal_gate);
      if (visual_triggered_) {
        // This is the same default side decision and target-pixel construction used by origincar_control.
        visual_side_code_ = selected_center > image_center_x_ - hat_direction_bias_px_ ? -1 : 1;
        visual_target_error_ =
          selected_center + hat_avoid_target_scale_ * visual_side_code_ * selected_width - image_center_x_;
        last_visual_received_ = now();
      }
    } else {
      visual_height_ratio_ = 0.0;
      visual_selected_width_ = 0.0;
    }

    // Match second_line's rush_p gate exactly: at least one P must exist, and
    // no HAT may overlap any P's horizontal corridor (including the margin).
    if (parking_found) {
      for (const auto & parking_interval : parking_intervals) {
        for (const auto & hat_interval : hat_intervals) {
          if (hat_interval.second >= parking_interval.first - parking_obstacle_margin_px_ &&
            hat_interval.first <= parking_interval.second + parking_obstacle_margin_px_)
          {
            parking_found = false;
            break;
          }
        }
        if (!parking_found) {
          break;
        }
      }
    }
    parking_detection_valid_ = parking_found;
    if (parking_found) {
      parking_target_center_x_ = parking_center;
      parking_target_ymax_ = parking_ymax;
      last_parking_detection_ = now();
    }
  }

  static bool fit_pixel_line(
    const std::vector<Eigen::Vector2d> & points,
    double & slope, double & intercept, double & rmse)
  {
    if (points.size() < 3U) {
      return false;
    }
    double mean_y = 0.0;
    double mean_x = 0.0;
    for (const auto & point : points) {
      mean_y += point.y();
      mean_x += point.x();
    }
    mean_y /= static_cast<double>(points.size());
    mean_x /= static_cast<double>(points.size());

    double denominator = 0.0;
    double numerator = 0.0;
    for (const auto & point : points) {
      const double dy = point.y() - mean_y;
      denominator += dy * dy;
      numerator += dy * (point.x() - mean_x);
    }
    if (denominator < 1e-6) {
      return false;
    }
    slope = numerator / denominator;
    intercept = mean_x - slope * mean_y;
    double squared_error = 0.0;
    for (const auto & point : points) {
      const double error = point.x() - (slope * point.y() + intercept);
      squared_error += error * error;
    }
    rmse = std::sqrt(squared_error / static_cast<double>(points.size()));
    return true;
  }

  void ring_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_ring_depth_ = msg;
  }

  void ring_camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_ring_camera_info_ = msg;
  }

  bool ring_depth_point(
    int u, int v, double fx, double fy, double cx, double cy,
    Eigen::Vector3d & point) const
  {
    if (!latest_ring_depth_ ||
      (latest_ring_depth_->encoding != "16UC1" &&
      latest_ring_depth_->encoding != "mono16") ||
      latest_ring_depth_->step < 2U * latest_ring_depth_->width)
    {
      return false;
    }
    const int width = static_cast<int>(latest_ring_depth_->width);
    const int height = static_cast<int>(latest_ring_depth_->height);
    std::vector<std::uint16_t> samples;
    const int radius = ring_depth_search_radius_;
    samples.reserve(static_cast<std::size_t>((2 * radius + 1) * (2 * radius + 1)));
    for (int y = std::max(0, v - radius); y <= std::min(height - 1, v + radius); ++y) {
      const std::uint8_t * row = latest_ring_depth_->data.data() +
        static_cast<std::size_t>(y) * latest_ring_depth_->step;
      for (int x = std::max(0, u - radius); x <= std::min(width - 1, u + radius); ++x) {
        const std::uint8_t * pixel = row + 2U * static_cast<std::size_t>(x);
        const std::uint16_t raw = latest_ring_depth_->is_bigendian != 0U ?
          static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(pixel[0]) << 8U) | pixel[1]) :
          static_cast<std::uint16_t>(
          pixel[0] | (static_cast<std::uint16_t>(pixel[1]) << 8U));
        const double depth = ring_depth_scale_ * static_cast<double>(raw);
        if (raw != 0U && depth >= ring_depth_min_range_ && depth <= ring_depth_max_range_) {
          samples.push_back(raw);
        }
      }
    }
    if (samples.empty()) {
      return false;
    }
    const auto middle = samples.begin() + static_cast<std::ptrdiff_t>(samples.size() / 2U);
    std::nth_element(samples.begin(), middle, samples.end());
    const double depth = ring_depth_scale_ * static_cast<double>(*middle);
    point = Eigen::Vector3d(
      (static_cast<double>(u) - cx) * depth / fx,
      (static_cast<double>(v) - cy) * depth / fy,
      depth);
    return point.allFinite();
  }

  RingMetricObservation estimate_ring_metric_depth(
    const sensor_msgs::msg::Image & image,
    const std::vector<RingRowRun> & runs)
  {
    RingMetricObservation observation;
    ring_localization_depth_valid_ = false;
    if (!ring_localization_use_depth_ || !latest_ring_depth_ || !latest_ring_camera_info_) {
      return observation;
    }
    if (latest_ring_depth_->width != image.width || latest_ring_depth_->height != image.height ||
      latest_ring_depth_->data.size() <
      static_cast<std::size_t>(latest_ring_depth_->step) * latest_ring_depth_->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "环形深度图与RGB掩码尺寸不一致：深度=%ux%u 二值图=%ux%u",
        latest_ring_depth_->width, latest_ring_depth_->height, image.width, image.height);
      return observation;
    }
    const rclcpp::Time image_stamp(image.header.stamp);
    const rclcpp::Time depth_stamp(latest_ring_depth_->header.stamp);
    const double stamp_delta = std::abs((image_stamp - depth_stamp).seconds());
    if (stamp_delta > ring_depth_sync_tolerance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "环形RGB与深度图时间戳相差%.3f秒，超过允许值%.3f秒",
        stamp_delta, ring_depth_sync_tolerance_);
      return observation;
    }

    const auto & info = *latest_ring_camera_info_;
    const double info_width = info.width > 0U ? static_cast<double>(info.width) : image.width;
    const double info_height = info.height > 0U ? static_cast<double>(info.height) : image.height;
    const double scale_x = static_cast<double>(image.width) / info_width;
    const double scale_y = static_cast<double>(image.height) / info_height;
    const double fx = info.k[0] * scale_x;
    const double fy = info.k[4] * scale_y;
    const double cx = info.k[2] * scale_x;
    const double cy = info.k[5] * scale_y;
    if (!(fx > 1.0 && fy > 1.0) || !std::isfinite(cx) || !std::isfinite(cy)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Aurora RGB camera_info has invalid intrinsics");
      return observation;
    }

    struct MetricRun
    {
      bool have_left{false};
      bool have_right{false};
      Eigen::Vector3d left{Eigen::Vector3d::Zero()};
      Eigen::Vector3d right{Eigen::Vector3d::Zero()};
    };
    std::vector<MetricRun> metric_runs;
    std::vector<Eigen::Vector3d> ground_points;
    metric_runs.reserve(runs.size());
    ground_points.reserve(5U * runs.size());
    constexpr int edge_margin = 3;
    for (const auto & run : runs) {
      if (run.y < ring_localization_min_row_ratio_ * (image.height - 1.0)) {
        continue;
      }
      MetricRun metric;
      const int y = static_cast<int>(std::lround(run.y));
      metric.have_left = run.begin > edge_margin && ring_depth_point(
        std::min(run.end, run.begin + 2), y, fx, fy, cx, cy, metric.left);
      metric.have_right = run.end < static_cast<int>(image.width) - 1 - edge_margin &&
        ring_depth_point(
        std::max(run.begin, run.end - 2), y, fx, fy, cx, cy, metric.right);
      for (int sample = 0; sample < 5; ++sample) {
        const double ratio = static_cast<double>(sample) / 4.0;
        const int x = static_cast<int>(std::lround(
          static_cast<double>(run.begin) + ratio * (run.end - run.begin)));
        Eigen::Vector3d point;
        if (ring_depth_point(x, y, fx, fy, cx, cy, point)) {
          ground_points.push_back(point);
        }
      }
      if (metric.have_left || metric.have_right) {
        metric_runs.push_back(metric);
      }
    }
    if (metric_runs.size() < static_cast<std::size_t>(ring_depth_min_center_points_) ||
      ground_points.size() < 3U * static_cast<std::size_t>(ring_depth_min_center_points_))
    {
      return observation;
    }

    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto & point : ground_points) {
      mean += point;
    }
    mean /= static_cast<double>(ground_points.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto & point : ground_points) {
      const Eigen::Vector3d centered = point - mean;
      covariance += centered * centered.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()(1) < 1e-4) {
      return observation;
    }
    const Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    double plane_squared_error = 0.0;
    for (const auto & point : ground_points) {
      const double error = normal.dot(point - mean);
      plane_squared_error += error * error;
    }
    const double plane_rmse = std::sqrt(
      plane_squared_error / static_cast<double>(ground_points.size()));
    if (plane_rmse > ring_depth_max_fit_rmse_) {
      return observation;
    }
    const Eigen::Vector3d ground_origin = normal * normal.dot(mean);
    Eigen::Vector3d right = Eigen::Vector3d::UnitX() -
      normal * normal.dot(Eigen::Vector3d::UnitX());
    if (right.norm() < 0.2) {
      return observation;
    }
    right.normalize();
    Eigen::Vector3d forward = normal.cross(right);
    if (forward.dot(Eigen::Vector3d::UnitZ()) < 0.0) {
      forward = -forward;
    }
    forward.normalize();

    std::size_t white_cross_section_samples = 0U;
    std::size_t tested_cross_section_samples = 0U;
    for (const auto & metric : metric_runs) {
      Eigen::Vector3d left_boundary;
      Eigen::Vector3d right_boundary;
      if (metric.have_left && metric.have_right) {
        left_boundary = metric.left;
        right_boundary = metric.right;
      } else if (metric.have_left) {
        left_boundary = metric.left;
        right_boundary = metric.left + ring_localization_lane_width_ * right;
      } else {
        right_boundary = metric.right;
        left_boundary = metric.right - ring_localization_lane_width_ * right;
      }
      for (int sample = 0; sample <= 10; ++sample) {
        const double ratio = static_cast<double>(sample) / 10.0;
        const Eigen::Vector3d point =
          (1.0 - ratio) * left_boundary + ratio * right_boundary;
        if (point.z() <= ring_depth_min_range_) {
          continue;
        }
        const int u = static_cast<int>(std::lround(fx * point.x() / point.z() + cx));
        const int v = static_cast<int>(std::lround(fy * point.y() / point.z() + cy));
        if (u < 0 || u >= static_cast<int>(image.width) ||
          v < 0 || v >= static_cast<int>(image.height))
        {
          continue;
        }
        const std::uint8_t * row = image.data.data() +
          static_cast<std::size_t>(v) * image.step;
        ++tested_cross_section_samples;
        if (row[u] > ring_image_threshold_) {
          ++white_cross_section_samples;
        }
      }
    }
    if (tested_cross_section_samples <
      5U * static_cast<std::size_t>(ring_depth_min_center_points_))
    {
      return observation;
    }
    observation.solid_fraction =
      static_cast<double>(white_cross_section_samples) /
      static_cast<double>(tested_cross_section_samples);
    if (observation.solid_fraction < ring_localization_min_solid_fraction_) {
      return observation;
    }

    std::vector<Eigen::Vector2d> centers;
    centers.reserve(metric_runs.size());
    double valid_width_sum = 0.0;
    int valid_width_count = 0;
    for (const auto & metric : metric_runs) {
      const auto ground_coordinates = [&](const Eigen::Vector3d & point) {
          const Eigen::Vector3d relative = point - ground_origin;
          return Eigen::Vector2d(
            ring_camera_x_ + forward.dot(relative),
            ring_camera_y_ - right.dot(relative));
        };
      if (metric.have_left && metric.have_right) {
        const Eigen::Vector2d left = ground_coordinates(metric.left);
        const Eigen::Vector2d right_boundary = ground_coordinates(metric.right);
        const double measured_width = left.y() - right_boundary.y();
        if (measured_width <= 0.0 ||
          std::abs(measured_width - ring_localization_lane_width_) >
          ring_depth_max_lane_width_error_)
        {
          continue;
        }
        centers.emplace_back(
          0.5 * (left.x() + right_boundary.x()),
          0.5 * (left.y() + right_boundary.y()));
        valid_width_sum += measured_width;
        ++valid_width_count;
      } else if (metric.have_left) {
        Eigen::Vector2d center = ground_coordinates(metric.left);
        center.y() -= 0.5 * ring_localization_lane_width_;
        centers.push_back(center);
      } else if (metric.have_right) {
        Eigen::Vector2d center = ground_coordinates(metric.right);
        center.y() += 0.5 * ring_localization_lane_width_;
        centers.push_back(center);
      }
    }
    if (centers.size() < static_cast<std::size_t>(ring_depth_min_center_points_)) {
      return observation;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    double minimum_x = std::numeric_limits<double>::infinity();
    double maximum_x = -std::numeric_limits<double>::infinity();
    for (const auto & center : centers) {
      mean_x += center.x();
      mean_y += center.y();
      minimum_x = std::min(minimum_x, center.x());
      maximum_x = std::max(maximum_x, center.x());
    }
    mean_x /= static_cast<double>(centers.size());
    mean_y /= static_cast<double>(centers.size());
    if (maximum_x - minimum_x < ring_depth_min_forward_span_) {
      return observation;
    }
    double numerator = 0.0;
    double denominator = 0.0;
    for (const auto & center : centers) {
      const double dx = center.x() - mean_x;
      numerator += dx * (center.y() - mean_y);
      denominator += dx * dx;
    }
    if (denominator < 1e-6) {
      return observation;
    }
    const double slope = numerator / denominator;
    const double intercept = mean_y - slope * mean_x;
    double squared_error = 0.0;
    for (const auto & center : centers) {
      const double error = center.y() - (slope * center.x() + intercept);
      squared_error += error * error;
    }
    const double rmse = std::sqrt(squared_error / static_cast<double>(centers.size()));
    observation.heading_error = -std::atan(slope);
    if (rmse > ring_depth_max_fit_rmse_ ||
      std::abs(observation.heading_error) > ring_localization_max_heading_error_)
    {
      return observation;
    }

    observation.valid = true;
    // intercept is lane-center y in the vehicle frame; the vehicle's offset
    // from that center has the opposite sign.
    observation.lateral_m = -intercept;
    observation.fit_rmse_px = rmse;
    if (valid_width_count > 0) {
      observation.lane_width_px = valid_width_sum / static_cast<double>(valid_width_count);
    }
    ring_localization_depth_valid_ = true;
    return observation;
  }

  RingMetricObservation estimate_ring_metric(
    const sensor_msgs::msg::Image & image,
    const std::vector<RingRowRun> & runs)
  {
    RingMetricObservation observation;
    const double width = static_cast<double>(image.width);
    const double height = static_cast<double>(image.height);
    const double minimum_row = ring_localization_min_row_ratio_ * (height - 1.0);
    const double vehicle_row = ring_localization_vehicle_row_ratio_ * (height - 1.0);
    const int edge_margin = 3;
    std::vector<Eigen::Vector2d> left_points;
    std::vector<Eigen::Vector2d> right_points;
    for (const auto & run : runs) {
      if (run.y < minimum_row || run.width < 0.35 * ring_localization_min_lane_width_px_) {
        continue;
      }
      if (run.begin > edge_margin) {
        left_points.emplace_back(static_cast<double>(run.begin), run.y);
      }
      if (run.end < static_cast<int>(image.width) - 1 - edge_margin) {
        right_points.emplace_back(static_cast<double>(run.end), run.y);
      }
    }

    double left_slope = 0.0;
    double left_intercept = 0.0;
    double left_rmse = std::numeric_limits<double>::infinity();
    double right_slope = 0.0;
    double right_intercept = 0.0;
    double right_rmse = std::numeric_limits<double>::infinity();
    const bool have_left = fit_pixel_line(
      left_points, left_slope, left_intercept, left_rmse);
    const bool have_right = fit_pixel_line(
      right_points, right_slope, right_intercept, right_rmse);
    if (!have_left && !have_right) {
      return observation;
    }

    const double left_at_vehicle = left_slope * vehicle_row + left_intercept;
    const double right_at_vehicle = right_slope * vehicle_row + right_intercept;
    const double cached_width = ring_localization_expected_width_px_ > 0.0 ?
      ring_localization_expected_width_px_ : ring_localization_width_px_estimate_;
    double lane_center = 0.0;
    double lane_width = cached_width;
    double center_slope = 0.0;
    if (have_left && have_right) {
      lane_width = right_at_vehicle - left_at_vehicle;
      lane_center = 0.5 * (left_at_vehicle + right_at_vehicle);
      center_slope = 0.5 * (left_slope + right_slope);
      observation.fit_rmse_px = std::max(left_rmse, right_rmse);
    } else {
      if (cached_width < ring_localization_min_lane_width_px_) {
        return observation;
      }
      if (have_left) {
        lane_center = left_at_vehicle + 0.5 * cached_width;
        center_slope = left_slope;
        observation.fit_rmse_px = left_rmse;
      } else {
        lane_center = right_at_vehicle - 0.5 * cached_width;
        center_slope = right_slope;
        observation.fit_rmse_px = right_rmse;
      }
    }

    if (lane_width < ring_localization_min_lane_width_px_ || lane_width > 0.95 * width ||
      observation.fit_rmse_px > ring_localization_max_fit_rmse_ratio_ * lane_width)
    {
      return observation;
    }

    const double horizon_row = ring_localization_horizon_row_ratio_ * (height - 1.0);
    const int solid_begin_y = std::max(
      static_cast<int>(minimum_row), static_cast<int>(vehicle_row - 0.18 * height));
    const int solid_end_y = std::min(
      static_cast<int>(image.height) - 2, static_cast<int>(vehicle_row));
    std::size_t white_pixels = 0U;
    std::size_t tested_pixels = 0U;
    for (int y = solid_begin_y; y <= solid_end_y; y += 2) {
      double predicted_center = 0.0;
      double predicted_width;
      if (have_left && have_right) {
        const double predicted_left = left_slope * static_cast<double>(y) + left_intercept;
        const double predicted_right = right_slope * static_cast<double>(y) + right_intercept;
        predicted_center = 0.5 * (predicted_left + predicted_right);
        predicted_width = predicted_right - predicted_left;
      } else {
        const double perspective_scale = clamp(
          (static_cast<double>(y) - horizon_row) /
          std::max(1.0, vehicle_row - horizon_row), 0.35, 1.10);
        predicted_width = lane_width * perspective_scale;
        const double visible_boundary = have_left ?
          left_slope * static_cast<double>(y) + left_intercept :
          right_slope * static_cast<double>(y) + right_intercept;
        predicted_center = have_left ?
          visible_boundary + 0.5 * predicted_width :
          visible_boundary - 0.5 * predicted_width;
      }
      const int begin_x = std::max(
        0, static_cast<int>(std::floor(predicted_center - 0.30 * predicted_width)));
      const int end_x = std::min(
        static_cast<int>(image.width) - 1,
        static_cast<int>(std::ceil(predicted_center + 0.30 * predicted_width)));
      if (end_x <= begin_x) {
        continue;
      }
      const std::uint8_t * row =
        image.data.data() + static_cast<std::size_t>(y) * image.step;
      for (int x = begin_x; x <= end_x; x += 2) {
        ++tested_pixels;
        if (row[x] > ring_image_threshold_) {
          ++white_pixels;
        }
      }
    }
    if (tested_pixels == 0U) {
      return observation;
    }
    observation.solid_fraction = static_cast<double>(white_pixels) /
      static_cast<double>(tested_pixels);
    observation.heading_error = std::atan(center_slope);
    if (observation.solid_fraction < ring_localization_min_solid_fraction_ ||
      std::abs(observation.heading_error) > ring_localization_max_heading_error_)
    {
      return observation;
    }

    if (have_left && have_right && ring_localization_expected_width_px_ <= 0.0) {
      if (ring_localization_width_px_estimate_ <= 0.0) {
        ring_localization_width_px_estimate_ = lane_width;
      } else {
        ring_localization_width_px_estimate_ =
          0.85 * ring_localization_width_px_estimate_ + 0.15 * lane_width;
      }
    }
    observation.valid = true;
    observation.lane_center_px = lane_center;
    observation.lane_width_px = lane_width;
    observation.lateral_m =
      (lane_center - ring_localization_camera_center_x_ratio_ * width) *
      ring_localization_lane_width_ / lane_width;
    return observation;
  }

  void update_ring_localization(const RingMetricObservation & observation)
  {
    if (!enable_ring_localization_correction_ || ring_localization_attempted_ ||
      !path_aligned_ || !have_odom_ || tracking_path_.empty())
    {
      return;
    }
    const int one_based_point = static_cast<int>(progress_index_) + 1;
    const bool in_window = one_based_point >= ring_localization_start_path_point_ &&
      one_based_point <= ring_localization_end_path_point_;
    if (!in_window) {
      if (ring_localization_window_active_ &&
        one_based_point > ring_localization_end_path_point_)
      {
        ring_localization_window_active_ = false;
        ring_localization_attempted_ = true;
        RCLCPP_WARN(
          get_logger(), "环形区Y坐标纠正窗口超时，仅收到%zu个有效样本",
          ring_localization_y_samples_.size());
      }
      return;
    }
    if (!ring_localization_window_active_) {
      ring_localization_window_active_ = true;
      ring_localization_y_samples_.clear();
      RCLCPP_INFO(
        get_logger(), "Entered ring y correction window at path point %d", one_based_point);
    }
    ring_localization_solid_fraction_ = observation.solid_fraction;
    ring_localization_lateral_m_ = observation.lateral_m;
    if (!observation.valid || ring_track_confidence_ < ring_vision_min_confidence_) {
      return;
    }

    const Pose2d & reference = tracking_path_[progress_index_].pose;
    const double y_observability = std::cos(reference.yaw);
    if (std::abs(y_observability) < ring_localization_min_y_observability_) {
      return;
    }
    const double measured_y =
      reference.y + observation.lateral_m / y_observability;
    const double correction = measured_y - current_pose_.y;
    ring_localization_measured_y_ = measured_y;
    ring_localization_correction_ = correction;
    if (std::abs(correction) > ring_localization_max_correction_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "拒绝环形区Y坐标测量：纠正量%.3f超过上限%.3f米",
        correction, ring_localization_max_correction_);
      return;
    }
    ring_localization_y_samples_.push_back(measured_y);
    if (ring_localization_y_samples_.size() <
      static_cast<std::size_t>(ring_localization_sample_count_))
    {
      return;
    }

    std::vector<double> sorted = ring_localization_y_samples_;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2U];
    std::vector<double> deviations;
    deviations.reserve(sorted.size());
    for (const double value : sorted) {
      deviations.push_back(std::abs(value - median));
    }
    std::sort(deviations.begin(), deviations.end());
    const double median_deviation = deviations[deviations.size() / 2U];
    if (median_deviation > ring_localization_max_sample_spread_) {
      ring_localization_y_samples_.erase(ring_localization_y_samples_.begin());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "环形区Y坐标样本不一致：中位绝对偏差为%.3f米",
        median_deviation);
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped measurement;
    measurement.header.stamp = now();
    measurement.header.frame_id = path_frame_;
    measurement.pose.pose.position.x = current_pose_.x;
    measurement.pose.pose.position.y = median;
    measurement.pose.pose.orientation.w = 1.0;
    measurement.pose.covariance.fill(1e6);
    measurement.pose.covariance[7] =
      ring_localization_measurement_std_ * ring_localization_measurement_std_;
    ring_localization_pub_->publish(measurement);
    ring_localization_applied_ = true;
    ring_localization_attempted_ = true;
    ring_localization_window_active_ = false;
    ring_localization_measured_y_ = median;
    ring_localization_correction_ = median - current_pose_.y;
    RCLCPP_WARN(
      get_logger(),
      "已发布环形区Y坐标纠正：视觉Y=%.3f LIO_Y=%.3f 差值=%.3f "
      "横向偏差=%.3f 有效度=%.2f 航向误差=%.3f 来源=%s",
      median, current_pose_.y, ring_localization_correction_, observation.lateral_m,
      observation.solid_fraction, observation.heading_error,
      ring_localization_depth_valid_ ? "rgbd" : "monocular");
  }

  void ring_binary_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_ring_image_received_ = now();
    ring_track_confidence_ = 0.0;
    update_ring_localization(RingMetricObservation{});
    const auto publish_observation = [this, &msg]() {
        geometry_msgs::msg::Vector3Stamped observation;
        observation.header = msg->header;
        observation.vector.x = ring_track_steering_error_;
        observation.vector.y = ring_track_lateral_offset_;
        observation.vector.z = ring_track_confidence_;
        ring_vision_pub_->publish(observation);
      };
    if (msg->encoding != "mono8" || msg->width < 16U || msg->height < 16U ||
      msg->step < msg->width ||
      msg->data.size() < static_cast<std::size_t>(msg->step) * msg->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "环形视觉需要有效的mono8图像，实际编码=%s 尺寸=%ux%u 行步长=%u",
        msg->encoding.c_str(), msg->width, msg->height, msg->step);
      publish_observation();
      return;
    }

    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    const int start_y = height - 3;
    const int end_y = std::max(1, start_y - ring_image_scan_rows_);
    const int maximum_run_width = std::max(
      ring_image_min_run_width_ + 1,
      static_cast<int>(ring_image_max_run_width_ratio_ * static_cast<double>(width)));
    double preferred_center = ring_last_center_x_ >= 0.0 ?
      ring_last_center_x_ : 0.5 * static_cast<double>(width);
    double expected_width = ring_last_run_width_ > 0.0 ?
      ring_last_run_width_ : 0.60 * static_cast<double>(width);
    std::vector<RingRowRun> centers;
    centers.reserve(static_cast<std::size_t>(ring_image_scan_rows_ / ring_image_row_step_ + 1));

    for (int y = start_y; y >= end_y; y -= ring_image_row_step_) {
      const std::uint8_t * row =
        msg->data.data() + static_cast<std::size_t>(y) * msg->step;
      bool found = false;
      RingRowRun best;
      double best_score = std::numeric_limits<double>::infinity();
      int x = 1;
      while (x < width - 1) {
        while (x < width - 1 && row[x] <= ring_image_threshold_) {
          ++x;
        }
        if (x >= width - 1) {
          break;
        }
        const int run_begin = x;
        while (x < width - 1 && row[x] > ring_image_threshold_) {
          ++x;
        }
        const int run_end = x - 1;
        const int run_width = run_end - run_begin + 1;
        if (run_width < ring_image_min_run_width_ || run_width > maximum_run_width) {
          continue;
        }
        const double center = 0.5 * static_cast<double>(run_begin + run_end);
        const double score = std::abs(center - preferred_center) +
          0.12 * std::abs(static_cast<double>(run_width) - expected_width);
        if (score < best_score) {
          best_score = score;
          best = {
            center, static_cast<double>(y), static_cast<double>(run_width),
            run_begin, run_end};
          found = true;
        }
      }
      if (!found) {
        continue;
      }
      const double continuity_limit = std::max(70.0, 0.45 * expected_width);
      if (!centers.empty() && std::abs(best.x - preferred_center) > continuity_limit) {
        continue;
      }
      centers.push_back(best);
      preferred_center = best.x;
      expected_width = 0.85 * expected_width + 0.15 * best.width;
    }

    if (centers.size() < static_cast<std::size_t>(ring_image_min_valid_rows_)) {
      publish_observation();
      return;
    }

    const RingRowRun & near_center = centers.front();
    const RingRowRun * lookahead_center = &centers.back();
    double accumulated = 0.0;
    for (std::size_t i = 1U; i < centers.size(); ++i) {
      accumulated += std::hypot(
        centers[i].x - centers[i - 1U].x,
        centers[i].y - centers[i - 1U].y);
      if (accumulated >= ring_image_lookahead_px_) {
        lookahead_center = &centers[i];
        break;
      }
    }

    const double image_center = 0.5 * static_cast<double>(width);
    const double dx = lookahead_center->x - image_center;
    const double forward = std::max(1.0, near_center.y - lookahead_center->y);
    ring_track_steering_error_ = -std::atan2(dx, forward);
    ring_track_lateral_offset_ = clamp(
      (near_center.x - image_center) / std::max(1.0, image_center), -1.0, 1.0);
    const double expected_rows = std::max(
      1.0, static_cast<double>(ring_image_scan_rows_) /
      static_cast<double>(ring_image_row_step_));
    const double row_confidence = clamp(
      static_cast<double>(centers.size()) / std::max(
        static_cast<double>(ring_image_min_valid_rows_) * 2.0, 0.65 * expected_rows),
      0.0, 1.0);
    const double lookahead_confidence = clamp(
      accumulated / ring_image_lookahead_px_, 0.0, 1.0);
    ring_track_confidence_ = row_confidence * (0.5 + 0.5 * lookahead_confidence);
    ring_last_center_x_ = near_center.x;
    ring_last_run_width_ = near_center.width;
    RingMetricObservation localization_observation;
    if (ring_localization_use_depth_) {
      localization_observation = estimate_ring_metric_depth(*msg, centers);
    }
    if (!localization_observation.valid &&
      (!ring_localization_use_depth_ || !ring_localization_require_depth_))
    {
      localization_observation = estimate_ring_metric(*msg, centers);
    }
    update_ring_localization(localization_observation);
    publish_observation();
  }

  bool in_ring_path_section() const
  {
    if (!enable_ring_vision_ || tracking_path_.empty()) {
      return false;
    }
    const int one_based_point = static_cast<int>(progress_index_) + 1;
    return one_based_point >= ring_start_path_point_ &&
           one_based_point <= ring_end_path_point_;
  }

  bool ring_obstacle_filter_active() const
  {
    if (!enable_ring_obstacle_path_filter_ || tracking_path_.empty()) {
      return false;
    }
    const int one_based_point = static_cast<int>(progress_index_) + 1;
    return one_based_point >= ring_start_path_point_ &&
           one_based_point <= ring_end_path_point_;
  }

  bool ring_obstacle_point_relevant(const Eigen::Vector2d & point) const
  {
    if (!ring_obstacle_filter_active()) {
      return true;
    }

    // A ring-exterior return is not an obstacle unless its center is close
    // enough to the recorded lane to enter the vehicle corridor. This keeps
    // outer cones and wall returns from steering the car off the ring.
    const Eigen::Vector2d world_point = transform_point(current_pose_, point);
    const Pose2d point_pose{world_point.x(), world_point.y(), current_pose_.yaw};
    const PathMetric metric = nearest_path_metric(point_pose);
    return std::isfinite(metric.distance) &&
           metric.distance <= ring_obstacle_path_half_width_;
  }

  bool ring_track_is_fresh(const rclcpp::Time & current_time) const
  {
    return in_ring_path_section() &&
           ring_track_confidence_ >= ring_vision_min_confidence_ &&
           (current_time - last_ring_image_received_).seconds() >= 0.0 &&
           (current_time - last_ring_image_received_).seconds() <= ring_vision_timeout_;
  }

  std::size_t direction_segment_end(std::size_t start) const
  {
    const int direction = tracking_path_[start].direction;
    std::size_t end = start;
    while (end + 1U < tracking_path_.size() &&
      tracking_path_[end + 1U].direction == direction)
    {
      ++end;
    }
    return end;
  }

  std::size_t update_progress_index(std::size_t segment_end)
  {
    const std::size_t begin = progress_index_;
    const double maximum_arc =
      tracking_path_[progress_index_].arc_length + progress_search_window_;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    std::size_t best_index = progress_index_;
    for (std::size_t i = begin; i <= segment_end; ++i) {
      if (tracking_path_[i].arc_length > maximum_arc) {
        break;
      }
      const double dx = tracking_path_[i].pose.x - current_pose_.x;
      const double dy = tracking_path_[i].pose.y - current_pose_.y;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_index = i;
      }
    }
    progress_index_ = std::max(progress_index_, best_index);
    return progress_index_;
  }

  PathMetric nearest_path_metric(const Pose2d & pose) const
  {
    PathMetric metric;
    const std::size_t begin = progress_index_;
    const std::size_t segment_end = direction_segment_end(progress_index_);
    const double maximum_arc = tracking_path_[progress_index_].arc_length + local_path_window_;
    double best_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = begin; i <= segment_end; ++i) {
      if (tracking_path_[i].arc_length > maximum_arc) {
        break;
      }
      const double distance = std::hypot(
        tracking_path_[i].pose.x - pose.x, tracking_path_[i].pose.y - pose.y);
      const double heading_error = normalize_angle(tracking_path_[i].pose.yaw - pose.yaw);
      const double score = distance * distance + 0.08 * heading_error * heading_error;
      if (score < best_score) {
        best_score = score;
        metric.distance = distance;
        metric.heading_error = heading_error;
        metric.arc_length = tracking_path_[i].arc_length;
      }
    }
    return metric;
  }

  std::size_t find_lookahead_index(double lookahead, std::size_t segment_end) const
  {
    const double target_arc = tracking_path_[progress_index_].arc_length + lookahead;
    auto iterator = std::lower_bound(
      tracking_path_.begin() + static_cast<std::ptrdiff_t>(progress_index_),
      tracking_path_.begin() + static_cast<std::ptrdiff_t>(segment_end + 1U), target_arc,
      [](const PathPoint & point, double arc) {return point.arc_length < arc;});
    if (iterator == tracking_path_.begin() + static_cast<std::ptrdiff_t>(segment_end + 1U)) {
      return segment_end;
    }
    return static_cast<std::size_t>(std::distance(tracking_path_.begin(), iterator));
  }

  LidarObstacle evaluate_lidar_obstacle(
    int direction, double trigger_distance, double path_curvature) const
  {
    LidarObstacle obstacle;
    double left_clearance = obstacle_max_trigger_distance_ + robot_half_length_;
    double right_clearance = left_clearance;
    const double corridor_half_width = robot_half_width_ + obstacle_corridor_margin_;

    for (const auto & point : laser_points_) {
      if (!ring_obstacle_point_relevant(point)) {
        ++obstacle.ring_filtered_points;
        continue;
      }
      const double longitudinal = direction > 0 ? point.x() : -point.x();
      const double clearance = longitudinal - robot_half_length_;
      if (clearance <= 0.0 || clearance > obstacle_max_trigger_distance_) {
        continue;
      }
      const double distance = point.norm();
      if (point.y() >= 0.0) {
        left_clearance = std::min(left_clearance, distance);
      } else {
        right_clearance = std::min(right_clearance, distance);
      }
      if (std::abs(point.y()) <= corridor_half_width && clearance <= trigger_distance) {
        obstacle.detected = true;
        if (clearance < obstacle.clearance) {
          obstacle.clearance = clearance;
          obstacle.point_x = point.x();
          obstacle.point_y = point.y();
          obstacle.detected_on_arc = false;
        }
      }
    }

    // The straight corridor above handles immediate threats. This second check
    // follows the current pursuit arc so obstacles on the inside of a bend are
    // detected before they enter the robot's longitudinal center corridor.
    const double inflated_half_length =
      robot_half_length_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double inflated_half_width =
      robot_half_width_ + obstacle_safety_margin_ + scan_distortion_margin_;
    for (double travel = 0.08; travel <= trigger_distance; travel += 0.08) {
      const double signed_distance = static_cast<double>(direction) * travel;
      Pose2d predicted;
      if (std::abs(path_curvature) < 1e-5) {
        predicted.x = signed_distance;
      } else {
        predicted.x = std::sin(path_curvature * signed_distance) / path_curvature;
        predicted.y = (1.0 - std::cos(path_curvature * signed_distance)) / path_curvature;
        predicted.yaw = path_curvature * signed_distance;
      }
      const double predicted_c = std::cos(predicted.yaw);
      const double predicted_s = std::sin(predicted.yaw);
      for (const auto & point : laser_points_) {
        if (!ring_obstacle_point_relevant(point)) {
          continue;
        }
        const double dx = point.x() - predicted.x;
        const double dy = point.y() - predicted.y;
        const double robot_x = predicted_c * dx + predicted_s * dy;
        const double robot_y = -predicted_s * dx + predicted_c * dy;
        const double outside_x = std::max(0.0, std::abs(robot_x) - inflated_half_length);
        const double outside_y = std::max(0.0, std::abs(robot_y) - inflated_half_width);
        if (outside_x == 0.0 && outside_y == 0.0) {
          obstacle.detected = true;
          if (travel < obstacle.clearance) {
            obstacle.clearance = travel;
            obstacle.point_x = point.x();
            obstacle.point_y = point.y();
            obstacle.detected_on_arc = true;
          }
        }
      }
    }
    obstacle.preferred_side = left_clearance >= right_clearance ? 1 : -1;
    return obstacle;
  }

  LocalPlan evaluate_local_candidate(
    double speed, double curvature, int preferred_side, double visual_yaw_rate,
    double corridor_limit, bool allow_forbidden_area, CandidateRejections & rejections) const
  {
    LocalPlan plan;
    const double yaw_rate = speed * curvature;
    if (std::abs(yaw_rate) > max_yaw_rate_ + 1e-6) {
      ++rejections.yaw_rate;
      return plan;
    }

    Pose2d local_pose;
    double accumulated_cost = 0.0;
    double max_path_deviation = 0.0;
    double min_clearance = std::numeric_limits<double>::infinity();
    bool enters_forbidden_area = false;
    PathMetric final_metric;
    const double horizon_distance = std::max(local_plan_step_distance_, local_plan_horizon_);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(horizon_distance / local_plan_step_distance_)));
    std::vector<Pose2d> trajectory;
    trajectory.reserve(static_cast<std::size_t>(steps));
    const double footprint_length =
      robot_half_length_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double footprint_width =
      robot_half_width_ + obstacle_safety_margin_ + scan_distortion_margin_;

    for (int step = 0; step < steps; ++step) {
      const double travelled = static_cast<double>(step) * local_plan_step_distance_;
      const double step_distance = std::min(
        local_plan_step_distance_, horizon_distance - travelled);
      const double horizon_ratio =
        (travelled + 0.5 * step_distance) / horizon_distance;
      double rollout_curvature = curvature;
      if (horizon_ratio >= local_plan_turn_fraction_ &&
        horizon_ratio < 1.0 - local_plan_turn_fraction_)
      {
        rollout_curvature = 0.0;
      } else if (horizon_ratio >= 1.0 - local_plan_turn_fraction_) {
        rollout_curvature = -curvature;
      }

      const double signed_step = std::copysign(step_distance, speed);
      if (std::abs(rollout_curvature) < 1e-6) {
        local_pose.x += signed_step * std::cos(local_pose.yaw);
        local_pose.y += signed_step * std::sin(local_pose.yaw);
      } else {
        const double next_yaw = local_pose.yaw + rollout_curvature * signed_step;
        local_pose.x +=
          (std::sin(next_yaw) - std::sin(local_pose.yaw)) / rollout_curvature;
        local_pose.y +=
          (std::cos(local_pose.yaw) - std::cos(next_yaw)) / rollout_curvature;
        local_pose.yaw = normalize_angle(next_yaw);
      }

      const Pose2d global_pose = compose(current_pose_, local_pose);
      trajectory.push_back(global_pose);
      const PathMetric path_metric = nearest_path_metric(global_pose);
      if (!std::isfinite(path_metric.distance)) {
        ++rejections.path_metric;
        return plan;
      }
      final_metric = path_metric;
      max_path_deviation = std::max(max_path_deviation, path_metric.distance);
      if (path_metric.distance > corridor_limit) {
        ++rejections.path_corridor;
        return plan;
      }

      const double forbidden_fraction = forbidden_footprint_fraction(global_pose);
      if (forbidden_fraction > 0.0 && !allow_forbidden_area) {
        ++rejections.keepout;
        return plan;
      }
      enters_forbidden_area = enters_forbidden_area || forbidden_fraction > 0.0;
      const double black_excursion = use_keepout_mask_ ? 0.0 :
        std::max(0.0, path_metric.distance - preferred_path_half_width_);
      accumulated_cost += step_distance * (
        weight_path_distance_ * path_metric.distance * path_metric.distance +
        weight_path_heading_ * path_metric.heading_error * path_metric.heading_error +
        weight_black_area_ *
        (black_excursion * black_excursion + forbidden_fraction * forbidden_fraction));

      const double pose_c = std::cos(local_pose.yaw);
      const double pose_s = std::sin(local_pose.yaw);
      for (const auto & obstacle : laser_points_) {
        if (!ring_obstacle_point_relevant(obstacle)) {
          continue;
        }
        const double dx = obstacle.x() - local_pose.x;
        const double dy = obstacle.y() - local_pose.y;
        const double robot_x = pose_c * dx + pose_s * dy;
        const double robot_y = -pose_s * dx + pose_c * dy;
        const double outside_x = std::max(0.0, std::abs(robot_x) - footprint_length);
        const double outside_y = std::max(0.0, std::abs(robot_y) - footprint_width);
        if (outside_x == 0.0 && outside_y == 0.0) {
          ++rejections.collision;
          return plan;
        }
        min_clearance = std::min(min_clearance, std::hypot(outside_x, outside_y));
      }
      if (std::isfinite(min_clearance)) {
        accumulated_cost += step_distance * weight_obstacle_clearance_ /
          std::max(0.03, min_clearance);
      }
    }

    const double current_arc = tracking_path_[progress_index_].arc_length;
    const double progress = std::max(0.0, final_metric.arc_length - current_arc);
    const double wrong_side = std::max(0.0, -static_cast<double>(preferred_side) * local_pose.y);
    accumulated_cost += weight_path_return_ * final_metric.distance * final_metric.distance;
    accumulated_cost += weight_preferred_side_ * wrong_side * wrong_side;
    accumulated_cost += weight_steering_change_ *
      (yaw_rate - last_commanded_yaw_rate_) * (yaw_rate - last_commanded_yaw_rate_);
    accumulated_cost += weight_visual_command_ *
      (yaw_rate - visual_yaw_rate) * (yaw_rate - visual_yaw_rate);
    accumulated_cost -= weight_progress_ * progress;
    accumulated_cost -= weight_speed_ * std::abs(speed);

    plan.valid = true;
    plan.speed = speed;
    plan.yaw_rate = yaw_rate;
    plan.cost = accumulated_cost;
    plan.max_path_deviation = max_path_deviation;
    plan.min_obstacle_clearance = min_clearance;
    plan.enters_black_area = enters_forbidden_area ||
      (!use_keepout_mask_ && max_path_deviation > preferred_path_half_width_);
    plan.trajectory = std::move(trajectory);
    return plan;
  }

  double white_tracking_cost(double curvature, int direction) const
  {
    Pose2d local_pose;
    double cost = 0.0;
    const int steps = std::max(
      1, static_cast<int>(
        std::ceil(nominal_mask_check_distance_ / local_plan_step_distance_)));
    for (int step = 0; step < steps; ++step) {
      const double travelled = static_cast<double>(step) * local_plan_step_distance_;
      const double step_distance = std::min(
        local_plan_step_distance_, nominal_mask_check_distance_ - travelled);
      const double signed_step = static_cast<double>(direction) * step_distance;
      if (std::abs(curvature) < 1e-6) {
        local_pose.x += signed_step * std::cos(local_pose.yaw);
        local_pose.y += signed_step * std::sin(local_pose.yaw);
      } else {
        const double next_yaw = local_pose.yaw + curvature * signed_step;
        local_pose.x +=
          (std::sin(next_yaw) - std::sin(local_pose.yaw)) / curvature;
        local_pose.y +=
          (std::cos(local_pose.yaw) - std::cos(next_yaw)) / curvature;
        local_pose.yaw = normalize_angle(next_yaw);
      }

      const Pose2d global_pose = compose(current_pose_, local_pose);
      if (forbidden_footprint_fraction(global_pose) > 0.0) {
        return std::numeric_limits<double>::infinity();
      }
      const PathMetric metric = nearest_path_metric(global_pose);
      if (!std::isfinite(metric.distance)) {
        return std::numeric_limits<double>::infinity();
      }
      cost += step_distance * (
        metric.distance * metric.distance + 0.08 * metric.heading_error * metric.heading_error);
    }
    return cost;
  }

  bool select_white_tracking_curvature(
    double pursuit_curvature, int direction, double & selected_curvature) const
  {
    selected_curvature = clamp(pursuit_curvature, -max_curvature_, max_curvature_);
    if (!use_keepout_mask_ || std::isfinite(white_tracking_cost(selected_curvature, direction))) {
      return true;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < local_plan_curvature_samples_; ++i) {
      const double ratio = local_plan_curvature_samples_ == 1 ? 0.0 :
        static_cast<double>(i) / static_cast<double>(local_plan_curvature_samples_ - 1);
      const double candidate = -max_curvature_ + 2.0 * max_curvature_ * ratio;
      const double tracking_cost = white_tracking_cost(candidate, direction);
      if (!std::isfinite(tracking_cost)) {
        continue;
      }
      const double steering_error = candidate - pursuit_curvature;
      const double cost = tracking_cost + 0.15 * steering_error * steering_error;
      if (cost < best_cost) {
        best_cost = cost;
        selected_curvature = candidate;
      }
    }
    return std::isfinite(best_cost);
  }

  LocalPlan evaluate_ring_recovery_candidate(
    double speed, double curvature, double reference_curvature,
    bool vision_fresh) const
  {
    LocalPlan plan;
    const double yaw_rate = speed * curvature;
    if (std::abs(yaw_rate) > max_yaw_rate_ + 1e-6) {
      return plan;
    }

    const double initial_forbidden = forbidden_footprint_fraction(current_pose_);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(ring_recovery_horizon_ / local_plan_step_distance_)));
    const double footprint_length =
      robot_half_length_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double footprint_width =
      robot_half_width_ + obstacle_safety_margin_ + scan_distortion_margin_;

    Pose2d local_pose;
    PathMetric final_metric;
    double final_forbidden = initial_forbidden;
    double maximum_forbidden = initial_forbidden;
    double maximum_path_deviation = 0.0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double cost = 0.0;
    std::vector<Pose2d> trajectory;
    trajectory.reserve(static_cast<std::size_t>(steps));

    for (int step = 0; step < steps; ++step) {
      const double travelled = static_cast<double>(step) * local_plan_step_distance_;
      const double step_distance = std::min(
        local_plan_step_distance_, ring_recovery_horizon_ - travelled);
      const double signed_step = std::copysign(step_distance, speed);
      if (std::abs(curvature) < 1e-6) {
        local_pose.x += signed_step * std::cos(local_pose.yaw);
        local_pose.y += signed_step * std::sin(local_pose.yaw);
      } else {
        const double next_yaw = local_pose.yaw + curvature * signed_step;
        local_pose.x +=
          (std::sin(next_yaw) - std::sin(local_pose.yaw)) / curvature;
        local_pose.y +=
          (std::cos(local_pose.yaw) - std::cos(next_yaw)) / curvature;
        local_pose.yaw = normalize_angle(next_yaw);
      }

      const Pose2d global_pose = compose(current_pose_, local_pose);
      trajectory.push_back(global_pose);
      const std::array<Eigen::Vector2d, 4> field_corners{{
        Eigen::Vector2d(robot_half_length_, robot_half_width_),
        Eigen::Vector2d(robot_half_length_, -robot_half_width_),
        Eigen::Vector2d(-robot_half_length_, robot_half_width_),
        Eigen::Vector2d(-robot_half_length_, -robot_half_width_)}};
      for (const auto & corner : field_corners) {
        const Eigen::Vector2d world_corner = transform_point(global_pose, corner);
        if (world_corner.x() < field_min_x_ + ring_field_boundary_margin_ ||
          world_corner.x() > field_max_x_ - ring_field_boundary_margin_ ||
          world_corner.y() < field_min_y_ + ring_field_boundary_margin_ ||
          world_corner.y() > field_max_y_ - ring_field_boundary_margin_)
        {
          return plan;
        }
      }
      const PathMetric metric = nearest_path_metric(global_pose);
      if (!std::isfinite(metric.distance) ||
        metric.distance > ring_recovery_path_half_width_)
      {
        return plan;
      }
      final_metric = metric;
      maximum_path_deviation = std::max(maximum_path_deviation, metric.distance);

      final_forbidden = forbidden_footprint_fraction(global_pose);
      maximum_forbidden = std::max(maximum_forbidden, final_forbidden);
      cost += step_distance * (
        weight_path_distance_ * metric.distance * metric.distance +
        weight_path_heading_ * metric.heading_error * metric.heading_error +
        weight_black_area_ * final_forbidden * final_forbidden);

      const double pose_c = std::cos(local_pose.yaw);
      const double pose_s = std::sin(local_pose.yaw);
      for (const auto & obstacle : laser_points_) {
        const double dx = obstacle.x() - local_pose.x;
        const double dy = obstacle.y() - local_pose.y;
        const double robot_x = pose_c * dx + pose_s * dy;
        const double robot_y = -pose_s * dx + pose_c * dy;
        const double outside_x = std::max(0.0, std::abs(robot_x) - footprint_length);
        const double outside_y = std::max(0.0, std::abs(robot_y) - footprint_width);
        if (outside_x == 0.0 && outside_y == 0.0) {
          return plan;
        }
        minimum_clearance = std::min(minimum_clearance, std::hypot(outside_x, outside_y));
      }
    }

    // Without a trustworthy camera observation, entering black from white is
    // forbidden, and a vehicle already in black must demonstrably move out.
    if (!vision_fresh) {
      if (initial_forbidden <= 0.0 && maximum_forbidden > 0.0) {
        return plan;
      }
      if (initial_forbidden > 0.0 &&
        final_forbidden > initial_forbidden - ring_recovery_min_mask_improvement_)
      {
        return plan;
      }
    } else if (initial_forbidden > 0.0 && final_forbidden > initial_forbidden + 0.12) {
      // Vision may override a drifted mask, but it may not drive substantially
      // deeper into the forbidden region according to the available estimate.
      return plan;
    }

    const double curvature_error = curvature - reference_curvature;
    const double progress = std::max(
      0.0, final_metric.arc_length - tracking_path_[progress_index_].arc_length);
    cost += 120.0 * final_forbidden * final_forbidden;
    cost += 3.0 * curvature_error * curvature_error;
    cost += weight_steering_change_ *
      (yaw_rate - last_commanded_yaw_rate_) * (yaw_rate - last_commanded_yaw_rate_);
    cost -= weight_progress_ * progress;
    if (std::isfinite(minimum_clearance)) {
      cost += weight_obstacle_clearance_ / std::max(0.03, minimum_clearance);
    }

    plan.valid = true;
    plan.speed = speed;
    plan.yaw_rate = yaw_rate;
    plan.cost = cost;
    plan.max_path_deviation = maximum_path_deviation;
    plan.min_obstacle_clearance = minimum_clearance;
    plan.enters_black_area = maximum_forbidden > 0.0;
    plan.trajectory = std::move(trajectory);
    return plan;
  }

  LocalPlan plan_ring_recovery(
    int direction, double pursuit_curvature, bool vision_fresh) const
  {
    const double maximum_speed = std::min(
      ring_recovery_speed_, direction > 0 ? max_forward_speed_ : max_reverse_speed_);
    const double speed = static_cast<double>(direction) * maximum_speed;
    const double reference_curvature = pursuit_curvature;

    std::vector<double> curvatures;
    curvatures.reserve(static_cast<std::size_t>(local_plan_curvature_samples_) + 2U);
    for (int i = 0; i < local_plan_curvature_samples_; ++i) {
      const double ratio = local_plan_curvature_samples_ == 1 ? 0.0 :
        static_cast<double>(i) / static_cast<double>(local_plan_curvature_samples_ - 1);
      curvatures.push_back(-max_curvature_ + 2.0 * max_curvature_ * ratio);
    }
    curvatures.push_back(clamp(pursuit_curvature, -max_curvature_, max_curvature_));
    LocalPlan best;
    for (const double candidate_curvature : curvatures) {
      const LocalPlan candidate = evaluate_ring_recovery_candidate(
        speed, candidate_curvature, reference_curvature, vision_fresh);
      if (candidate.valid && candidate.cost < best.cost) {
        best = candidate;
      }
    }
    return best;
  }

  LocalPlan plan_local_avoidance(
    int direction, double pursuit_curvature, int preferred_side,
    double visual_yaw_rate, CandidateRejections & rejections) const
  {
    const double maximum_speed = std::min(obstacle_avoid_speed_, path_speed_limit(direction));
    std::vector<double> curvatures;
    curvatures.reserve(static_cast<std::size_t>(local_plan_curvature_samples_) + 2U);
    for (int i = 0; i < local_plan_curvature_samples_; ++i) {
      const double ratio = local_plan_curvature_samples_ == 1 ? 0.0 :
        static_cast<double>(i) / static_cast<double>(local_plan_curvature_samples_ - 1);
      curvatures.push_back(-max_curvature_ + 2.0 * max_curvature_ * ratio);
    }
    curvatures.push_back(clamp(pursuit_curvature, -max_curvature_, max_curvature_));
    if (maximum_speed > 0.05) {
      curvatures.push_back(clamp(
        visual_yaw_rate / (static_cast<double>(direction) * maximum_speed),
        -max_curvature_, max_curvature_));
    }

    const bool prefer_side_rear_reverse_arc =
      reverse_side_rear_active_ && direction < 0;
    auto search_with_corridor = [&] (
      double corridor_limit, bool allow_forbidden_area, bool enforce_preferred_side)
      {
        LocalPlan best;
        for (int speed_index = 0; speed_index < local_plan_speed_samples_; ++speed_index) {
          const double ratio = local_plan_speed_samples_ == 1 ? 1.0 :
            1.0 - 0.35 * static_cast<double>(speed_index) /
            static_cast<double>(local_plan_speed_samples_ - 1);
          const double speed = static_cast<double>(direction) * maximum_speed * ratio;
          for (const double curvature : curvatures) {
            // Second (blue) stage: for a right-rear obstacle, reverse left or
            // straight. Do not choose the mirrored right-rear arc merely
            // because another soft cost is slightly lower.
            if (enforce_preferred_side &&
              curvature * static_cast<double>(preferred_side) < -1e-6)
            {
              continue;
            }
            const LocalPlan candidate = evaluate_local_candidate(
              speed, curvature, preferred_side, visual_yaw_rate, corridor_limit,
              allow_forbidden_area, rejections);
            if (candidate.valid && candidate.cost < best.cost) {
              best = candidate;
            }
          }
        }
        return best;
      };

    LocalPlan plan;
    if (use_keepout_mask_) {
      // This function is only entered during active obstacle avoidance. Black is
      // therefore a high-cost soft constraint, matching the competition rule
      // that it may be crossed briefly to pass a cone.
      if (prefer_side_rear_reverse_arc) {
        plan = search_with_corridor(emergency_path_half_width_, true, true);
      }
      if (!plan.valid) {
        plan = search_with_corridor(emergency_path_half_width_, true, false);
      }
    } else {
      // Fallback for runs without a calibrated field mask.
      if (prefer_side_rear_reverse_arc) {
        plan = search_with_corridor(preferred_path_half_width_, false, true);
      }
      if (!plan.valid) {
        plan = search_with_corridor(preferred_path_half_width_, false, false);
      }
      if (!plan.valid) {
        if (prefer_side_rear_reverse_arc) {
          plan = search_with_corridor(emergency_path_half_width_, true, true);
        }
        if (!plan.valid) {
          plan = search_with_corridor(emergency_path_half_width_, true, false);
        }
      }
    }
    return plan;
  }

  LocalPlan evaluate_reverse_escape_candidate(
    double curvature, int path_direction, double distance,
    bool ring_section, bool vision_fresh, EscapeRejections & rejections) const
  {
    LocalPlan plan;
    const double speed = -static_cast<double>(path_direction) * reverse_escape_speed_;
    const double yaw_rate = speed * curvature;
    if (std::abs(yaw_rate) > max_yaw_rate_ + 1e-6) {
      ++rejections.yaw_rate;
      return plan;
    }

    const double footprint_length =
      robot_half_length_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double footprint_width =
      robot_half_width_ + obstacle_safety_margin_ + scan_distortion_margin_;
    const double initial_forbidden = forbidden_footprint_fraction(current_pose_);
    double final_forbidden = initial_forbidden;
    double maximum_forbidden = initial_forbidden;
    double maximum_path_deviation = 0.0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double cost = 0.0;
    bool final_inflated_overlap = false;
    Pose2d local_pose;
    PathMetric final_metric;

    // A cone can already overlap the inflated footprint when recovery starts.
    // Such an existing overlap may unwind, but no candidate may deepen it or
    // create a new collision. This avoids deadlocking on the safety margin
    // without allowing the physical manoeuvre to move into an obstacle.
    std::vector<double> initial_penetrations;
    initial_penetrations.reserve(laser_points_.size());
    for (const auto & obstacle : laser_points_) {
      if (!ring_obstacle_point_relevant(obstacle)) {
        initial_penetrations.push_back(-std::numeric_limits<double>::infinity());
        continue;
      }
      const double penetration_x = footprint_length - std::abs(obstacle.x());
      const double penetration_y = footprint_width - std::abs(obstacle.y());
      initial_penetrations.push_back(std::min(penetration_x, penetration_y));
    }

    const double horizon = std::max(local_plan_step_distance_, distance);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(horizon / local_plan_step_distance_)));
    std::vector<Pose2d> trajectory;
    trajectory.reserve(static_cast<std::size_t>(steps));

    for (int step = 0; step < steps; ++step) {
      const double travelled = static_cast<double>(step) * local_plan_step_distance_;
      const double step_distance = std::min(local_plan_step_distance_, horizon - travelled);
      const double signed_step = -static_cast<double>(path_direction) * step_distance;
      if (std::abs(curvature) < 1e-6) {
        local_pose.x += signed_step * std::cos(local_pose.yaw);
        local_pose.y += signed_step * std::sin(local_pose.yaw);
      } else {
        const double next_yaw = local_pose.yaw + curvature * signed_step;
        local_pose.x +=
          (std::sin(next_yaw) - std::sin(local_pose.yaw)) / curvature;
        local_pose.y +=
          (std::cos(local_pose.yaw) - std::cos(next_yaw)) / curvature;
        local_pose.yaw = normalize_angle(next_yaw);
      }

      const Pose2d global_pose = compose(current_pose_, local_pose);
      trajectory.push_back(global_pose);
      const std::array<Eigen::Vector2d, 4> field_corners{{
        Eigen::Vector2d(robot_half_length_, robot_half_width_),
        Eigen::Vector2d(robot_half_length_, -robot_half_width_),
        Eigen::Vector2d(-robot_half_length_, robot_half_width_),
        Eigen::Vector2d(-robot_half_length_, -robot_half_width_)}};
      for (const auto & corner : field_corners) {
        const Eigen::Vector2d world_corner = transform_point(global_pose, corner);
        if (world_corner.x() <= field_min_x_ + ring_field_boundary_margin_ ||
          world_corner.x() >= field_max_x_ - ring_field_boundary_margin_ ||
          world_corner.y() <= field_min_y_ + ring_field_boundary_margin_ ||
          world_corner.y() >= field_max_y_ - ring_field_boundary_margin_)
        {
          ++rejections.field_boundary;
          return plan;
        }
      }

      const PathMetric metric = nearest_path_metric(global_pose);
      if (!std::isfinite(metric.distance) || metric.distance > reverse_escape_path_half_width_) {
        ++rejections.path_corridor;
        return plan;
      }
      final_metric = metric;
      maximum_path_deviation = std::max(maximum_path_deviation, metric.distance);
      final_forbidden = forbidden_footprint_fraction(global_pose);
      maximum_forbidden = std::max(maximum_forbidden, final_forbidden);
      cost += step_distance * (
        8.0 * metric.distance * metric.distance +
        metric.heading_error * metric.heading_error +
        weight_black_area_ * final_forbidden * final_forbidden);

      const double pose_c = std::cos(local_pose.yaw);
      const double pose_s = std::sin(local_pose.yaw);
      final_inflated_overlap = false;
      for (std::size_t i = 0; i < laser_points_.size(); ++i) {
        if (!ring_obstacle_point_relevant(laser_points_[i])) {
          continue;
        }
        const double dx = laser_points_[i].x() - local_pose.x;
        const double dy = laser_points_[i].y() - local_pose.y;
        const double robot_x = pose_c * dx + pose_s * dy;
        const double robot_y = -pose_s * dx + pose_c * dy;
        const double penetration_x = footprint_length - std::abs(robot_x);
        const double penetration_y = footprint_width - std::abs(robot_y);
        const double penetration = std::min(penetration_x, penetration_y);
        if (penetration > 0.0) {
          final_inflated_overlap = true;
          if (initial_penetrations[i] <= 0.0 ||
            penetration > initial_penetrations[i] + 0.005)
          {
            ++rejections.collision;
            return plan;
          }
        } else {
          const double outside_x = std::max(0.0, -penetration_x);
          const double outside_y = std::max(0.0, -penetration_y);
          minimum_clearance = std::min(
            minimum_clearance, std::hypot(outside_x, outside_y));
        }
      }
      if (std::isfinite(minimum_clearance)) {
        cost += step_distance * weight_obstacle_clearance_ /
          std::max(0.03, minimum_clearance);
      }
    }

    // The complete manoeuvre must clear any safety-margin overlap that was
    // present at its beginning; otherwise it has not created usable space.
    if (final_inflated_overlap) {
      ++rejections.collision;
      return plan;
    }

    if (!ring_section) {
      if (maximum_forbidden > 0.0) {
        ++rejections.keepout;
        return plan;
      }
    } else if (!vision_fresh) {
      if (initial_forbidden <= 0.0 && maximum_forbidden > 0.0) {
        ++rejections.keepout;
        return plan;
      }
      if (initial_forbidden > 0.0 &&
        final_forbidden > initial_forbidden - ring_recovery_min_mask_improvement_)
      {
        ++rejections.keepout;
        return plan;
      }
    } else {
      // A fresh Aurora observation can soften a drifted ring mask, but the
      // selected endpoint still may not move substantially deeper into black.
      if (final_forbidden > initial_forbidden + 0.12) {
        ++rejections.keepout;
        return plan;
      }
    }

    const double desired_escape_yaw_rate = avoidance_active_ ?
      static_cast<double>(avoidance_side_ * path_direction) *
      std::min(0.60, reverse_escape_speed_ * reverse_escape_max_curvature_) : 0.0;
    cost += 4.0 * final_metric.distance * final_metric.distance;
    cost += 1.5 * (yaw_rate - desired_escape_yaw_rate) *
      (yaw_rate - desired_escape_yaw_rate);
    cost += 1.0 * curvature * curvature;
    cost += 120.0 * final_forbidden * final_forbidden;
    if (std::isfinite(minimum_clearance)) {
      cost += 1.5 * weight_obstacle_clearance_ / std::max(0.03, minimum_clearance);
    }

    plan.valid = true;
    plan.speed = speed;
    plan.yaw_rate = yaw_rate;
    plan.cost = cost;
    plan.max_path_deviation = maximum_path_deviation;
    plan.min_obstacle_clearance = minimum_clearance;
    plan.enters_black_area = maximum_forbidden > 0.0;
    plan.travel_distance = horizon;
    plan.trajectory = std::move(trajectory);
    return plan;
  }

  LocalPlan plan_reverse_escape(
    int path_direction, double distance, bool ring_section, bool vision_fresh,
    EscapeRejections & rejections) const
  {
    // Prefer the requested clearance distance. Near a field wall a shorter
    // 80% or 60% retreat is still useful and safer than rejecting recovery
    // solely because the full endpoint does not fit.
    const std::array<double, 3> distance_ratios{{1.0, 0.8, 0.6}};
    for (const double distance_ratio : distance_ratios) {
      const double candidate_distance = std::max(0.10, distance * distance_ratio);
      auto search_curvatures = [&](bool enforce_green_direction) {
          LocalPlan best;
          for (int i = 0; i < reverse_escape_curvature_samples_; ++i) {
            const double ratio = reverse_escape_curvature_samples_ == 1 ? 0.5 :
              static_cast<double>(i) /
              static_cast<double>(reverse_escape_curvature_samples_ - 1);
            const double curvature =
              -reverse_escape_max_curvature_ + 2.0 * reverse_escape_max_curvature_ * ratio;
            if (enforce_green_direction) {
              // First (green) stage: move the front toward the obstacle side.
              // With a right-rear cone avoidance_side_ is +left for the later
              // reverse arc, therefore the forward curvature must be -right.
              const double forward_side = -static_cast<double>(avoidance_side_);
              if (curvature * forward_side < reverse_side_rear_forward_min_curvature_) {
                continue;
              }
            }
            const LocalPlan candidate = evaluate_reverse_escape_candidate(
              curvature, path_direction, candidate_distance,
              ring_section, vision_fresh, rejections);
            if (candidate.valid && candidate.cost < best.cost) {
              best = candidate;
            }
          }
          return best;
        };

      if (reverse_side_rear_active_ && path_direction < 0) {
        LocalPlan preferred = search_curvatures(true);
        if (preferred.valid) {
          return preferred;
        }
      }
      LocalPlan best = search_curvatures(false);
      if (best.valid) {
        return best;
      }
    }
    return LocalPlan{};
  }

  bool start_reverse_escape(
    const std::string & cause, int path_direction, const rclcpp::Time & current_time)
  {
    if (reverse_side_rear_active_ && path_direction < 0 &&
      reverse_side_rear_forward_done_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "侧后方障碍已经完成一次前向腾挪，不再反复向前试探；等待安全的倒车弧线");
      return false;
    }
    if (!enable_reverse_escape_ || escape_phase_ != EscapePhase::IDLE ||
      reverse_escape_attempts_ >= reverse_escape_max_attempts_)
    {
      return false;
    }

    ++reverse_escape_attempts_;
    escape_phase_ = EscapePhase::BRAKING;
    escape_phase_start_time_ = current_time;
    escape_path_direction_ = path_direction;
    escape_cause_ = cause;
    escape_braking_curvature_ = std::abs(commanded_speed_) > 0.02 ?
      clamp(last_commanded_yaw_rate_ / commanded_speed_, -max_curvature_, max_curvature_) : 0.0;
    reverse_escape_last_arc_ = tracking_path_[progress_index_].arc_length;
    clear_local_plan();
    RCLCPP_WARN(
      get_logger(),
      "触发第%d/%d次脱困，原因=%s：先制动，再执行%s运动",
      reverse_escape_attempts_, reverse_escape_max_attempts_, cause.c_str(),
      path_direction > 0 ? "倒车" : "前进");
    return true;
  }

  void maybe_reset_reverse_escape_budget()
  {
    if (escape_phase_ == EscapePhase::IDLE && reverse_escape_attempts_ > 0 &&
      tracking_path_[progress_index_].arc_length >=
      reverse_escape_last_arc_ + reverse_escape_reset_progress_)
    {
      reverse_escape_attempts_ = 0;
      RCLCPP_INFO(get_logger(), "路径有效前进后，脱困重试计数已重置");
    }
  }

  bool run_reverse_escape(
    double dt, const rclcpp::Time & current_time,
    bool ring_section, bool vision_fresh)
  {
    if (escape_phase_ == EscapePhase::IDLE) {
      return false;
    }

    if (escape_phase_ == EscapePhase::BRAKING) {
      apply_acceleration_limit(0.0, dt);
      const double yaw_rate = commanded_speed_ * escape_braking_curvature_;
      publish_command(commanded_speed_, yaw_rate);
      publish_state("ESCAPE_BRAKING");
      if (std::abs(measured_velocity_) <= reverse_escape_stop_speed_ &&
        std::abs(commanded_speed_) <= reverse_escape_stop_speed_)
      {
        EscapeRejections rejections;
        const double requested_distance =
          reverse_side_rear_active_ && escape_path_direction_ < 0 ?
          reverse_side_rear_forward_distance_ : reverse_escape_distance_;
        escape_selected_plan_ = plan_reverse_escape(
          escape_path_direction_, requested_distance,
          ring_section, vision_fresh, rejections);
        if (!escape_selected_plan_.valid) {
          escape_phase_ = EscapePhase::IDLE;
          reverse_escape_attempts_ = reverse_escape_max_attempts_;
          stop_with_state("ESCAPE_NO_SAFE_TRAJECTORY");
          RCLCPP_WARN(
            get_logger(),
            "脱困终止：没有安全的反向曲率轨迹 "
            "拒绝统计[角速度=%d 边界=%d 路径走廊=%d 禁行区=%d 碰撞=%d]",
            rejections.yaw_rate, rejections.field_boundary, rejections.path_corridor,
            rejections.keepout, rejections.collision);
          return true;
        }
        escape_selected_curvature_ = escape_selected_plan_.yaw_rate /
          escape_selected_plan_.speed;
        escape_selected_distance_ = escape_selected_plan_.travel_distance;
        reverse_escape_start_pose_ = current_pose_;
        escape_phase_ = EscapePhase::REVERSING;
        escape_phase_start_time_ = current_time;
        publish_local_plan(escape_selected_plan_);
        publish_state("ESCAPE_TRAJECTORY_SELECTED");
        RCLCPP_INFO(
          get_logger(),
          "已选脱困轨迹：速度=%.2f 曲率=%.2f 距离=%.2f 安全余量=%.2f",
          escape_selected_plan_.speed, escape_selected_curvature_, escape_selected_distance_,
          escape_selected_plan_.min_obstacle_clearance);
      }
      return true;
    }

    if (escape_phase_ == EscapePhase::REVERSING) {
      const double travelled = std::hypot(
        current_pose_.x - reverse_escape_start_pose_.x,
        current_pose_.y - reverse_escape_start_pose_.y);
      const double remaining = std::max(0.0, escape_selected_distance_ - travelled);
      const double elapsed = (current_time - escape_phase_start_time_).seconds();
      const double anticipated_speed = std::max(
        std::abs(measured_velocity_), std::abs(commanded_speed_));
      const double braking_distance =
        anticipated_speed * reverse_escape_reaction_time_ +
        anticipated_speed * anticipated_speed /
        (2.0 * std::max(0.1, max_deceleration_));

      std::string settling_state;
      if (elapsed >= reverse_escape_max_duration_) {
        settling_state = "ESCAPE_TIMEOUT_BRAKE";
      } else if (remaining <= std::max(0.02, braking_distance)) {
        settling_state = "ESCAPE_DISTANCE_BRAKE";
      } else {
        EscapeRejections rejections;
        const LocalPlan safety_check = evaluate_reverse_escape_candidate(
          escape_selected_curvature_, escape_path_direction_, remaining,
          ring_section, vision_fresh, rejections);
        if (!safety_check.valid) {
          settling_state = "ESCAPE_BLOCKED_BRAKE";
        }
      }

      if (!settling_state.empty()) {
        escape_phase_ = EscapePhase::SETTLING;
        escape_phase_start_time_ = current_time;
        escape_settling_state_ = settling_state;
        apply_acceleration_limit(0.0, dt);
        publish_command(commanded_speed_, commanded_speed_ * escape_selected_curvature_);
        publish_state(escape_settling_state_);
        return true;
      }

      apply_acceleration_limit(escape_selected_plan_.speed, dt);
      publish_command(commanded_speed_, commanded_speed_ * escape_selected_curvature_);
      publish_local_plan(escape_selected_plan_);
      publish_state(
        reverse_side_rear_active_ && escape_path_direction_ < 0 ?
        "REVERSE_SIDE_REAR_FORWARD" :
        (escape_selected_plan_.speed < 0.0 ?
        "ESCAPE_REVERSING" : "ESCAPE_FORWARD_CLEARANCE"));
      return true;
    }

    apply_acceleration_limit(0.0, dt);
    publish_command(commanded_speed_, commanded_speed_ * escape_selected_curvature_);
    publish_state(escape_settling_state_);
    const double settled_for = (current_time - escape_phase_start_time_).seconds();
    if (std::abs(measured_velocity_) <= reverse_escape_stop_speed_ &&
      std::abs(commanded_speed_) <= reverse_escape_stop_speed_ &&
      settled_for >= reverse_escape_settle_time_)
    {
      if (reverse_side_rear_active_ && escape_path_direction_ < 0 &&
        escape_selected_plan_.speed > 0.0)
      {
        reverse_side_rear_forward_done_ = true;
        RCLCPP_INFO(
          get_logger(), "侧后方避障第一阶段完成，开始重新规划%s后方倒车弧线",
          avoidance_side_ > 0 ? "左" : "右");
      }
      escape_phase_ = EscapePhase::IDLE;
      commanded_speed_ = 0.0;
      clear_local_plan();
      publish_command(0.0, 0.0);
      publish_state("ESCAPE_REPLAN");
    }
    return true;
  }

  bool visual_parking_is_armed(double distance_to_goal, int direction) const
  {
    if (!enable_visual_parking_ || direction <= 0 ||
      distance_to_goal > parking_arm_distance_)
    {
      return false;
    }
    if (visual_parking_only_post_qr_) {
      return enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::POST_QR;
    }
    return true;
  }

  bool parking_detection_is_fresh(const rclcpp::Time & current_time) const
  {
    return parking_detection_valid_ &&
           (current_time - last_parking_detection_).seconds() <= parking_detection_timeout_;
  }

  void run_visual_parking(const rclcpp::Time & current_time)
  {
    if (parking_phase_ == ParkingPhase::STOPPED) {
      stop_with_state("PARKING_COMPLETE");
      return;
    }

    if (parking_phase_ == ParkingPhase::FINAL_CRAWL) {
      const double elapsed = (current_time - parking_final_crawl_start_).seconds();
      if (elapsed >= parking_final_crawl_duration_) {
        parking_phase_ = ParkingPhase::STOPPED;
        completed_ = true;
        stop_with_state("PARKING_COMPLETE");
        RCLCPP_INFO(get_logger(), "Visual parking completed; zero velocity is latched");
        return;
      }
      commanded_speed_ = std::min(parking_final_crawl_speed_, max_forward_speed_);
      publish_command(commanded_speed_, 0.0);
      publish_state("PARKING_FINAL_CRAWL");
      return;
    }

    if (!parking_detection_is_fresh(current_time)) {
      parking_phase_ = ParkingPhase::IDLE;
      return;
    }

    parking_phase_ = ParkingPhase::APPROACH;
    if (parking_target_ymax_ >= parking_trigger_y_) {
      parking_phase_ = ParkingPhase::FINAL_CRAWL;
      parking_final_crawl_start_ = current_time;
      commanded_speed_ = std::min(parking_final_crawl_speed_, max_forward_speed_);
      publish_command(commanded_speed_, 0.0);
      publish_state("PARKING_FINAL_CRAWL");
      RCLCPP_INFO(
        get_logger(), "P target reached parking trigger: ymax=%.1f px, final crawl=%.2f s",
        parking_target_ymax_, parking_final_crawl_duration_);
      return;
    }

    const double ratio = clamp(
      (parking_stop_y_ - parking_target_ymax_) /
      (parking_stop_y_ - parking_decel_start_y_), 0.0, 1.0);
    const double approach_speed = parking_target_ymax_ < parking_decel_start_y_ ?
      parking_max_speed_ : parking_max_speed_ * ratio;
    const double error = parking_target_center_x_ - parking_center_x_;
    const double derivative = error - previous_parking_error_;
    previous_parking_error_ = error;
    const double yaw_rate = clamp(
      -(parking_kp_ * error + parking_kd_ * derivative), -max_yaw_rate_, max_yaw_rate_);
    commanded_speed_ = std::min(approach_speed, max_forward_speed_);
    publish_command(commanded_speed_, yaw_rate);
    publish_state("PARKING_VISUAL_APPROACH");
  }

  void control_step()
  {
    const rclcpp::Time current_time = now();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    if (!(dt > 0.0 && dt < 0.5)) {
      dt = 1.0 / std::max(1.0, control_frequency_);
    }

    if (parking_phase_ == ParkingPhase::STOPPED) {
      stop_with_state("PARKING_COMPLETE");
      return;
    }

    if (enable_qr_mission_ && first_line_mode_ == "external" &&
      qr_mission_phase_ == QrMissionPhase::SPRINT)
    {
      // The external visual controller publishes /cmd_vel directly. LIO keeps
      // updating pose, scans and RViz diagnostics but emits no command until QR.
      publish_state("QR_EXTERNAL_MONITOR");
      return;
    }

    if (enable_qr_mission_ && first_line_mode_ == "vision" &&
      qr_mission_phase_ == QrMissionPhase::SPRINT)
    {
      const bool command_is_fresh = have_visual_first_line_cmd_ &&
        (current_time - last_visual_first_line_cmd_).seconds() <=
        visual_first_line_cmd_timeout_;
      if (!command_is_fresh) {
        stop_with_state("QR_VISUAL_WAIT_CMD");
      } else {
        commanded_speed_ = visual_first_line_command_.linear.x;
        publish_command(
          visual_first_line_command_.linear.x,
          visual_first_line_command_.angular.z);
        publish_state("QR_VISUAL_FIRST_LINE");
      }
      return;
    }

    if (!have_odom_ || !path_aligned_ ||
      (current_time - last_odom_received_).seconds() > odom_timeout_ ||
      (current_time - last_odom_stamp_).seconds() > max_odom_message_age_)
    {
      stop_with_state("WAIT_ODOM");
      return;
    }
    if (use_keepout_mask_ && !mask_alignment_valid_) {
      stop_with_state("KEEP_OUT_MASK_MISALIGNED");
      return;
    }
    if (enable_obstacle_avoidance_ && require_fresh_scan_ &&
      (!have_scan_ || (current_time - last_scan_received_).seconds() > scan_timeout_))
    {
      stop_with_state("STALE_SCAN");
      return;
    }
    if (enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::BRAKING) {
      if (first_line_mode_ == "external" && !external_control_released_) {
        stop_with_state("QR_WAIT_CONTROL_RELEASE");
        return;
      }
      const double braking_time = (current_time - qr_switch_start_time_).seconds();
      stop_with_state("QR_BRAKING");
      if (std::abs(measured_velocity_) <= qr_switch_stop_speed_)
      {
        activate_qr_post_path();
      } else if (braking_time >= qr_switch_max_brake_time_) {
        if (first_line_mode_ == "external") {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "视觉控制已释放，但轮速仍为 %.3f m/s；为避免抢控制，继续等待停车",
            measured_velocity_);
        } else {
          RCLCPP_WARN(
            get_logger(), "二维码换段制动等待 %.2f s 后轮速仍为 %.3f m/s，继续切换固定路线",
            braking_time, measured_velocity_);
          activate_qr_post_path();
        }
      }
      return;
    }
    if (completed_) {
      stop_with_state("COMPLETE");
      return;
    }

    if (enable_obstacle_avoidance_) {
      refresh_obstacle_points(current_time);
    }

    std::size_t segment_end = direction_segment_end(progress_index_);
    update_progress_index(segment_end);
    segment_end = direction_segment_end(progress_index_);
    const int direction = tracking_path_[progress_index_].direction;
    maybe_trigger_ring_photo();
    const double path_end_arc = tracking_path_.back().arc_length;
    const double remaining_distance =
      std::max(0.0, path_end_arc - tracking_path_[progress_index_].arc_length);
    const double distance_to_goal = std::hypot(
      tracking_path_.back().pose.x - current_pose_.x,
      tracking_path_.back().pose.y - current_pose_.y);
    const bool visual_parking_armed = visual_parking_is_armed(distance_to_goal, direction);
    if (parking_phase_ == ParkingPhase::FINAL_CRAWL) {
      run_visual_parking(current_time);
      return;
    }
    if (visual_parking_armed && parking_detection_is_fresh(current_time)) {
      if (parking_phase_ == ParkingPhase::IDLE) {
        parking_phase_ = ParkingPhase::APPROACH;
        avoidance_active_ = false;
        reverse_side_rear_active_ = false;
        reverse_side_rear_forward_done_ = false;
        escape_phase_ = EscapePhase::IDLE;
        clear_local_plan();
        RCLCPP_INFO(
          get_logger(), "rush_p is valid near path end; using visual parking command");
      }
      run_visual_parking(current_time);
      return;
    }
    if (parking_phase_ == ParkingPhase::APPROACH) {
      // This is the LIO equivalent of second_line's per-frame rush_p branch:
      // without a current rush_p result, return ownership to normal path tracking.
      parking_phase_ = ParkingPhase::IDLE;
      previous_parking_error_ = 0.0;
      RCLCPP_INFO(
        get_logger(), "rush_p is no longer valid; resuming normal path tracking");
    }
    if (distance_to_goal <= goal_tolerance_ && remaining_distance <= 2.0 * goal_tolerance_) {
      if (enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::SPRINT) {
        stop_with_state("QR_WAIT_DETECTION");
        return;
      }
      completed_ = true;
      stop_with_state("COMPLETE");
      RCLCPP_INFO(get_logger(), "记录路径已完成");
      return;
    }

    const bool direction_change_ahead = segment_end + 1U < tracking_path_.size();
    const double distance_to_segment_end = std::hypot(
      tracking_path_[segment_end].pose.x - current_pose_.x,
      tracking_path_[segment_end].pose.y - current_pose_.y);
    const double segment_arc_remaining = std::max(
      0.0, tracking_path_[segment_end].arc_length -
      tracking_path_[progress_index_].arc_length);
    const double switch_stopping_distance =
      measured_velocity_ * measured_velocity_ /
      (2.0 * std::max(0.1, max_deceleration_));
    const double switch_approach_distance = std::max(
      direction_switch_tolerance_,
      switch_stopping_distance + direction_switch_reaction_time_ *
      std::abs(measured_velocity_) +
      0.5 * direction_switch_tolerance_);
    if (direction_change_ahead &&
      distance_to_segment_end <= switch_approach_distance &&
      segment_arc_remaining <= switch_approach_distance + 0.12)
    {
      direction_switch_pending_ = true;
    }
    const bool at_direction_switch = direction_change_ahead && direction_switch_pending_;
    const bool can_finish_reverse_early = allow_early_reverse_completion_ &&
      direction < 0 && direction_change_ahead &&
      tracking_path_[segment_end + 1U].direction > 0 &&
      distance_to_segment_end <= early_reverse_completion_distance_;

    double lookahead = clamp(
      min_lookahead_ + lookahead_speed_gain_ * std::abs(measured_velocity_),
      min_lookahead_, max_lookahead_);
    if (direction < 0) {
      lookahead = std::min(lookahead, max_reverse_lookahead_);
    }
    if (direction_change_ahead) {
      lookahead = std::min(
        lookahead, std::max(0.15, 0.75 * distance_to_segment_end));
    }
    const std::size_t lookahead_index = find_lookahead_index(lookahead, segment_end);
    const Pose2d & target = tracking_path_[lookahead_index].pose;
    publish_lookahead(target);

    const double dx = target.x - current_pose_.x;
    const double dy = target.y - current_pose_.y;
    const double c = std::cos(current_pose_.yaw);
    const double s = std::sin(current_pose_.yaw);
    const double target_x = c * dx + s * dy;
    const double target_y = -s * dx + c * dy;
    const double target_distance_sq = std::max(1e-4, target_x * target_x + target_y * target_y);
    double curvature = 2.0 * target_y / target_distance_sq;
    const bool ring_section = in_ring_path_section();
    const bool ring_vision_fresh = ring_track_is_fresh(current_time);
    // Normal tracking remains LIO pose + path pure pursuit. Ring vision is an
    // absolute-y observation and a recovery safety cue; it never owns steering.
    curvature = clamp(curvature, -max_curvature_, max_curvature_);

    // A recovery owns the command until both direction transitions are
    // physically settled. This prevents the normal tracker or obstacle state
    // from changing steering midway through the selected reverse rollout.
    if (run_reverse_escape(dt, current_time, ring_section, ring_vision_fresh)) {
      return;
    }

    if (at_direction_switch) {
      apply_acceleration_limit(0.0, dt);
      if (std::abs(measured_velocity_) <= direction_switch_speed_threshold_ &&
        std::abs(commanded_speed_) <= direction_switch_speed_threshold_)
      {
        RCLCPP_INFO(
          get_logger(), "第%zu个路径点换向：%s -> %s",
          segment_end + 1U, direction > 0 ? "前进" : "倒车",
          tracking_path_[segment_end + 1U].direction > 0 ? "前进" : "倒车");
        progress_index_ = segment_end + 1U;
        direction_switch_pending_ = false;
        commanded_speed_ = 0.0;
        publish_command(0.0, 0.0);
        publish_state("DIRECTION_SWITCH");
      } else {
        const double yaw_rate = clamp(
          commanded_speed_ * curvature, -max_yaw_rate_, max_yaw_rate_);
        publish_command(commanded_speed_, yaw_rate);
        publish_state("APPROACH_DIRECTION_SWITCH");
      }
      return;
    }

    const double reaction_distance =
      std::abs(measured_velocity_) * obstacle_reaction_time_ +
      measured_velocity_ * measured_velocity_ / (2.0 * std::max(0.1, max_deceleration_));
    const double trigger_distance = clamp(
      obstacle_min_trigger_distance_ + reaction_distance,
      obstacle_min_trigger_distance_, obstacle_max_trigger_distance_);
    const LidarObstacle lidar_obstacle = enable_obstacle_avoidance_ ?
      evaluate_lidar_obstacle(direction, trigger_distance, curvature) : LidarObstacle{};
    latest_lidar_obstacle_ = lidar_obstacle;
    const bool visual_is_fresh = enable_obstacle_avoidance_ && visual_triggered_ &&
      (current_time - last_visual_received_).seconds() <= vision_timeout_ &&
      (direction > 0 || use_vision_when_reversing_);
    const bool obstacle_now = lidar_obstacle.detected || visual_is_fresh;
    if (enable_obstacle_avoidance_) {
      update_avoidance_state(
        direction, obstacle_now, visual_is_fresh, lidar_obstacle, current_time);
    }

    if (avoidance_active_) {
      double visual_yaw_rate =
        static_cast<double>(avoidance_side_) * static_cast<double>(direction) *
        obstacle_avoid_yaw_rate_;
      if (visual_is_fresh && !reverse_side_rear_active_) {
        const double error = visual_target_error_;
        hat_integral_ = clamp(hat_integral_ + error * dt, -2000.0, 2000.0);
        const double derivative = (error - previous_hat_error_) / std::max(1e-3, dt);
        previous_hat_error_ = error;
        const double visual_turn = hat_kp_ * error + hat_ki_ * hat_integral_ + hat_kd_ * derivative;
        visual_yaw_rate = clamp(-visual_turn, -max_yaw_rate_, max_yaw_rate_);
      }

      if (reverse_side_rear_active_ && direction < 0 &&
        !reverse_side_rear_forward_done_)
      {
        if (start_reverse_escape(
            "REVERSE_SIDE_REAR_FORWARD_CLEARANCE", direction, current_time))
        {
          run_reverse_escape(dt, current_time, ring_section, ring_vision_fresh);
          return;
        }
      }

      LocalPlan local_plan = plan_local_avoidance(
        direction, curvature, avoidance_side_, visual_yaw_rate, candidate_rejections_);
      if (!local_plan.valid) {
        clear_local_plan();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "没有可行的避障轨迹：候选点=%zu 拒绝统计[角速度=%d 路径指标=%d 路径走廊=%d "
          "禁行区=%d 碰撞=%d]",
          laser_points_.size(), candidate_rejections_.yaw_rate,
          candidate_rejections_.path_metric, candidate_rejections_.path_corridor,
          candidate_rejections_.keepout, candidate_rejections_.collision);
        candidate_rejections_ = CandidateRejections{};
        if (can_finish_reverse_early) {
          direction_switch_pending_ = true;
          commanded_speed_ = 0.0;
          publish_command(0.0, 0.0);
          publish_state("EARLY_REVERSE_COMPLETION");
          return;
        }
        if (start_reverse_escape(
            "AVOID_NO_FEASIBLE_TRAJECTORY", direction, current_time))
        {
          run_reverse_escape(dt, current_time, ring_section, ring_vision_fresh);
          return;
        }
        stop_with_state(
          lidar_obstacle.detected && lidar_obstacle.clearance <= obstacle_stop_clearance_ ?
          "AVOID_EMERGENCY_STOP" : "AVOID_NO_FEASIBLE_TRAJECTORY");
        return;
      }
      candidate_rejections_ = CandidateRejections{};

      apply_acceleration_limit(local_plan.speed, dt);
      const double applied_yaw_rate = std::abs(local_plan.speed) > 0.02 ?
        local_plan.yaw_rate * std::abs(commanded_speed_ / local_plan.speed) : 0.0;
      publish_command(commanded_speed_, applied_yaw_rate);
      publish_local_plan(local_plan);
      maybe_reset_reverse_escape_budget();
      if (reverse_side_rear_active_ && direction < 0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "侧后方避障第二阶段：向%s后方倒车，速度=%.2f 曲率=%.2f 安全余量=%.2f",
          avoidance_side_ > 0 ? "左" : "右", local_plan.speed,
          local_plan.yaw_rate / std::min(-0.02, local_plan.speed),
          local_plan.min_obstacle_clearance);
        publish_state("REVERSE_SIDE_REAR_ARC");
      } else if (local_plan.enters_black_area) {
        publish_state(visual_is_fresh ? "AVOID_VISION_BLACK" : "AVOID_LIDAR_BLACK");
      } else {
        publish_state(visual_is_fresh ? "AVOID_VISION_WHITE" : "AVOID_LIDAR_WHITE");
      }
      return;
    }

    if (!select_white_tracking_curvature(curvature, direction, curvature)) {
      if (ring_section) {
        const LocalPlan recovery = plan_ring_recovery(
          direction, curvature, ring_vision_fresh);
        if (recovery.valid) {
          apply_acceleration_limit(recovery.speed, dt);
          const double applied_yaw_rate = std::abs(recovery.speed) > 0.02 ?
            recovery.yaw_rate * std::abs(commanded_speed_ / recovery.speed) : 0.0;
          publish_command(commanded_speed_, applied_yaw_rate);
          publish_local_plan(recovery);
          maybe_reset_reverse_escape_budget();
          if (ring_vision_fresh) {
            publish_state(
              recovery.enters_black_area ?
              "RING_VISION_RECOVERY_BLACK" : "RING_VISION_RECOVERY_WHITE");
          } else {
            publish_state("RING_GEOMETRIC_RECOVERY");
          }
          return;
        }
      }
      if (can_finish_reverse_early) {
        direction_switch_pending_ = true;
        commanded_speed_ = 0.0;
        publish_command(0.0, 0.0);
        publish_state("EARLY_REVERSE_COMPLETION");
        return;
      }
      if (start_reverse_escape("TRACK_NO_WHITE_TRAJECTORY", direction, current_time)) {
        run_reverse_escape(dt, current_time, ring_section, ring_vision_fresh);
        return;
      }
      stop_with_state("TRACK_NO_WHITE_TRAJECTORY");
      return;
    }

    const double nominal_max_speed = path_speed_limit(direction);
    double feasible_speed = nominal_max_speed;
      if (std::abs(curvature) > 1e-4) {
        feasible_speed = std::min(
          feasible_speed, std::sqrt(max_lateral_acceleration_ / std::abs(curvature)));
        feasible_speed = std::min(feasible_speed, max_yaw_rate_ / std::abs(curvature));
      }
      if (direction_change_ahead) {
        const double braking_distance = std::max(
          0.0, distance_to_segment_end - 0.5 * direction_switch_tolerance_);
        feasible_speed = std::min(
          feasible_speed,
          std::sqrt(2.0 * std::max(0.1, max_deceleration_) * braking_distance));
      }
      feasible_speed = std::min(
      feasible_speed,
      std::sqrt(2.0 * std::max(0.1, max_deceleration_) * std::max(0.0, remaining_distance)));

    const double cross_track_error = std::sqrt(
      std::pow(tracking_path_[progress_index_].pose.x - current_pose_.x, 2) +
      std::pow(tracking_path_[progress_index_].pose.y - current_pose_.y, 2));
    if (cross_track_error > cross_track_slowdown_distance_) {
      feasible_speed *= clamp(
        cross_track_slowdown_distance_ / cross_track_error, 0.25, 1.0);
    }

    const double desired_speed = static_cast<double>(direction) * feasible_speed;
    apply_acceleration_limit(desired_speed, dt);
    const double yaw_rate = clamp(commanded_speed_ * curvature, -max_yaw_rate_, max_yaw_rate_);
    publish_command(commanded_speed_, yaw_rate);
    maybe_reset_reverse_escape_budget();
    if (enable_qr_mission_ && qr_mission_phase_ == QrMissionPhase::SPRINT) {
      publish_state("QR_SPRINT");
    } else {
      publish_state(direction > 0 ? "TRACK_FORWARD" : "TRACK_REVERSE");
    }
  }

  bool is_reverse_side_rear_obstacle(
    int direction, const LidarObstacle & lidar_obstacle) const
  {
    return enable_reverse_side_rear_maneuver_ && direction < 0 &&
           lidar_obstacle.detected && std::isfinite(lidar_obstacle.point_x) &&
           std::isfinite(lidar_obstacle.point_y) &&
           lidar_obstacle.point_x <= -reverse_side_rear_min_rearward_ &&
           std::abs(lidar_obstacle.point_y) >= reverse_side_rear_min_lateral_ &&
           std::abs(lidar_obstacle.point_y) <= reverse_side_rear_max_lateral_;
  }

  void update_avoidance_state(
    int direction, bool obstacle_now, bool visual_is_fresh,
    const LidarObstacle & lidar_obstacle,
    const rclcpp::Time & current_time)
  {
    const int vision_side = visual_side_code_ < 0 ? 1 : -1;
    const bool side_rear_obstacle =
      is_reverse_side_rear_obstacle(direction, lidar_obstacle);
    if (!avoidance_active_ && obstacle_now) {
      avoidance_active_ = true;
      avoidance_start_time_ = current_time;
      obstacle_clear_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      reverse_side_rear_active_ = side_rear_obstacle;
      reverse_side_rear_forward_done_ = false;
      if (side_rear_obstacle) {
        // A right-rear cone requires a short right-forward clearance move,
        // followed by a left-rear arc. avoidance_side_ stores the second-stage
        // reverse side, so it is always opposite the obstacle's lateral side.
        avoidance_side_ = lidar_obstacle.point_y < 0.0 ? 1 : -1;
        avoidance_source_visual_ = false;
        reverse_escape_attempts_ = 0;
      } else if (visual_is_fresh) {
        // visual_side_code=-1 creates a target to the image left; the original controller then commands +z.
        avoidance_side_ = vision_side;
        avoidance_source_visual_ = true;
      } else {
        avoidance_side_ = lidar_obstacle.preferred_side;
        avoidance_source_visual_ = false;
      }
      hat_integral_ = 0.0;
      previous_hat_error_ = visual_target_error_;
      RCLCPP_WARN(
        get_logger(),
        "开始避障：来源=%s 绕行侧=%s 安全余量=%.3f "
        "点=(%.3f, %.3f) 模式=%s 环外过滤=%d",
        visual_is_fresh ? "视觉" : "雷达", avoidance_side_ > 0 ? "左侧" : "右侧",
        lidar_obstacle.clearance, lidar_obstacle.point_x, lidar_obstacle.point_y,
        lidar_obstacle.detected ?
        (lidar_obstacle.detected_on_arc ? "弧线" : "直行走廊") : "无",
        lidar_obstacle.ring_filtered_points);
      if (side_rear_obstacle) {
        RCLCPP_WARN(
          get_logger(),
          "侧后方两阶段避障已锁定：障碍在%s后方，先向%s前方行驶%.2f米，再优先向%s后方倒车",
          lidar_obstacle.point_y < 0.0 ? "右" : "左",
          lidar_obstacle.point_y < 0.0 ? "右" : "左",
          reverse_side_rear_forward_distance_,
          avoidance_side_ > 0 ? "左" : "右");
      }
    }

    if (avoidance_active_ && !reverse_side_rear_active_ && side_rear_obstacle) {
      reverse_side_rear_active_ = true;
      reverse_side_rear_forward_done_ = false;
      avoidance_side_ = lidar_obstacle.point_y < 0.0 ? 1 : -1;
      avoidance_source_visual_ = false;
      reverse_escape_attempts_ = 0;
      RCLCPP_WARN(
        get_logger(), "避障过程中发现%s后方障碍，切换为侧后方两阶段策略",
        lidar_obstacle.point_y < 0.0 ? "右" : "左");
    }

    // Lidar can trigger earlier at race speed, but once HAT is visible the
    // proven camera policy owns the side decision. Lock it for this obstacle
    // so detector jitter cannot alternate left/right commands.
    if (avoidance_active_ && visual_is_fresh && !avoidance_source_visual_ &&
      !reverse_side_rear_active_)
    {
      avoidance_side_ = vision_side;
      avoidance_source_visual_ = true;
      hat_integral_ = 0.0;
      previous_hat_error_ = visual_target_error_;
      RCLCPP_INFO(
        get_logger(), "视觉接管避障方向：%s",
        avoidance_side_ > 0 ? "左侧" : "右侧");
    }

    if (!avoidance_active_) {
      return;
    }
    if (obstacle_now) {
      obstacle_clear_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      return;
    }
    if (obstacle_clear_since_.nanoseconds() == 0) {
      obstacle_clear_since_ = current_time;
      return;
    }

    const double active_duration = (current_time - avoidance_start_time_).seconds();
    const double clear_duration = (current_time - obstacle_clear_since_).seconds();
    if (active_duration >= obstacle_min_avoid_duration_ && clear_duration >= obstacle_clear_duration_) {
      avoidance_active_ = false;
      avoidance_source_visual_ = false;
      reverse_side_rear_active_ = false;
      reverse_side_rear_forward_done_ = false;
      hat_integral_ = 0.0;
      previous_hat_error_ = 0.0;
      clear_local_plan();
      RCLCPP_INFO(get_logger(), "障碍已清除，返回记录路径跟踪");
    }
  }

  void apply_acceleration_limit(double desired_speed, double dt)
  {
    if (commanded_speed_ * desired_speed < 0.0 && std::abs(commanded_speed_) > 0.02) {
      desired_speed = 0.0;
    }
    const double delta = desired_speed - commanded_speed_;
    const bool increasing_magnitude = std::abs(desired_speed) > std::abs(commanded_speed_);
    const double limit =
      (increasing_magnitude ? max_acceleration_ : max_deceleration_) * dt;
    commanded_speed_ += clamp(delta, -limit, limit);
  }

  void publish_command(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_x;
    command.angular.z = angular_z;
    cmd_pub_->publish(command);
    last_commanded_yaw_rate_ = angular_z;
    publish_debug_markers(linear_x, angular_z);
  }

  void publish_debug_markers(double linear_x, double angular_z)
  {
    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = now();

    visualization_msgs::msg::Marker footprint;
    footprint.header.stamp = stamp;
    footprint.header.frame_id = path_frame_;
    footprint.ns = "vehicle";
    footprint.id = 0;
    footprint.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint.action = visualization_msgs::msg::Marker::ADD;
    footprint.scale.x = 0.025;
    footprint.color.a = 1.0F;
    footprint.color.r = avoidance_active_ ? 1.0F : 0.1F;
    footprint.color.g = avoidance_active_ ? 0.75F : 0.9F;
    footprint.color.b = 0.1F;
    const std::array<Eigen::Vector2d, 5> corners{{
      Eigen::Vector2d(robot_half_length_, robot_half_width_),
      Eigen::Vector2d(robot_half_length_, -robot_half_width_),
      Eigen::Vector2d(-robot_half_length_, -robot_half_width_),
      Eigen::Vector2d(-robot_half_length_, robot_half_width_),
      Eigen::Vector2d(robot_half_length_, robot_half_width_)}};
    for (const auto & corner : corners) {
      const Eigen::Vector2d world = transform_point(current_pose_, corner);
      geometry_msgs::msg::Point point;
      point.x = world.x();
      point.y = world.y();
      point.z = 0.06;
      footprint.points.push_back(point);
    }
    markers.markers.push_back(footprint);

    visualization_msgs::msg::Marker command;
    command.header = footprint.header;
    command.ns = "command";
    command.id = 1;
    command.type = visualization_msgs::msg::Marker::ARROW;
    command.action = visualization_msgs::msg::Marker::ADD;
    command.scale.x = 0.035;
    command.scale.y = 0.075;
    command.scale.z = 0.10;
    command.color.a = 1.0F;
    command.color.r = 0.1F;
    command.color.g = 0.8F;
    command.color.b = 1.0F;
    geometry_msgs::msg::Point arrow_start;
    arrow_start.x = current_pose_.x;
    arrow_start.y = current_pose_.y;
    arrow_start.z = 0.10;
    const double command_heading = current_pose_.yaw + clamp(
      angular_z / std::max(0.20, std::abs(linear_x)) * 0.25, -0.8, 0.8);
    const double arrow_length = std::abs(linear_x) > 0.01 ?
      0.25 + 0.35 * std::abs(linear_x) : 0.0;
    geometry_msgs::msg::Point arrow_end = arrow_start;
    arrow_end.x += std::copysign(arrow_length, linear_x == 0.0 ? 1.0 : linear_x) *
      std::cos(command_heading);
    arrow_end.y += std::copysign(arrow_length, linear_x == 0.0 ? 1.0 : linear_x) *
      std::sin(command_heading);
    command.points = {arrow_start, arrow_end};
    markers.markers.push_back(command);

    visualization_msgs::msg::Marker status;
    status.header = footprint.header;
    status.ns = "status";
    status.id = 2;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.position.x = current_pose_.x;
    status.pose.position.y = current_pose_.y;
    status.pose.position.z = 0.36;
    status.pose.orientation.w = 1.0;
    status.scale.z = 0.14;
    status.color.a = 1.0F;
    status.color.r = 1.0F;
    status.color.g = 1.0F;
    status.color.b = 1.0F;
    std::ostringstream text;
    text << (last_state_.empty() ? "STARTING" : last_state_) << "  v=" <<
      std::fixed << std::setprecision(2) << linear_x << "  w=" << angular_z;
    if (avoidance_active_) {
      text << (avoidance_source_visual_ ? "  VISION" : "  LIDAR") <<
        (avoidance_side_ > 0 ? " LEFT" : " RIGHT");
    }
    if (escape_phase_ != EscapePhase::IDLE) {
      text << "\nESCAPE " << reverse_escape_attempts_ << '/' << reverse_escape_max_attempts_ <<
        " cause=" << escape_cause_ << " k=" << escape_selected_curvature_;
    }
    if (visual_triggered_ && (stamp - last_visual_received_).seconds() <= vision_timeout_) {
      text << "\nHAT h=" << visual_height_ratio_ << " cx=" << visual_selected_center_ <<
        " width=" << visual_selected_width_ << " error=" << visual_target_error_;
    }
    if (in_ring_path_section()) {
      text << "\nRING conf=" << ring_track_confidence_ <<
        " steer=" << ring_track_steering_error_ <<
        " lateral=" << ring_track_lateral_offset_ <<
        " black=" << forbidden_footprint_fraction(current_pose_);
      if (enable_ring_localization_correction_) {
        text << "\nYLOC " << (ring_localization_applied_ ? "APPLIED" : "WAIT") <<
          " src=" << (ring_localization_depth_valid_ ? "RGBD" : "NONE") <<
          " lat=" << ring_localization_lateral_m_ <<
          " dy=" << ring_localization_correction_ <<
          " solid=" << ring_localization_solid_fraction_;
      }
    }
    status.text = text.str();
    markers.markers.push_back(status);

    visualization_msgs::msg::Marker trigger;
    trigger.header = footprint.header;
    trigger.ns = "lidar_trigger";
    trigger.id = 3;
    trigger.type = visualization_msgs::msg::Marker::SPHERE;
    if (latest_lidar_obstacle_.detected &&
      std::isfinite(latest_lidar_obstacle_.point_x) &&
      std::isfinite(latest_lidar_obstacle_.point_y))
    {
      trigger.action = visualization_msgs::msg::Marker::ADD;
      const Eigen::Vector2d world = transform_point(
        current_pose_, Eigen::Vector2d(
          latest_lidar_obstacle_.point_x, latest_lidar_obstacle_.point_y));
      trigger.pose.position.x = world.x();
      trigger.pose.position.y = world.y();
      trigger.pose.position.z = 0.10;
      trigger.pose.orientation.w = 1.0;
      trigger.scale.x = 0.12;
      trigger.scale.y = 0.12;
      trigger.scale.z = 0.12;
      trigger.color.a = 1.0F;
      trigger.color.r = 1.0F;
      trigger.color.g = 0.05F;
      trigger.color.b = 0.05F;
    } else {
      trigger.action = visualization_msgs::msg::Marker::DELETE;
    }
    markers.markers.push_back(trigger);
    debug_marker_pub_->publish(markers);
  }

  void stop_with_state(const std::string & state)
  {
    commanded_speed_ = 0.0;
    publish_command(0.0, 0.0);
    publish_state(state);
  }

  void publish_state(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    last_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
    RCLCPP_INFO(
      get_logger(), "控制器状态：%s（%s）", state_label_chinese(state), state.c_str());
  }

  static const char * state_label_chinese(const std::string & state)
  {
    if (state == "PARKING_VISUAL_APPROACH") {return "visual parking approach";}
    if (state == "PARKING_FINAL_CRAWL") {return "visual parking final crawl";}
    if (state == "PARKING_COMPLETE") {return "visual parking complete";}
    if (state == "WAIT_ODOM") {return "等待新鲜定位";}
    if (state == "QR_EXTERNAL_MONITOR") {return "二维码前仅监控定位，等待外部视觉控制";}
    if (state == "QR_WAIT_CONTROL_RELEASE") {return "已扫码，等待视觉控制释放";}
    if (state == "QR_VISUAL_WAIT_CMD") {return "等待第一赛段视觉控制命令";}
    if (state == "QR_VISUAL_FIRST_LINE") {return "第一赛段由origincar_control纯视觉控制";}
    if (state == "QR_SPRINT") {return "冲向二维码扫描区";}
    if (state == "QR_WAIT_DETECTION") {return "已到扫码终点，等待二维码结果";}
    if (state == "QR_BRAKING") {return "识别二维码后立即制动";}
    if (state == "QR_FIXED_PATH_TRACK") {return "二维码后固定路线巡线";}
    if (state == "TRACK_FORWARD") {return "正向跟踪路径";}
    if (state == "TRACK_REVERSE") {return "倒车跟踪路径";}
    if (state == "APPROACH_DIRECTION_SWITCH") {return "接近换向点";}
    if (state == "DIRECTION_SWITCH") {return "正在换向";}
    if (state == "EARLY_REVERSE_COMPLETION") {return "提前结束倒车段";}
    if (state == "STALE_SCAN") {return "雷达数据超时";}
    if (state == "COMPLETE") {return "路径完成";}
    if (state == "TRACK_NO_WHITE_TRAJECTORY") {return "正常轨道内无可行轨迹";}
    if (state == "AVOID_NO_FEASIBLE_TRAJECTORY") {return "避障无可行轨迹";}
    if (state == "AVOID_EMERGENCY_STOP") {return "避障紧急停车";}
    if (state == "AVOID_VISION_BLACK") {return "视觉避障，允许借用禁行区";}
    if (state == "AVOID_VISION_WHITE") {return "视觉避障，保持正常区域";}
    if (state == "AVOID_LIDAR_BLACK") {return "雷达避障，允许借用禁行区";}
    if (state == "AVOID_LIDAR_WHITE") {return "雷达避障，保持正常区域";}
    if (state == "ESCAPE_BRAKING") {return "脱困前制动";}
    if (state == "ESCAPE_TRAJECTORY_SELECTED") {return "已选定脱困轨迹";}
    if (state == "ESCAPE_REVERSING") {return "倒车脱困";}
    if (state == "ESCAPE_FORWARD_CLEARANCE") {return "前进脱困";}
    if (state == "REVERSE_SIDE_REAR_FORWARD") {return "侧后方避障：向障碍侧前方腾挪";}
    if (state == "REVERSE_SIDE_REAR_ARC") {return "侧后方避障：反向弧线绕行";}
    if (state == "ESCAPE_NO_SAFE_TRAJECTORY") {return "没有安全脱困轨迹";}
    if (state == "ESCAPE_TIMEOUT_BRAKE") {return "脱困超时制动";}
    if (state == "ESCAPE_DISTANCE_BRAKE") {return "达到脱困距离，正在制动";}
    if (state == "ESCAPE_BLOCKED_BRAKE") {return "脱困方向受阻，正在制动";}
    if (state == "ESCAPE_REPLAN") {return "脱困后重新规划";}
    if (state == "RING_VISION_RECOVERY_BLACK") {return "环形区视觉纠偏，允许借用禁行区";}
    if (state == "RING_VISION_RECOVERY_WHITE") {return "环形区视觉纠偏，保持正常区域";}
    if (state == "RING_GEOMETRIC_RECOVERY") {return "环形区几何脱困";}
    if (state == "KEEP_OUT_MASK_MISALIGNED") {return "禁行区掩码未对齐";}
    return "未分类状态";
  }

  void publish_lookahead(const Pose2d & target)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = path_frame_;
    pose.pose.position.x = target.x;
    pose.pose.position.y = target.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, target.yaw);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    lookahead_pub_->publish(pose);
  }

  void publish_local_plan(const LocalPlan & local_plan)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = path_frame_;
    for (const auto & state : local_plan.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = state.x;
      pose.pose.position.y = state.y;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, state.yaw);
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
      path.poses.push_back(pose);
    }
    local_plan_pub_->publish(path);
  }

  void clear_local_plan()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = path_frame_;
    local_plan_pub_->publish(path);
  }

  std::string path_file_;
  bool enable_qr_mission_{true};
  std::string qr_sprint_path_file_;
  std::string qr_clockwise_path_file_;
  std::string qr_counterclockwise_path_file_;
  std::string qr_sign_topic_{"/sign_switch"};
  bool qr_allow_keepout_{false};
  double qr_detection_arm_delay_{0.50};
  double qr_switch_stop_speed_{0.04};
  double qr_switch_max_brake_time_{2.0};
  std::string first_line_mode_{"lio"};
  std::string control_started_topic_{"/origincar_control/started"};
  std::string control_released_topic_{"/origincar_control/control_released"};
  std::string visual_first_line_cmd_topic_{"/lio_ob/visual_first_line_cmd"};
  double visual_first_line_cmd_timeout_{0.30};
  std::string odom_topic_;
  std::string scan_topic_;
  std::string vision_topic_;
  std::string cmd_vel_topic_;
  std::string path_frame_;
  bool align_path_to_start_{true};
  int ring_path_start_point_{78};
  int ring_path_end_point_{159};
  double ring_path_y_offset_{0.0};
  int ring_path_blend_points_{4};
  bool enable_ring_photo_trigger_{true};
  std::string ring_photo_trigger_topic_{"/take_photo/trigger"};
  int ring_photo_forward_path_point_{125};
  int ring_photo_reverse_path_point_{120};
  double ring_photo_max_abs_yaw_rate_{0.25};
  int ring_photo_force_after_points_{5};
  bool use_keepout_mask_{false};
  std::string keepout_mask_file_;
  double keepout_resolution_{0.0};
  Pose2d keepout_origin_;
  int keepout_white_threshold_{128};
  double max_path_mask_violation_ratio_{0.10};
  int keepout_width_{0};
  int keepout_height_{0};
  std::vector<std::uint8_t> keepout_pixels_;
  Pose2d path_alignment_;
  bool mask_alignment_valid_{true};

  double control_frequency_{30.0};
  double min_lookahead_{0.42};
  double max_lookahead_{0.95};
  double lookahead_speed_gain_{0.42};
  double max_forward_speed_{0.50};
  double max_reverse_speed_{0.30};
  double post_qr_transition_forward_speed_{0.28};
  double post_qr_transition_reverse_speed_{0.28};
  double post_qr_transition_end_y_{1.90};
  std::size_t post_qr_transition_end_index_{0U};
  bool auto_select_post_qr_route_{false};
  double auto_route_obstacle_influence_radius_{0.55};
  std::vector<std::string> auto_route_clockwise_path_files_;
  std::vector<std::string> auto_route_counterclockwise_path_files_;
  double max_yaw_rate_{2.50};
  double max_lateral_acceleration_{2.20};
  double max_acceleration_{2.00};
    double max_deceleration_{0.35};
    double goal_tolerance_{0.12};
    double cross_track_slowdown_distance_{0.75};
    double max_reverse_lookahead_{0.35};
    double direction_switch_tolerance_{0.16};
    double direction_switch_speed_threshold_{0.04};
    double direction_switch_reaction_time_{0.75};
    bool allow_early_reverse_completion_{true};
    double early_reverse_completion_distance_{0.45};
  double odom_timeout_{0.50};
  double max_odom_message_age_{0.30};
  double scan_timeout_{0.30};
  bool require_fresh_scan_{true};
  bool enable_obstacle_avoidance_{true};

  Pose2d laser_extrinsic_;
  double robot_half_length_{0.16};
  double robot_half_width_{0.11};
  bool enable_ring_obstacle_path_filter_{true};
  double ring_obstacle_path_half_width_{0.30};
  double obstacle_corridor_margin_{0.08};
  double obstacle_min_trigger_distance_{0.45};
  double obstacle_max_trigger_distance_{2.20};
  double obstacle_reaction_time_{0.75};
  double obstacle_stop_clearance_{0.10};
  double obstacle_avoid_speed_{0.90};
  double obstacle_avoid_yaw_rate_{1.80};
  double obstacle_min_avoid_duration_{0.70};
  double obstacle_clear_duration_{0.40};
  bool enable_reverse_escape_{true};
  double reverse_escape_speed_{0.18};
  double reverse_escape_distance_{0.38};
  double reverse_escape_max_duration_{4.0};
  double reverse_escape_max_curvature_{2.0};
  int reverse_escape_curvature_samples_{11};
  int reverse_escape_max_attempts_{2};
  double reverse_escape_path_half_width_{0.70};
  double reverse_escape_stop_speed_{0.04};
  double reverse_escape_reset_progress_{0.30};
  double reverse_escape_reaction_time_{0.75};
  double reverse_escape_settle_time_{0.20};
  bool enable_reverse_side_rear_maneuver_{true};
  double reverse_side_rear_min_rearward_{0.02};
  double reverse_side_rear_min_lateral_{0.06};
  double reverse_side_rear_max_lateral_{0.60};
  double reverse_side_rear_forward_distance_{0.30};
  double reverse_side_rear_forward_min_curvature_{0.60};
  bool deskew_obstacle_scan_{true};
  bool scan_index_time_reversed_{true};
  double scan_distortion_margin_{0.04};
  double local_plan_horizon_{1.80};
  double local_plan_step_distance_{0.05};
  double local_plan_turn_fraction_{0.40};
  int local_plan_curvature_samples_{15};
  int local_plan_speed_samples_{1};
  double max_curvature_{2.5};
  double preferred_path_half_width_{0.32};
  double emergency_path_half_width_{0.80};
  double local_path_window_{2.5};
  double progress_search_window_{1.5};
  double nominal_mask_check_distance_{0.45};
  bool enable_ring_vision_{true};
  std::string ring_binary_topic_;
  int ring_start_path_point_{72};
  int ring_end_path_point_{160};
  double ring_vision_timeout_{0.25};
  double ring_vision_min_confidence_{0.35};
  int ring_image_threshold_{127};
  int ring_image_row_step_{8};
  int ring_image_scan_rows_{240};
  int ring_image_min_valid_rows_{10};
  int ring_image_min_run_width_{20};
  double ring_image_max_run_width_ratio_{0.85};
  double ring_image_lookahead_px_{180.0};
  bool enable_ring_localization_correction_{true};
  std::string ring_localization_correction_topic_;
  int ring_localization_start_path_point_{78};
  int ring_localization_end_path_point_{86};
  double ring_localization_lane_width_{0.50};
  bool ring_localization_use_depth_{true};
  bool ring_localization_require_depth_{true};
  std::string ring_depth_topic_;
  std::string ring_camera_info_topic_;
  double ring_depth_sync_tolerance_{0.12};
  double ring_depth_scale_{0.001};
  double ring_depth_min_range_{0.20};
  double ring_depth_max_range_{4.0};
  int ring_depth_search_radius_{3};
  int ring_depth_min_center_points_{6};
  double ring_depth_min_forward_span_{0.35};
  double ring_depth_max_lane_width_error_{0.18};
  double ring_depth_max_fit_rmse_{0.06};
  double ring_camera_x_{0.0};
  double ring_camera_y_{0.0};
  double ring_localization_camera_center_x_ratio_{0.50};
  double ring_localization_min_y_observability_{0.90};
  double ring_localization_vehicle_row_ratio_{0.96};
  double ring_localization_min_row_ratio_{0.52};
  double ring_localization_horizon_row_ratio_{0.25};
  double ring_localization_min_lane_width_px_{80.0};
  double ring_localization_expected_width_px_{0.0};
  double ring_localization_max_fit_rmse_ratio_{0.10};
  double ring_localization_min_solid_fraction_{0.78};
  double ring_localization_max_heading_error_{0.40};
  int ring_localization_sample_count_{5};
  double ring_localization_max_sample_spread_{0.05};
  double ring_localization_measurement_std_{0.025};
  double ring_localization_max_correction_{0.35};
  double ring_recovery_speed_{0.28};
  double ring_recovery_horizon_{0.65};
  double ring_recovery_path_half_width_{0.55};
  double ring_recovery_min_mask_improvement_{0.05};
  double ring_field_boundary_margin_{0.05};
  double obstacle_safety_margin_{0.05};
  double obstacle_memory_duration_{1.0};
  double obstacle_memory_radius_{2.5};
  double obstacle_memory_voxel_size_{0.06};
  bool enable_lidar_noise_filter_{true};
  int scan_cluster_max_beam_gap_{2};
  double scan_cluster_max_distance_{0.16};
  int scan_cluster_min_points_{3};
  int obstacle_confirmation_hits_{2};
  double obstacle_confirmation_window_{0.30};
  double obstacle_confirmation_radius_{0.10};
  double unconfirmed_obstacle_ttl_{0.35};
  bool enable_speed_weighted_obstacle_memory_{true};
  double obstacle_low_speed_threshold_{0.35};
  double obstacle_high_speed_measurement_weight_{0.20};
  double obstacle_stopped_measurement_weight_{0.75};
  bool filter_field_boundary_returns_{true};
  double field_min_x_{0.0};
  double field_max_x_{5.0};
  double field_min_y_{0.0};
  double field_max_y_{5.0};
  double field_boundary_filter_band_{0.20};
  bool filter_wall_line_clusters_{true};
  int wall_cluster_min_points_{8};
  double wall_cluster_min_length_{0.45};
  double wall_cluster_max_thickness_{0.07};
  double wall_cluster_min_linearity_{0.97};
  double weight_path_distance_{18.0};
  double weight_path_heading_{2.0};
  double weight_path_return_{24.0};
  double weight_black_area_{80.0};
  double weight_obstacle_clearance_{0.8};
  double weight_steering_change_{0.8};
  double weight_preferred_side_{10.0};
  double weight_progress_{7.0};
  double weight_speed_{8.0};
  double weight_visual_command_{24.0};

  double image_width_{1920.0};
  double image_height_{1080.0};
  double image_center_x_{920.0};
  double hat_trigger_ratio_{0.22};
  double hat_force_trigger_ratio_{0.50};
  double hat_horizontal_limit_{500.0};
  double hat_width_scale_{1.55};
  double hat_avoid_target_scale_{1.60};
  double hat_direction_bias_px_{180.0};
  double hat_kp_{0.002};
  double hat_ki_{0.00007};
  double hat_kd_{0.00007};
  double vision_timeout_{0.35};
  bool use_vision_when_reversing_{false};
  bool enable_visual_parking_{true};
  bool visual_parking_only_post_qr_{true};
  std::string parking_target_type_{"P"};
  double parking_arm_distance_{2.00};
  double parking_min_confidence_{0.0};
  double parking_detection_timeout_{0.35};
  double parking_obstacle_margin_px_{150.0};
  double parking_center_x_{960.0};
  double parking_decel_start_y_{500.0};
  double parking_trigger_y_{1030.0};
  double parking_stop_y_{1080.0};
  double parking_max_speed_{0.80};
  double parking_kp_{0.0025};
  double parking_kd_{0.0010};
  double parking_final_crawl_speed_{0.36};
  double parking_final_crawl_duration_{0.60};

  std::vector<PathPoint> recorded_path_;
  std::vector<PathPoint> tracking_path_;
  std::vector<PathPoint> qr_sprint_path_;
  std::vector<PathPoint> qr_clockwise_path_;
  std::vector<PathPoint> qr_counterclockwise_path_;
  std::vector<std::vector<PathPoint>> auto_route_clockwise_candidates_;
  std::vector<std::vector<PathPoint>> auto_route_counterclockwise_candidates_;
  std::vector<std::string> auto_route_clockwise_candidate_names_;
  std::vector<std::string> auto_route_counterclockwise_candidate_names_;
  std::vector<Eigen::Vector2d> laser_points_;
  std::unordered_map<std::uint64_t, TimedObstaclePoint> obstacle_memory_;
  std::uint64_t scan_sequence_{0U};
  Pose2d current_pose_;
  std::size_t progress_index_{0U};
  double measured_velocity_{0.0};
  double measured_yaw_rate_{0.0};
  double commanded_speed_{0.0};
  double last_commanded_yaw_rate_{0.0};
  bool have_odom_{false};
  bool have_scan_{false};
  bool path_aligned_{false};
  bool completed_{false};
  bool direction_switch_pending_{false};
  bool have_visual_first_line_cmd_{false};
  bool external_control_started_{false};
  bool external_control_released_{false};
  geometry_msgs::msg::Twist visual_first_line_command_;
  QrMissionPhase qr_mission_phase_{QrMissionPhase::DISABLED};
  std::size_t qr_detection_progress_index_{0U};
  int qr_selected_sign_{0};
  Pose2d qr_detection_pose_;

  bool visual_triggered_{false};
  int visual_side_code_{0};
  double visual_target_error_{0.0};
  double visual_height_ratio_{0.0};
  double visual_selected_center_{0.0};
  double visual_selected_width_{0.0};
  double hat_integral_{0.0};
  double previous_hat_error_{0.0};
  ParkingPhase parking_phase_{ParkingPhase::IDLE};
  bool parking_detection_valid_{false};
  double parking_target_center_x_{0.0};
  double parking_target_ymax_{0.0};
  double previous_parking_error_{0.0};
  double ring_track_steering_error_{0.0};
  double ring_track_lateral_offset_{0.0};
  double ring_track_confidence_{0.0};
  double ring_last_center_x_{-1.0};
  double ring_last_run_width_{-1.0};
  double ring_localization_width_px_estimate_{0.0};
  double ring_localization_lateral_m_{0.0};
  double ring_localization_solid_fraction_{0.0};
  double ring_localization_measured_y_{0.0};
  double ring_localization_correction_{0.0};
  bool ring_localization_window_active_{false};
  bool ring_localization_attempted_{false};
  bool ring_localization_applied_{false};
  bool ring_localization_depth_valid_{false};
  bool ring_photo_triggered_{false};
  std::vector<double> ring_localization_y_samples_;
  bool avoidance_active_{false};
  bool avoidance_source_visual_{false};
  bool reverse_side_rear_active_{false};
  bool reverse_side_rear_forward_done_{false};
  LidarObstacle latest_lidar_obstacle_;
  int avoidance_side_{1};
  EscapePhase escape_phase_{EscapePhase::IDLE};
  int escape_path_direction_{1};
  int reverse_escape_attempts_{0};
  double escape_selected_curvature_{0.0};
  double escape_selected_distance_{0.0};
  double escape_braking_curvature_{0.0};
  double reverse_escape_last_arc_{-std::numeric_limits<double>::infinity()};
  std::string escape_cause_;
  std::string escape_settling_state_{"ESCAPE_SETTLING"};
  LocalPlan escape_selected_plan_;
  rclcpp::Time last_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_visual_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_parking_detection_{0, 0, RCL_ROS_TIME};
  rclcpp::Time parking_final_crawl_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ring_image_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time avoidance_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time obstacle_clear_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time escape_phase_start_time_{0, 0, RCL_ROS_TIME};
  Pose2d reverse_escape_start_pose_;
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time qr_mission_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time qr_switch_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_visual_first_line_cmd_{0, 0, RCL_ROS_TIME};
  std::string last_state_;
  CandidateRejections candidate_rejections_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr vision_sub_;
  rclcpp::Subscription<origincar_msg::msg::Sign>::SharedPtr qr_sign_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr visual_first_line_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_started_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_released_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ring_binary_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ring_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr ring_camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ring_photo_trigger_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr keepout_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr ring_vision_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ring_localization_pub_;
  sensor_msgs::msg::Image::SharedPtr latest_ring_depth_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_ring_camera_info_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace lio_ob

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lio_ob::PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
