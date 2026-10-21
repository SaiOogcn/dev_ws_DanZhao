from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("lio_aurora")
    default_params = os.path.join(package_share, "config", "lio_aurora.yaml")
    default_keepout_mask = os.path.join(package_share, "config", "field_keepout_mask.pgm")
    default_rviz_config = os.path.join(package_share, "rviz", "lio_aurora.rviz")
    default_path = os.path.join(package_share, "paths", "path_competition_ideal.txt")
    default_qr_sprint_path = os.path.join(package_share, "paths", "path_qr_sprint.txt")
    default_qr_parking_clockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_parking_clockwise.txt"
    )
    default_qr_parking_counterclockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_parking_counterclockwise.txt"
    )
    default_qr_right_turn_clockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_right_turn_clockwise.txt"
    )
    default_qr_right_turn_counterclockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_right_turn_counterclockwise.txt"
    )
    default_qr_reverse_ring_counterclockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_ring_counterclockwise.txt"
    )
    default_qr_reverse_ring_counterclockwise_inner_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_ring_counterclockwise_inner.txt"
    )
    default_qr_reverse_ring_clockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_ring_clockwise.txt"
    )
    default_qr_reverse_ring_clockwise_inner_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_ring_clockwise_inner.txt"
    )
    default_qr_reverse_full_ring_clockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_full_ring_clockwise.txt"
    )
    default_qr_reverse_full_ring_counterclockwise_path = os.path.join(
        package_share, "paths", "path_post_qr_reverse_full_ring_counterclockwise.txt"
    )
    qr_detection_launch = os.path.join(
        get_package_share_directory("qr_code_detection"),
        "qr_code_detection.launch.py",
    )

    params_file = LaunchConfiguration("params_file")
    path_file = LaunchConfiguration("path_file")
    rviz = LaunchConfiguration("rviz")
    enable_obstacle_avoidance = LaunchConfiguration("enable_obstacle_avoidance")
    enable_rear_depth_avoidance = LaunchConfiguration(
        "enable_rear_depth_avoidance"
    )
    rear_depth_topic = LaunchConfiguration("rear_depth_topic")
    use_lidar_when_reversing = LaunchConfiguration("use_lidar_when_reversing")
    require_fresh_rear_depth_when_reversing = LaunchConfiguration(
        "require_fresh_rear_depth_when_reversing"
    )
    rear_depth_camera_x = LaunchConfiguration("rear_depth_camera_x")
    rear_depth_camera_y = LaunchConfiguration("rear_depth_camera_y")
    rear_depth_camera_z = LaunchConfiguration("rear_depth_camera_z")
    rear_depth_camera_yaw = LaunchConfiguration("rear_depth_camera_yaw")
    rear_depth_upside_down = LaunchConfiguration("rear_depth_upside_down")
    rear_depth_avoidance_end_path_point = LaunchConfiguration(
        "rear_depth_avoidance_end_path_point"
    )
    rear_depth_avoid_speed = LaunchConfiguration("rear_depth_avoid_speed")
    rear_depth_channel_speed = LaunchConfiguration("rear_depth_channel_speed")
    rear_depth_min_trigger_distance = LaunchConfiguration(
        "rear_depth_min_trigger_distance"
    )
    rear_depth_reaction_time = LaunchConfiguration("rear_depth_reaction_time")
    rear_depth_local_plan_horizon = LaunchConfiguration(
        "rear_depth_local_plan_horizon"
    )
    rear_depth_local_plan_turn_fraction = LaunchConfiguration(
        "rear_depth_local_plan_turn_fraction"
    )
    rear_depth_weight_obstacle_clearance = LaunchConfiguration(
        "rear_depth_weight_obstacle_clearance"
    )
    rear_depth_weight_steering_change = LaunchConfiguration(
        "rear_depth_weight_steering_change"
    )
    rear_depth_close_turn_bias = LaunchConfiguration("rear_depth_close_turn_bias")
    rear_depth_forward_shuffle_speed = LaunchConfiguration(
        "rear_depth_forward_shuffle_speed"
    )
    use_keepout_mask = LaunchConfiguration("use_keepout_mask")
    ring_path_y_offset = LaunchConfiguration("ring_path_y_offset")
    enable_ring_photo_trigger = LaunchConfiguration("enable_ring_photo_trigger")
    ring_photo_trigger_topic = LaunchConfiguration("ring_photo_trigger_topic")
    ring_photo_forward_path_point = LaunchConfiguration(
        "ring_photo_forward_path_point"
    )
    ring_photo_reverse_path_point = LaunchConfiguration(
        "ring_photo_reverse_path_point"
    )
    ring_photo_max_abs_yaw_rate = LaunchConfiguration(
        "ring_photo_max_abs_yaw_rate"
    )
    ring_photo_force_after_points = LaunchConfiguration(
        "ring_photo_force_after_points"
    )
    ring_vision = LaunchConfiguration("ring_vision")
    ring_camera_topic = LaunchConfiguration("ring_camera_topic")
    ring_vision_debug = LaunchConfiguration("ring_vision_debug")
    enable_qr_mission = LaunchConfiguration("enable_qr_mission")
    post_qr_route_mode = LaunchConfiguration("post_qr_route_mode")
    qr_detection = LaunchConfiguration("qr_detection")
    qr_sprint_path_file = LaunchConfiguration("qr_sprint_path_file")
    qr_clockwise_path_file = LaunchConfiguration("qr_clockwise_path_file")
    qr_counterclockwise_path_file = LaunchConfiguration(
        "qr_counterclockwise_path_file"
    )
    default_qr_clockwise_path = PythonExpression([
        "'", default_qr_parking_clockwise_path,
        "' if '", post_qr_route_mode, "' == 'parking' else '",
        default_qr_right_turn_clockwise_path,
        "' if '", post_qr_route_mode, "' == 'right_turn' else '",
        default_qr_reverse_full_ring_clockwise_path,
        "' if '", post_qr_route_mode, "' == 'reverse_full_ring' else '",
        default_qr_reverse_ring_clockwise_inner_path,
        "' if '", post_qr_route_mode,
        "' in ('reverse_ring_inner', 'reverse_ring_ccw_inner') else '",
        default_qr_reverse_ring_clockwise_path, "'",
    ])
    default_qr_counterclockwise_path = PythonExpression([
        "'", default_qr_parking_counterclockwise_path,
        "' if '", post_qr_route_mode, "' == 'parking' else '",
        default_qr_right_turn_counterclockwise_path,
        "' if '", post_qr_route_mode, "' == 'right_turn' else '",
        default_qr_reverse_full_ring_counterclockwise_path,
        "' if '", post_qr_route_mode, "' == 'reverse_full_ring' else '",
        default_qr_reverse_ring_counterclockwise_inner_path,
        "' if '", post_qr_route_mode,
        "' in ('reverse_ring_inner', 'reverse_ring_ccw_inner') else '",
        default_qr_reverse_ring_counterclockwise_path, "'",
    ])
    pre_qr_control_mode = LaunchConfiguration("pre_qr_control_mode")
    lio_cmd_topic = LaunchConfiguration("lio_cmd_topic")
    control_started_topic = LaunchConfiguration("control_started_topic")
    control_released_topic = LaunchConfiguration("control_released_topic")
    ring_processing_initially_enabled = PythonExpression([
        "not ('", enable_qr_mission, "' == 'true' and '",
        pre_qr_control_mode, "' == 'external')",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Planar LIO and controller parameter file",
        ),
        DeclareLaunchArgument(
            "path_file",
            default_value=default_path,
            description="Recorded CSV path produced by dzsavepath",
        ),
        DeclareLaunchArgument(
            "enable_qr_mission",
            default_value="true",
            description="Run QR first stage, selected post-QR entry path and ring mission",
        ),
        DeclareLaunchArgument(
            "post_qr_route_mode",
            default_value="reverse_ring",
            choices=[
                "reverse_ring",
                "reverse_ring_inner",
                "reverse_full_ring",
                "parking",
                "right_turn",
                "reverse_ring_ccw",
                "reverse_ring_ccw_inner",
            ],
            description=(
                "Post-QR route family. QR sign 3/4 selects clockwise/counterclockwise; "
                "reverse_ring is the default reverse-entry route"
            ),
        ),
        DeclareLaunchArgument(
            "pre_qr_control_mode",
            default_value="external",
            choices=["external", "lio"],
            description="Before QR: external standalone origincar_control or LIO path tracking",
        ),
        DeclareLaunchArgument(
            "lio_cmd_topic",
            default_value="/cmd_vel",
            description="Direct chassis velocity output used by the post-QR LIO controller",
        ),
        DeclareLaunchArgument(
            "control_started_topic",
            default_value="/origincar_control/started",
            description="Latched acknowledgement that the visual first stage was armed by keypress",
        ),
        DeclareLaunchArgument(
            "control_released_topic",
            default_value="/origincar_control/control_released",
            description="One-shot release acknowledgement from standalone origincar_control",
        ),
        DeclareLaunchArgument(
            "qr_detection",
            default_value="true",
            description="Start qr_code_detection as part of this launch",
        ),
        DeclareLaunchArgument(
            "qr_sprint_path_file",
            default_value=default_qr_sprint_path,
            description="Forward sprint path ending at the QR scan corner",
        ),
        DeclareLaunchArgument(
            "qr_clockwise_path_file",
            default_value=default_qr_clockwise_path,
            description="Optional selected post-QR clockwise path template",
        ),
        DeclareLaunchArgument(
            "qr_counterclockwise_path_file",
            default_value=default_qr_counterclockwise_path,
            description="Optional selected post-QR counterclockwise path template",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start RViz2 with the LIO/controller debug layout",
        ),
        DeclareLaunchArgument(
            "enable_obstacle_avoidance",
            default_value="true",
            description="Enable lidar/vision local obstacle avoidance",
        ),
        DeclareLaunchArgument(
            "enable_rear_depth_avoidance",
            default_value="true",
            description="Use the rear Aurora point cloud on reverse path segments",
        ),
        DeclareLaunchArgument(
            "rear_depth_topic",
            default_value="/aurora/points2",
            description="Rear-facing Aurora PointCloud2 topic",
        ),
        DeclareLaunchArgument(
            "use_lidar_when_reversing",
            default_value="true",
            description="Fuse lidar with Aurora during reverse; false selects Aurora-only A/B mode",
        ),
        DeclareLaunchArgument(
            "require_fresh_rear_depth_when_reversing",
            default_value="true",
            description="Stop reverse motion when the Aurora point cloud is stale",
        ),
        DeclareLaunchArgument(
            "rear_depth_camera_x",
            default_value="-0.1375",
            description="Rear Aurora optical centre x in base_footprint, metres",
        ),
        DeclareLaunchArgument(
            "rear_depth_camera_y",
            default_value="0.0",
            description="Rear Aurora optical centre y in base_footprint, metres",
        ),
        DeclareLaunchArgument(
            "rear_depth_camera_z",
            default_value="0.16",
            description="Rear Aurora optical centre height above ground, metres",
        ),
        DeclareLaunchArgument(
            "rear_depth_camera_yaw",
            default_value="0.0",
            description="Rear Aurora yaw correction relative to straight backward, radians",
        ),
        DeclareLaunchArgument(
            "rear_depth_upside_down",
            default_value="true",
            description="Rotate the rear Aurora optical image by 180 degrees",
        ),
        DeclareLaunchArgument(
            "rear_depth_avoidance_end_path_point",
            default_value="41",
            description="First path point where Aurora avoidance is disabled",
        ),
        DeclareLaunchArgument(
            "rear_depth_channel_speed",
            default_value="0.55",
            description="Reverse cruise speed in the post-QR channel, m/s",
        ),
        DeclareLaunchArgument(
            "rear_depth_avoid_speed",
            default_value="0.55",
            description="Reverse avoidance speed in the post-QR channel, m/s",
        ),
        DeclareLaunchArgument(
            "rear_depth_min_trigger_distance",
            default_value="0.35",
            description="Base Aurora obstacle trigger distance in the channel, metres",
        ),
        DeclareLaunchArgument(
            "rear_depth_reaction_time",
            default_value="0.30",
            description="Aurora channel reaction-time allowance, seconds",
        ),
        DeclareLaunchArgument(
            "rear_depth_local_plan_horizon",
            default_value="1.30",
            description="Aurora channel local-plan rollout distance, metres",
        ),
        DeclareLaunchArgument(
            "rear_depth_local_plan_turn_fraction",
            default_value="0.46",
            description="Fraction of Aurora rollout spent turning away and returning",
        ),
        DeclareLaunchArgument(
            "rear_depth_weight_obstacle_clearance",
            default_value="0.45",
            description="Soft obstacle-clearance cost for Aurora channel planning",
        ),
        DeclareLaunchArgument(
            "rear_depth_weight_steering_change",
            default_value="0.20",
            description="Steering-change cost for Aurora channel planning",
        ),
        DeclareLaunchArgument(
            "rear_depth_close_turn_bias",
            default_value="0.60",
            description="Reward for decisive curvature in Aurora channel avoidance",
        ),
        DeclareLaunchArgument(
            "rear_depth_forward_shuffle_speed",
            default_value="0.28",
            description="Forward shuffle speed when Aurora reverse planning is blocked, m/s",
        ),
        DeclareLaunchArgument(
            "use_keepout_mask",
            default_value="true",
            description="Enable the competition keepout mask constraint",
        ),
        DeclareLaunchArgument(
            "ring_path_y_offset",
            default_value="0.0",
            description="Temporary +y offset in metres for ring path points",
        ),
        DeclareLaunchArgument(
            "enable_ring_photo_trigger",
            default_value="true",
            description="Trigger one PC-inference photo before the third ring corner",
        ),
        DeclareLaunchArgument(
            "ring_photo_trigger_topic",
            default_value="/take_photo/trigger",
            description="One-shot camera selection topic consumed by ring_photo_client",
        ),
        DeclareLaunchArgument(
            "ring_photo_forward_path_point",
            default_value="125",
            description="One-based photo point for forward ring travel/front camera",
        ),
        DeclareLaunchArgument(
            "ring_photo_reverse_path_point",
            default_value="120",
            description="One-based photo point for full-reverse ring travel/Aurora",
        ),
        DeclareLaunchArgument(
            "ring_photo_max_abs_yaw_rate",
            default_value="0.25",
            description="Maximum measured yaw rate for a low-blur ring photo",
        ),
        DeclareLaunchArgument(
            "ring_photo_force_after_points",
            default_value="5",
            description="Force the photo after this many points if yaw rate stays high",
        ),
        DeclareLaunchArgument(
            "ring_vision",
            default_value="true",
            description="Start the Aurora image-to-track observation pipeline",
        ),
        DeclareLaunchArgument(
            "ring_camera_topic",
            default_value="/aurora/rgb/image_raw",
            description="BGR camera topic used for upper-ring track observation",
        ),
        DeclareLaunchArgument(
            "ring_vision_debug",
            default_value="false",
            description="Publish the binary RGB debug image",
        ),
        Node(
            package="image_bridge_hsv",
            executable="image_bridge_hsv",
            name="ring_image_bridge_hsv",
            output="screen",
            parameters=[{
                "input_image_topic": ring_camera_topic,
                "output_hsv_topic": "/image_new_hsv",
                "input_is_bgr": True,
                "show_debug_log": False,
                "start_enabled": ParameterValue(
                    ring_processing_initially_enabled, value_type=bool
                ),
                "enable_topic": control_released_topic,
            }],
            condition=IfCondition(ring_vision),
        ),
        Node(
            package="image_binary",
            executable="image_binary",
            name="ring_image_binary",
            output="screen",
            parameters=[{
                "input_hsv_topic": "/image_new_hsv",
                "yellow_binary_topic": "/image_yellow_binary",
                "publish_separate_masks": True,
                "publish_rgb_debug": ParameterValue(ring_vision_debug, value_type=bool),
                "show_debug_log": False,
                "yellow_h_low": 15,
                "yellow_h_high": 35,
                "yellow_s_low": 70,
                "yellow_s_high": 255,
                "yellow_v_low": 50,
                "yellow_v_high": 255,
                "green_h_low": 35,
                "green_h_high": 90,
                "green_s_low": 50,
                "green_s_high": 255,
                "green_v_low": 40,
                "green_v_high": 255,
            }],
            condition=IfCondition(ring_vision),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(qr_detection_launch),
            condition=IfCondition(qr_detection),
        ),
        Node(
            package="lio_aurora",
            executable="submap_lio_node",
            name="submap_lio",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="lio_aurora",
            executable="pure_pursuit_node",
            name="lio_aurora_controller",
            output="screen",
            parameters=[
                params_file,
                {
                    "path_file": path_file,
                    "enable_qr_mission": ParameterValue(
                        enable_qr_mission, value_type=bool
                    ),
                    "qr_sprint_path_file": qr_sprint_path_file,
                    "qr_clockwise_path_file": qr_clockwise_path_file,
                    "qr_counterclockwise_path_file": qr_counterclockwise_path_file,
                    "first_line_mode": pre_qr_control_mode,
                    "cmd_vel_topic": lio_cmd_topic,
                    "control_started_topic": control_started_topic,
                    "control_released_topic": control_released_topic,
                    "keepout_mask_file": default_keepout_mask,
                    "use_keepout_mask": ParameterValue(
                        use_keepout_mask, value_type=bool
                    ),
                    "ring_path_y_offset": ParameterValue(
                        ring_path_y_offset, value_type=float
                    ),
                    "enable_ring_photo_trigger": ParameterValue(
                        enable_ring_photo_trigger, value_type=bool
                    ),
                    "ring_photo_trigger_topic": ring_photo_trigger_topic,
                    "ring_photo_forward_path_point": ParameterValue(
                        ring_photo_forward_path_point, value_type=int
                    ),
                    "ring_photo_reverse_path_point": ParameterValue(
                        ring_photo_reverse_path_point, value_type=int
                    ),
                    "ring_photo_max_abs_yaw_rate": ParameterValue(
                        ring_photo_max_abs_yaw_rate, value_type=float
                    ),
                    "ring_photo_force_after_points": ParameterValue(
                        ring_photo_force_after_points, value_type=int
                    ),
                    "enable_obstacle_avoidance": ParameterValue(
                        enable_obstacle_avoidance, value_type=bool
                    ),
                    "enable_rear_depth_avoidance": ParameterValue(
                        enable_rear_depth_avoidance, value_type=bool
                    ),
                    "rear_depth_topic": rear_depth_topic,
                    "use_lidar_when_reversing": ParameterValue(
                        use_lidar_when_reversing, value_type=bool
                    ),
                    "require_fresh_rear_depth_when_reversing": ParameterValue(
                        require_fresh_rear_depth_when_reversing, value_type=bool
                    ),
                    "rear_depth_camera_x": ParameterValue(
                        rear_depth_camera_x, value_type=float
                    ),
                    "rear_depth_camera_y": ParameterValue(
                        rear_depth_camera_y, value_type=float
                    ),
                    "rear_depth_camera_z": ParameterValue(
                        rear_depth_camera_z, value_type=float
                    ),
                    "rear_depth_camera_yaw": ParameterValue(
                        rear_depth_camera_yaw, value_type=float
                    ),
                    "rear_depth_upside_down": ParameterValue(
                        rear_depth_upside_down, value_type=bool
                    ),
                    "rear_depth_avoidance_end_path_point": ParameterValue(
                        rear_depth_avoidance_end_path_point, value_type=int
                    ),
                    "rear_depth_avoid_speed": ParameterValue(
                        rear_depth_avoid_speed, value_type=float
                    ),
                    "rear_depth_channel_speed": ParameterValue(
                        rear_depth_channel_speed, value_type=float
                    ),
                    "rear_depth_min_trigger_distance": ParameterValue(
                        rear_depth_min_trigger_distance, value_type=float
                    ),
                    "rear_depth_reaction_time": ParameterValue(
                        rear_depth_reaction_time, value_type=float
                    ),
                    "rear_depth_local_plan_horizon": ParameterValue(
                        rear_depth_local_plan_horizon, value_type=float
                    ),
                    "rear_depth_local_plan_turn_fraction": ParameterValue(
                        rear_depth_local_plan_turn_fraction, value_type=float
                    ),
                    "rear_depth_weight_obstacle_clearance": ParameterValue(
                        rear_depth_weight_obstacle_clearance, value_type=float
                    ),
                    "rear_depth_weight_steering_change": ParameterValue(
                        rear_depth_weight_steering_change, value_type=float
                    ),
                    "rear_depth_close_turn_bias": ParameterValue(
                        rear_depth_close_turn_bias, value_type=float
                    ),
                    "rear_depth_forward_shuffle_speed": ParameterValue(
                        rear_depth_forward_shuffle_speed, value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="lio_aurora_rviz",
            arguments=["-d", default_rviz_config],
            output="screen",
            condition=IfCondition(rviz),
        ),
    ])
