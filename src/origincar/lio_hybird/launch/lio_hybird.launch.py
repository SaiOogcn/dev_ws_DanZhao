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
    package_share = get_package_share_directory("lio_hybird")
    default_params = os.path.join(package_share, "config", "lio_hybird.yaml")
    default_keepout_mask = os.path.join(package_share, "config", "field_keepout_mask.pgm")
    default_rviz_config = os.path.join(package_share, "rviz", "lio_hybird.rviz")
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
    default_qr_reverse_full_real_ring_clockwise_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_clockwise.txt",
    )
    default_qr_reverse_full_real_ring_counterclockwise_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_counterclockwise.txt",
    )
    default_qr_reverse_full_real_ring_clockwise_straight_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_clockwise_straight.txt",
    )
    default_qr_reverse_full_real_ring_counterclockwise_straight_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_counterclockwise_straight.txt",
    )
    default_qr_reverse_full_real_ring_clockwise_left_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_clockwise_left.txt",
    )
    default_qr_reverse_full_real_ring_counterclockwise_left_path = os.path.join(
        package_share,
        "paths",
        "path_post_qr_reverse_full_real_ring_counterclockwise_left.txt",
    )
    auto_route_clockwise_paths = [
        default_qr_reverse_ring_clockwise_path,
        default_qr_reverse_ring_clockwise_inner_path,
        default_qr_reverse_full_ring_clockwise_path,
        default_qr_reverse_full_real_ring_clockwise_path,
        default_qr_reverse_full_real_ring_clockwise_straight_path,
        default_qr_reverse_full_real_ring_clockwise_left_path,
    ]
    auto_route_counterclockwise_paths = [
        default_qr_reverse_ring_counterclockwise_path,
        default_qr_reverse_ring_counterclockwise_inner_path,
        default_qr_reverse_full_ring_counterclockwise_path,
        default_qr_reverse_full_real_ring_counterclockwise_path,
        default_qr_reverse_full_real_ring_counterclockwise_straight_path,
        default_qr_reverse_full_real_ring_counterclockwise_left_path,
    ]
    default_qr_symmetry_clockwise_path = os.path.join(
        package_share, "paths", "symmetry_path_clockwise.txt"
    )
    default_qr_symmetry_counterclockwise_path = os.path.join(
        package_share, "paths", "symmetry_path_counterclockwise.txt"
    )
    qr_detection_launch = os.path.join(
        get_package_share_directory("zxing_qr_code_detection"),
        "launch/"
        "zxing_qr_code_detection.launch.py",
    )

    params_file = LaunchConfiguration("params_file")
    path_file = LaunchConfiguration("path_file")
    rviz = LaunchConfiguration("rviz")
    enable_obstacle_avoidance = LaunchConfiguration("enable_obstacle_avoidance")
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
        default_qr_reverse_full_real_ring_clockwise_path,
        "' if '", post_qr_route_mode, "' == 'reverse_full_real_ring' else '",
        default_qr_reverse_full_real_ring_clockwise_straight_path,
        "' if '", post_qr_route_mode, "' == 'better_second' else '",
        default_qr_symmetry_clockwise_path,
        "' if '", post_qr_route_mode, "' == 'symmetry' else '",
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
        default_qr_reverse_full_real_ring_counterclockwise_path,
        "' if '", post_qr_route_mode, "' == 'reverse_full_real_ring' else '",
        default_qr_reverse_full_real_ring_counterclockwise_straight_path,
        "' if '", post_qr_route_mode, "' == 'better_second' else '",
        default_qr_symmetry_counterclockwise_path,
        "' if '", post_qr_route_mode, "' == 'symmetry' else '",
        default_qr_reverse_ring_counterclockwise_inner_path,
        "' if '", post_qr_route_mode,
        "' in ('reverse_ring_inner', 'reverse_ring_ccw_inner') else '",
        default_qr_reverse_ring_counterclockwise_path, "'",
    ])
    pre_qr_control_mode = LaunchConfiguration("pre_qr_control_mode")
    lio_cmd_topic = LaunchConfiguration("lio_cmd_topic")
    control_started_topic = LaunchConfiguration("control_started_topic")
    control_released_topic = LaunchConfiguration("control_released_topic")
    post_qr_transition_forward_speed = LaunchConfiguration(
        "post_qr_transition_forward_speed"
    )
    post_qr_transition_reverse_speed = LaunchConfiguration(
        "post_qr_transition_reverse_speed"
    )
    post_qr_transition_end_y = LaunchConfiguration(
        "post_qr_transition_end_y"
    )
    auto_select_post_qr_route = LaunchConfiguration(
        "auto_select_post_qr_route"
    )
    auto_route_obstacle_influence_radius = LaunchConfiguration(
        "auto_route_obstacle_influence_radius"
    )
    enable_speed_weighted_obstacle_memory = LaunchConfiguration(
        "enable_speed_weighted_obstacle_memory"
    )
    obstacle_low_speed_threshold = LaunchConfiguration(
        "obstacle_low_speed_threshold"
    )
    obstacle_high_speed_measurement_weight = LaunchConfiguration(
        "obstacle_high_speed_measurement_weight"
    )
    obstacle_stopped_measurement_weight = LaunchConfiguration(
        "obstacle_stopped_measurement_weight"
    )
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
                "reverse_full_real_ring",
                "better_second",
                "symmetry",
                "parking",
                "right_turn",
                "reverse_ring_ccw",
                "reverse_ring_ccw_inner",
            ],
            description=(
                "Post-QR route family. QR sign 3/4 selects clockwise/counterclockwise; "
                "reverse_ring is the default reverse-entry route; "
                "reverse_full_real_ring selects the measured full-reverse routes; "
                "better_second selects the straight-entry measured full-reverse routes; "
                "symmetry selects the symmetry_path clockwise/counterclockwise pair"
            ),
        ),
        DeclareLaunchArgument(
            "pre_qr_control_mode",
            default_value="external",
            choices=["external", "lio"],
            description="Before QR: external standalone origincar_control or LIO path tracking",
        ),
        DeclareLaunchArgument(
            "post_qr_transition_forward_speed",
            default_value="0.28",
            description="Forward speed limit after QR and before the ring channel, m/s",
        ),
        DeclareLaunchArgument(
            "post_qr_transition_reverse_speed",
            default_value="0.28",
            description="Reverse speed limit after QR and before the ring channel, m/s",
        ),
        DeclareLaunchArgument(
            "post_qr_transition_end_y",
            default_value="1.90",
            description="Map Y coordinate where the route enters the ring channel, m",
        ),
        DeclareLaunchArgument(
            "auto_select_post_qr_route",
            default_value="false",
            description=(
                "After QR, choose the reverse-entry route with the fewest and easiest "
                "obstacles before the ring channel"
            ),
        ),
        DeclareLaunchArgument(
            "auto_route_obstacle_influence_radius",
            default_value="0.55",
            description="Obstacle distance included in automatic route scoring, m",
        ),
        DeclareLaunchArgument(
            "enable_speed_weighted_obstacle_memory",
            default_value="true",
            description="Give low-speed lidar scans more influence on obstacle world positions",
        ),
        DeclareLaunchArgument(
            "obstacle_low_speed_threshold",
            default_value="0.35",
            description="Speed below which obstacle measurement trust increases, m/s",
        ),
        DeclareLaunchArgument(
            "obstacle_high_speed_measurement_weight",
            default_value="0.20",
            description="Obstacle position update weight at/above the low-speed threshold",
        ),
        DeclareLaunchArgument(
            "obstacle_stopped_measurement_weight",
            default_value="0.75",
            description="Obstacle position update weight when the vehicle is stopped",
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
            package="lio_hybird",
            executable="submap_lio_node",
            name="submap_lio",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="lio_hybird",
            executable="pure_pursuit_node",
            name="lio_hybird_controller",
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
                    "post_qr_transition_forward_speed": ParameterValue(
                        post_qr_transition_forward_speed, value_type=float
                    ),
                    "post_qr_transition_reverse_speed": ParameterValue(
                        post_qr_transition_reverse_speed, value_type=float
                    ),
                    "post_qr_transition_end_y": ParameterValue(
                        post_qr_transition_end_y, value_type=float
                    ),
                    "auto_select_post_qr_route": ParameterValue(
                        auto_select_post_qr_route, value_type=bool
                    ),
                    "auto_route_obstacle_influence_radius": ParameterValue(
                        auto_route_obstacle_influence_radius, value_type=float
                    ),
                    "auto_route_clockwise_path_files": auto_route_clockwise_paths,
                    "auto_route_counterclockwise_path_files": auto_route_counterclockwise_paths,
                    "enable_speed_weighted_obstacle_memory": ParameterValue(
                        enable_speed_weighted_obstacle_memory, value_type=bool
                    ),
                    "obstacle_low_speed_threshold": ParameterValue(
                        obstacle_low_speed_threshold, value_type=float
                    ),
                    "obstacle_high_speed_measurement_weight": ParameterValue(
                        obstacle_high_speed_measurement_weight, value_type=float
                    ),
                    "obstacle_stopped_measurement_weight": ParameterValue(
                        obstacle_stopped_measurement_weight, value_type=float
                    ),
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
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="lio_hybird_rviz",
            arguments=["-d", default_rviz_config],
            output="screen",
            condition=IfCondition(rviz),
        ),
    ])
