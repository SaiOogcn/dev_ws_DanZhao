from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "raw_image_topic",
            default_value="/aurora/rgb/image_raw",
            description="sensor_msgs/Image topic; set to an empty string to disable",
        ),
        DeclareLaunchArgument(
            "compressed_image_topic",
            default_value="/image",
            description="sensor_msgs/CompressedImage topic; set to an empty string to disable",
        ),
        DeclareLaunchArgument(
            "scan_interval_ms",
            default_value="150",
            description="Minimum scan interval for each input topic",
        ),
        DeclareLaunchArgument(
            "stop_after_detection",
            default_value="true",
            description="Stop processing images after the first valid QR code",
        ),
        DeclareLaunchArgument(
            "try_harder",
            default_value="true",
            description="Let zxing-cpp spend more time searching for a QR code",
        ),
        DeclareLaunchArgument(
            "try_rotate",
            default_value="true",
            description="Search for QR codes in rotated orientations",
        ),
    ]

    detector = Node(
        package="zxing_qr_code_detection",
        executable="zxing_qr_detection_node",
        name="zxing_qr_detection",
        output="screen",
        parameters=[
            {
                "raw_image_topic": LaunchConfiguration("raw_image_topic"),
                "compressed_image_topic": LaunchConfiguration(
                    "compressed_image_topic"
                ),
                "scan_interval_ms": ParameterValue(
                    LaunchConfiguration("scan_interval_ms"), value_type=int
                ),
                "stop_after_detection": ParameterValue(
                    LaunchConfiguration("stop_after_detection"), value_type=bool
                ),
                "try_harder": ParameterValue(
                    LaunchConfiguration("try_harder"), value_type=bool
                ),
                "try_rotate": ParameterValue(
                    LaunchConfiguration("try_rotate"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(arguments + [detector])
