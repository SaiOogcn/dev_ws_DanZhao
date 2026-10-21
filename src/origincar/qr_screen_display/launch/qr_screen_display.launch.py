from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('font_path', default_value=''),
        DeclareLaunchArgument('fullscreen', default_value='true'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument(
            'enable_photo_client',
            default_value='true',
            description='Start the ring camera/ZMQ inference client',
        ),
        DeclareLaunchArgument('trigger_topic', default_value='/take_photo/trigger'),
        DeclareLaunchArgument('result_topic', default_value='/take_photo/result'),
        DeclareLaunchArgument('status_topic', default_value='/take_photo/status'),
        DeclareLaunchArgument(
            'front_image_topic',
            default_value='/image',
            description='Front YOLO camera sensor_msgs/CompressedImage topic',
        ),
        DeclareLaunchArgument(
            'aurora_image_topic',
            default_value='/aurora/rgb/image_raw',
            description='Rear Aurora sensor_msgs/Image topic',
        ),
        DeclareLaunchArgument('server_ip', default_value='172.20.10.9'),
        DeclareLaunchArgument('server_port', default_value='5555'),
        DeclareLaunchArgument('debug_image_path', default_value='/tmp/ring_photo_latest.jpg'),
        Node(
            package='qr_screen_display',
            executable='qr_screen_display',
            parameters=[{
                'font_path': LaunchConfiguration('font_path'),
                'fullscreen': ParameterValue(
                    LaunchConfiguration('fullscreen'), value_type=bool),
                'width': ParameterValue(
                    LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(
                    LaunchConfiguration('height'), value_type=int),
                'caption_topic': LaunchConfiguration('result_topic'),
            }],
            output='screen',
        ),
        Node(
            package='qr_screen_display',
            executable='ring_photo_client',
            name='ring_photo_client',
            parameters=[{
                'trigger_topic': LaunchConfiguration('trigger_topic'),
                'result_topic': LaunchConfiguration('result_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'front_image_topic': LaunchConfiguration('front_image_topic'),
                'aurora_image_topic': LaunchConfiguration('aurora_image_topic'),
                'server_ip': LaunchConfiguration('server_ip'),
                'server_port': ParameterValue(
                    LaunchConfiguration('server_port'), value_type=int),
                'debug_image_path': LaunchConfiguration('debug_image_path'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_photo_client')),
        ),
    ])
