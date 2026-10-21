from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='teleop_ackermann_joy',
            executable='teleop_ackermann_joy_node',
            name='teleop_ackermann_joy',
            output='screen',
            parameters=[{
                'joy_topic': '/joy',
                'cmd_vel_topic': '/cmd_vel',
                'axis_linear': 1,
                'axis_angular': 0,
                'linear_sign': 1.0,
                'angular_sign': 1.0,
                'deadzone': 0.1,
                'default_speed': 0.5,
                'max_speed': 2.0,
                'max_angular': 10.0,
                'accel_rate': 0.1,
                'decel_rate': 0.2,
                'button_accel': 5,
                'button_decel': 4,
                'button_brake': 2,
                'use_trigger_axes': False,
                'rt_axis': 5,
                'lt_axis': 2,
                'joy_timeout': 0.5,
                'publish_rate': 20.0,
            }],
        )
    ])
