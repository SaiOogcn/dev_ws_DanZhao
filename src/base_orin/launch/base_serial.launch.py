from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom',
         'cmd_vel': 'cmd_vel',
         'gyro_frame_id': 'gyro_link',
         'calibrate_imu_gyro_z_bias': True,
         'imu_gyro_bias_warmup_samples': 200,
         'imu_gyro_bias_calibration_samples': 200,
         'imu_gyro_z_bias': 0.00128,
         'imu_gyro_z_variance': 1e-6,
         'imu_gyro_bias_max_stddev': 0.0015,
         # Four straight runs measured a robust chassis bias of -0.04644 rad/m.
         'chassis_curvature_compensation': 0.0464,
         'product_number': 0}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='base_orin',
            executable='base_orin_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'ackermann_cmd'}],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='base_orin',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
        ),

        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='base_orin',
            executable='base_orin_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'none'}],
        )
    ])
