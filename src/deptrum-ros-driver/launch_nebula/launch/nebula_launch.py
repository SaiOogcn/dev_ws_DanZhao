from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="deptrum-ros-driver-nebula",
            executable="nebula_node",
            namespace="nebula",
            parameters=[
                {"rgb_enable": LaunchConfiguration('rgb_enable', default=True),
                 "ir_enable": LaunchConfiguration('ir_enable', default=True),
                 "depth_enable": LaunchConfiguration('depth_enable', default=True),
                 "flag_enable": LaunchConfiguration('flag_enable', default=True),
                 "point_cloud_enable": LaunchConfiguration('point_cloud_enable', default=True),
                 "slam_mode": LaunchConfiguration('slam_mode', default=1),
                 "boot_order": LaunchConfiguration('boot_order', default=1),
                 "resolution_mode_index": LaunchConfiguration('resolution_mode_index', default=0),
                 "usb_port_number": LaunchConfiguration('usb_port_number', default=""),
                 "serial_number": LaunchConfiguration('serial_number', default=""),
                 "rgbd_enable": LaunchConfiguration('rgbd_enable', default=False),
                 "exposure_enable": LaunchConfiguration('exposure_enable', default=False),
                 "exposure_time": LaunchConfiguration('exposure_time', default=1000),
                 "mtof_filter_level": LaunchConfiguration('mtof_filter_level', default=1),
                 "stof_filter_level": LaunchConfiguration('stof_filter_level', default=1),
                 "update_file_path": LaunchConfiguration('update_file_path', default=""),
                 "log_dir": LaunchConfiguration('log_dir', default="/tmp/"),
                 "stream_sdk_log_enable": LaunchConfiguration('stream_sdk_log_enable', default=True),
                 "ir_fps": LaunchConfiguration('ir_fps', default=25),
                 "stof_minimum_range": LaunchConfiguration('stof_minimum_range', default=10),
                 "mtof_crop_up": LaunchConfiguration('mtof_crop_up', default=50),
                 "mtof_crop_down": LaunchConfiguration('mtof_crop_down', default=80),
                 "ext_rgb_mode": LaunchConfiguration('ext_rgb_mode', default=0),
                 "enable_imu": LaunchConfiguration('enable_imu', default=True),
                 }
            ],
            arguments=None,
            output="screen",   
        )
    ])