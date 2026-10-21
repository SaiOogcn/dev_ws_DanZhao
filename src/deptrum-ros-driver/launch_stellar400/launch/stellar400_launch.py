from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    device_node = Node(
        package="deptrum-ros-driver-stellar400",
        executable="stellar400_node",
        namespace="stellar",
        parameters=[
            {
                "rgb_enable": LaunchConfiguration('rgb_enable', default=True),
                "ir_enable": LaunchConfiguration('ir_enable', default=True),
                "depth_enable": LaunchConfiguration('depth_enable', default=True),
                "point_cloud_enable": LaunchConfiguration('point_cloud_enable', default=True),
                "boot_order": LaunchConfiguration('boot_order', default=1),
                "ir_fps": LaunchConfiguration('ir_fps', default=25),
                "rgb_fps": LaunchConfiguration('rgb_fps', default=25),
                "resolution_mode_index": LaunchConfiguration('resolution_mode_index', default=0),
                "usb_port_number": LaunchConfiguration('usb_port_number', default=""),
                "serial_number": LaunchConfiguration('serial_number', default=""),
                "rgbd_enable": LaunchConfiguration('rgbd_enable', default=False),
                "exposure_enable": LaunchConfiguration('exposure_enable', default=False),
                "exposure_time": LaunchConfiguration('exposure_time', default=1000),
                "undistortion_enable": LaunchConfiguration('undistortion_enable', default=False),
                "filter_type": LaunchConfiguration('filter_type', default=1),
                "depth_frequency_fusion_threshold": LaunchConfiguration('depth_frequency_fusion_threshold', default=120.0),
                "ir_frequency_fusion_threshold": LaunchConfiguration('ir_frequency_fusion_threshold', default=24),
                "ratio_scatter_filter_threshold": LaunchConfiguration('ratio_scatter_filter_threshold', default=0.025),
                "outlier_point_removal_flag": LaunchConfiguration('outlier_point_removal_flag', default=True),
                "heart_enable": LaunchConfiguration('heart_enable', default=True),
                "update_file_path": LaunchConfiguration('update_file_path', default=""),
                "log_dir": LaunchConfiguration('log_dir', default="/tmp/"),
                "heart_timeout_times": LaunchConfiguration('heart_timeout_times', default=8),
                "stream_sdk_log_enable": LaunchConfiguration('stream_sdk_log_enable', default=True),
            }
        ],
        arguments=None,
        output="screen",   
    )
    ld.add_action(device_node)

    return ld