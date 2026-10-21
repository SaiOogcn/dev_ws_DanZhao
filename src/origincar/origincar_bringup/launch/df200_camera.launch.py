from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'device',
            default_value=TextSubstitution(text='/dev/video2'),
            description='DF200 usb camera device'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value=TextSubstitution(text='1920'),
            description='DF200 image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value=TextSubstitution(text='1080'),
            description='DF200 image height'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value=TextSubstitution(text='30'),
            description='DF200 camera framerate'
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value=TextSubstitution(text='/image_new'),
            description='DF200 output image topic'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value=TextSubstitution(text='/camera_info_new'),
            description='DF200 output camera_info topic'
        ),
    ]

    df200_camera_node = Node(
        package='hobot_usb_cam',
        executable='hobot_usb_cam',
        name='df200_camera_node',
        output='screen',
        parameters=[
            {
                'video_device': LaunchConfiguration('device'),
                'pixel_format': 'mjpeg2rgb',
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
            }
        ],
        remappings=[
            ('/image', LaunchConfiguration('image_topic')),
            ('/camera_info', LaunchConfiguration('camera_info_topic')),
        ]
    )

    return LaunchDescription(launch_args + [
        df200_camera_node
    ])