from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': 0.0625,
        'reference_frame': '',
        'camera_frame': 'camera_color_optical_frame',
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', '/camera/color/camera_info'),
                    ('/image', '/camera/color/image_raw')]
    )

    return [aruco_marker_publisher]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
