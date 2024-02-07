#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    phidgets_container = ComposableNodeContainer(
        name='robp_phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='robp_phidgets_motors',
                    plugin='robp::phidgets::Motors',
                    name='robp_phidgets_motors'),
                ComposableNode(
                    package='robp_phidgets_encoders',
                    plugin='robp::phidgets::Encoders',
                    name='robp_phidgets_encoders'),
                ComposableNode(
                    package='robp_phidgets_spatial',
                    plugin='robp::phidgets::Spatial',
                    name='robp_phidgets_spatial'),
                ComposableNode(
                    package='robp_phidgets_temperature',
                    plugin='robp::phidgets::Temperature',
                    name='robp_phidgets_temperature'),
        ],
        output='both',
    )  # all phidgets node
    realsense_path = get_package_share_directory('realsense2_camera')
    realsense_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        realsense_path, 'launch/rs_launch.py')), launch_arguments={'pointcloud.enable': 'true', 'enable_rgbd': 'true',
                                                                   'enable_gyro': 'true', 'enable_accel': 'true'}.items())

    bringup_path = get_package_share_directory('bringup')

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

    return launch.LaunchDescription([
        # phidgets
        phidgets_container,

        # joy
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),

        # chassis controller

        Node(package='controller',
             executable='cartesian_controller',
             ),

        # servo arm
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"]
        ),

        # realsense.
        realsense_launch,
        Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
            '--child-frame-id', 'camera_link', '--frame-id', 'base_link', '--x', '0.08987', '--y', '0.0175', '--z', '0.10456']),

        # lidar
        Node(
            name='rplidar_composition',
            # name='rplidar_node.cpp',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
            '--child-frame-id', 'lidar_link', '--frame-id', 'base_link']),


        # camera on arm
        Node(package='usb_cam',
             executable='usb_cam_node_exe',
             parameters=[os.path.join(bringup_path, 'config/usb_cam_param.yaml')]),
        aruco_marker_publisher,

    ])
