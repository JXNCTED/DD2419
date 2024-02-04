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
        realsense_path, 'launch/rs_launch.py')), launch_arguments={'pointcloud.enable': 'true'}.items())

    bringup_path = get_package_share_directory('bringup')

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
            arguments=["serial", "--dev", "/dev/ttyUSB0", "-v6"]
        ),

        # realsense
        realsense_launch,
        Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
            '--child-frame-id', 'camera_link', '--frame-id', 'base_link']),

        # lidar
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
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
    ])
