#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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

    return launch.LaunchDescription([
        phidgets_container,
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        Node(
            package='controller',
            executable='cartesian_controller'
        ),  # chassis cartetian controller
        Node(
            package='chassis_controller',
            executable='joy_controller'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyUSB0", "-v6"]
        ),
    ])
