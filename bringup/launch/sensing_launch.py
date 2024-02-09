#!/usr/bin/env python3

import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='odometry', executable='wheel_odom', name='odometry'),
        Node(
            package='detection', executable='detection', name='detection'),
        Node(
            package='display_markers', executable='display_markers', name='display_markers'),
        Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
            '--child-frame-id', 'odom', '--frame-id', 'map']),
    ])
