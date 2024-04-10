#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='odometry', executable='wheel_odom', name='wheel_odom'),
        Node(
            package='odometry', executable='lidar_compensator', name='lidar_compensator',
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/camera/depth/color/points']),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/realsense-scan'])],
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='odometry', executable='filter_odom', name='filter_odom'),
        Node(
            package='mapping', executable='mapping', name='mapping',
        ),
        Node(
            package='workspace', executable='workspace', name='workspace',
        ),
        Node(
            package='display_markers', executable='display_markers', name='display_markers'),
        # Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
        #     '--child-frame-id', 'odom', '--frame-id', 'map']),

    ])
