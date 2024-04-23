#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='odometry', executable='wheel_odom', name='wheel_odom'),
        Node(
            package='odometry', executable='lidar_compensator', name='lidar_compensator',
        ),
        # Node(
        #     package='odometry', executable='lidar_landmarker', name='lidar_landmarker', ros_arguments=[
        #         '--log-level', 'warn'
        #     ],
        # ),
        Node(
            package='odometry', executable='filter_odom', name='filter_odom',
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/camera/depth/color/points'),
                        ('scan', '/realsense_scan')],
            parameters=[{
                'target_frame': 'camera_depth_frame',
                'transform_tolerance': 0.01,
                'min_height': -0.05,
                'max_height': 0.0,
                'angle_min': -0.5,  # POV of D435i
                'angle_max': 0.5,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.20,
                'range_max': 1.5,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='mapping', executable='mapping', name='mapping',
        ),
        Node(
            package='workspace', executable='workspace', name='workspace',
        ),
        Node(
            package='detection_ml', executable='detection_ml', name='detection_ml',
        ),
        Node(
            package='detection_ml', executable='category_eval', name='detection_ml',
        ),
        Node(
            package='display_markers', executable='display_markers', name='display_markers'),
        Node(executable='static_transform_publisher', package='tf2_ros', arguments=[
            '--child-frame-id', 'odom', '--frame-id', 'map']),
    ])
