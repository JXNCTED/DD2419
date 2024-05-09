#!/usr/bin/env python3

import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='controller', executable='approach_action_server', name='approach_action_server'),
        Node(
            package='controller', executable='arm_controller', name='arm_controller'),
        Node(
            package='controller', executable='dummy_explorer_action_server', name='dummy_explorer_action_server'),
        Node(
            package='controller', executable='finetune_object_action_server', name='finetune_object_action_server'),
        Node(
            package='controller', executable='pursuit_action_server', name='pursuit_action_server'),
        Node(
            package='talking', executable='talking', name='talking'),
    ])
