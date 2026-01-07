#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('acamana'))
    urdf_file_name = 'acamana_ignition.urdf.xacro'  # 使用ignition专用的URDF
    
    # Paths
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'))

    # Add actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld 