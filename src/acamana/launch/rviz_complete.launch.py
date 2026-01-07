#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('acamana'))
    
    # Paths
    urdf_file = os.path.join(pkg_path, 'urdf', 'acamana.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'config', 'acamana.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Joint State Publisher (without GUI)
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('use_gui', 'false')
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('use_gui', 'true')
    )

    # RViz Node
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform from base_link to base_footprint
    static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', 
                                       description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('use_gui', default_value='true',
                                       description='Use joint state publisher GUI'))

    # Add actions with timing
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(static_tf_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(joint_state_publisher_gui_cmd)
    ld.add_action(TimerAction(period=2.0, actions=[rviz_cmd]))

    return ld 