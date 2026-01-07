#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_path = os.path.join(get_package_share_directory('acamana'))
    
    # Paths
    rviz_config = os.path.join(pkg_path, 'config', 'acamana.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Include robot publishers launch
    robot_publishers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'robot_publishers.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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

    # Joint State Publisher GUI
    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'))

    # Add actions with timing
    ld.add_action(robot_publishers_launch)
    ld.add_action(static_tf_cmd)
    ld.add_action(TimerAction(period=2.0, actions=[joint_state_publisher_gui_cmd]))
    ld.add_action(TimerAction(period=3.0, actions=[rviz_cmd]))

    return ld 