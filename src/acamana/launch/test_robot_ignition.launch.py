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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Include robot publishers launch for ignition
    robot_publishers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'robot_publishers_ignition.launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
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

    # Ignition Gazebo launch
    ignition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')
        ]),
        launch_arguments={'ign_args': '-v 4 -r empty.sdf'}.items()
    )

    # Spawn robot in Ignition Gazebo with improved parameters
    spawn_robot_cmd = Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'acamana',
            '-allow_renaming', 'true',
            '-z', '0.1'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'))

    # Add actions with timing
    ld.add_action(robot_publishers_launch)
    ld.add_action(TimerAction(period=1.0, actions=[ignition_launch]))
    ld.add_action(TimerAction(period=4.0, actions=[spawn_robot_cmd]))  # 延迟spawn
    ld.add_action(TimerAction(period=2.0, actions=[joint_state_publisher_gui_cmd]))
    ld.add_action(TimerAction(period=5.0, actions=[rviz_cmd]))

    return ld 