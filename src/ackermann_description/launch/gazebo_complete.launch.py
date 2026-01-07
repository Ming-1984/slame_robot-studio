#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_path = os.path.join(get_package_share_directory('ackermann_description'))
    acamana_pkg_path = os.path.join(get_package_share_directory('acamana'))
    
    # Paths
    urdf_file = os.path.join(acamana_pkg_path, 'urdf', 'acamana.urdf.xacro')
    rviz_config = os.path.join(pkg_path, 'config', 'display.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='empty.world')
    
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

    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
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

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_name,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'acamana',
            '-z', '0.1'
        ],
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
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', 
                                       description='Use simulation time'))
    ld.add_action(DeclareLaunchArgument('world_name', default_value='empty.world',
                                       description='Gazebo world file'))

    # Add actions with timing
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(TimerAction(period=3.0, actions=[spawn_robot_cmd]))
    ld.add_action(TimerAction(period=2.0, actions=[joint_state_publisher_gui_cmd]))
    ld.add_action(TimerAction(period=4.0, actions=[rviz_cmd]))

    return ld 