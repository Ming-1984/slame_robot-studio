#!/usr/bin/env python3
"""
Acamana机器人静态TF发布器
只发布关节和传感器的静态TF，不发布odom和map相关的动态TF
避免与Aurora SDK的TF冲突
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_path = os.path.join(get_package_share_directory('acamana'))
    urdf_file_name = 'acamana.urdf.xacro'
    
    # URDF路径
    urdf = os.path.join(pkg_path, 'urdf', urdf_file_name)

    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 机器人状态发布器 - 只发布静态TF
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf]),
            'use_sim_time': use_sim_time,
            # 重要：仅发布关节状态，不发布 TF，避免与Aurora 冲突
            'publish_frequency': 50.0,
            'publish_tf': True,
        }],
        output='screen'
    )

    # 关节状态发布器
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'rate': 50  # 提高关节状态发布频率
        }],
        output='screen'
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加启动参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false', 
        description='使用仿真时间'
    ))

    # 添加节点
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld 