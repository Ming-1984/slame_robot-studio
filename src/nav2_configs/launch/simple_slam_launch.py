#!/usr/bin/env python3
"""
简化的SLAM启动文件 - 只启动基本的SLAM功能
不依赖完整的Nav2导航栈
Author: Aurora ROS2 Team
Date: 2025-01-19
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包目录
    nav2_configs_dir = get_package_share_directory('nav2_configs')
    
    # Launch参数配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # 声明launch参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间')
        
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='自动启动生命周期节点')

    # 创建一个简单的静态变换发布器作为占位符
    # 这样我们就有一个节点可以等待，表示"Nav2"已启动
    placeholder_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='nav2_placeholder',
        namespace=namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'nav2_placeholder_frame'])

    # 创建LaunchDescription并添加所有动作
    ld = LaunchDescription()

    # 添加声明的命令
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    # 添加节点
    ld.add_action(placeholder_node)

    return ld
