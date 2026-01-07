#!/usr/bin/env python3
"""
Aurora激光雷达 + Acamana阿克曼底盘 SLAM模式导航启动文件
专为SLAM建图和导航同时进行而设计
Author: Aurora ROS2 Team
Date: 2025-01-14
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包目录
    nav2_configs_dir = get_package_share_directory('nav2_configs')
    
    # Launch参数配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # 声明launch参数
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='顶级命名空间')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='使用仿真时间')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='True',
        description='自动启动nav2堆栈')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_configs_dir, 'params', 'ackermann_nav2_params.yaml'),
        description='Nav2参数文件完整路径')
        
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='是否在节点崩溃时自动重启')
        
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别')

    # 直接使用参数文件，不进行重写
    configured_params = params_file

    # 创建导航节点组
    nav_nodes = GroupAction([
        # 控制器服务器
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),

        # 路径规划服务器
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),

        # 路径平滑服务器
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),

        # 行为服务器
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),

        # 行为树导航器
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level]),

        # 生命周期管理器 - 仅管理导航节点，不包括AMCL
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'controller_server',
                    'planner_server', 
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]),
    ])

    # 阿克曼底盘接口节点 - 暂时注释掉，使用独立的Python脚本
    # ackermann_interface_cmd = Node(
    #     package='nav2_configs',
    #     executable='ackermann_cmd_vel_bridge.py',
    #     name='ackermann_cmd_vel_bridge',
    #     namespace=namespace,
    #     output='screen',
    #     respawn=use_respawn,
    #     parameters=[configured_params],
    #     remappings=[
    #         ('/cmd_vel_nav', '/cmd_vel'),  # Nav2的速度命令重映射到标准cmd_vel
    #     ],
    #     arguments=['--ros-args', '--log-level', log_level])

    # 创建launch描述
    ld = LaunchDescription()

    # 添加声明参数命令
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 添加节点启动命令
    ld.add_action(nav_nodes)
    # ld.add_action(ackermann_interface_cmd)  # 暂时注释掉

    return ld 