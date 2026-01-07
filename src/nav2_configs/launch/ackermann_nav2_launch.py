#!/usr/bin/env python3
"""
Aurora激光雷达 + Acamana阿克曼底盘 Nav2导航启动文件
专为Jetson Orin Nano平台优化
Author: Aurora ROS2 Team
Date: 2025-01-14
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取包目录
    nav2_configs_dir = get_package_share_directory('nav2_configs')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch参数配置
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
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
        
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', 
        default_value='False',
        description='是否使用组合节点以降低内存占用')
        
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='组合容器名称')
        
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', 
        default_value='False',
        description='是否在节点崩溃时自动重启')
        
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别')

    # 创建重写的参数配置
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'yaml_filename': '',  # 不使用静态地图文件，依赖Aurora SDK
        },
        convert_types=True)

    # 包含Nav2 bringup启动文件
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level
        }.items())

    # 启动阿克曼底盘接口节点
    ackermann_interface_cmd = Node(
        package='nav2_configs',
        executable='ackermann_cmd_vel_bridge.py',
        name='ackermann_cmd_vel_bridge',
        namespace=namespace,
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),  # Nav2的速度命令
            ('/ackermann_cmd', '/cmd_vel'),  # 发送给底盘的速度命令
        ],
        respawn=use_respawn)

    # 静态TF发布器 - 补充Aurora SDK没有发布的变换
    # 注意：主要的TF变换由Aurora SDK负责，这里只发布必要的补充变换
    static_tf_publisher_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        namespace=namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen')

    # 创建launch描述
    ld = LaunchDescription()

    # 添加声明参数命令
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 添加节点启动命令
    ld.add_action(static_tf_publisher_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(ackermann_interface_cmd)

    return ld 