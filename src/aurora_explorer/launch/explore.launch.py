#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import TimerAction

def generate_launch_description():
    # 声明参数
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1',
        description='Aurora雷达的IP地址'
    )
    
    min_frontier_size_arg = DeclareLaunchArgument(
        'min_frontier_size',
        default_value='10',
        description='最小Frontier区域大小'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.3',
        description='机器人半径(m)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 获取配置
    ip_address = LaunchConfiguration('ip_address')
    min_frontier_size = LaunchConfiguration('min_frontier_size')
    robot_radius = LaunchConfiguration('robot_radius')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 启动Aurora雷达节点
    slamware_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slamware_ros_sdk'),
                'launch',
                'slamware_ros_sdk_server_node.py'
            ])
        ]),
        launch_arguments={
            'ip_address': ip_address,
            'map_topic': 'map',
            'map_pub_period': '0.1',
            'scan_pub_period': '0.05',
            'robot_pose_pub_period': '0.05',
            'map_frame': 'map',
            'robot_frame': 'base_link'
        }.items()
    )
    
    # 启动Nav2导航堆栈
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_configs'),
                'launch',
                'nav2_ackermann_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )
    
    # 等待一段时间后再启动依赖于雷达和导航堆栈的节点
    # 使用TimerAction来确保先启动雷达和导航，然后再启动探索节点
    explorer_timer = TimerAction(
        period=5.0,  # 等待5秒，确保雷达和导航已经启动
        actions=[
            # 启动探索节点
            Node(
                package='aurora_explorer',
                executable='explore_node',
                name='aurora_explorer',
                parameters=[{
                    'min_frontier_size': min_frontier_size,
                    'robot_radius': robot_radius,
                    'frontier_detection_interval': 1.0  # 优化：从5.0减少到1.0秒，与脚本参数保持一致
                }],
                output='screen'
            )
        ]
    )
    
    # 启动可视化
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('aurora_explorer'),
            'config',
            'explore.rviz'
        ])]
    )
    
    # 启动地图保存节点
    map_saver_node = Node(
        package='aurora_explorer',
        executable='map_saver',
        name='map_saver',
        parameters=[{
            'save_path': '~/maps',
            'save_interval': 30.0,
            'auto_save': True
        }],
        output='screen'
    )
    
    # 创建启动描述
    return LaunchDescription([
        ip_address_arg,
        min_frontier_size_arg,
        robot_radius_arg,
        use_sim_time_arg,
        slamware_sdk_launch,
        nav2_launch,
        # 探索节点通过计时器延迟启动，确保依赖项已启动
        explorer_timer,
        map_saver_node,
        rviz_launch
    ]) 
