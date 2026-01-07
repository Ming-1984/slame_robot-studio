#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明参数
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1',
        description='Aurora雷达的IP地址'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.3',
        description='车轴距离(m)'
    )
    
    max_steer_angle_arg = DeclareLaunchArgument(
        'max_steer_angle',
        default_value='0.6',
        description='最大转向角(rad)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='最大速度(m/s)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='底盘控制串口设备'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )
    
    # 获取配置
    ip_address = LaunchConfiguration('ip_address')
    wheelbase = LaunchConfiguration('wheelbase')
    max_steer_angle = LaunchConfiguration('max_steer_angle')
    max_speed = LaunchConfiguration('max_speed')
    use_sim_time = LaunchConfiguration('use_sim_time')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    
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
    
    # 启动硬件接口
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hardware_interface'),
                'launch',
                'hardware_interface.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheelbase': wheelbase,
            'max_steer_angle': max_steer_angle,
            'send_frequency': '20.0'
        }.items()
    )
    
    # 启动覆盖路径规划节点
    planner_node = Node(
        package='room_coverage_planner',
        executable='coverage_planner_node',
        name='coverage_planner',
        parameters=[{
            'robot_radius': 0.3,
            'cell_size': 0.2,
            'min_path_points': 10,
            'free_space_threshold': 0,
            'auto_start': False
        }],
        output='screen'
    )
    
    # 启动阿克曼底盘控制节点
    controller_node = Node(
        package='ackermann_controller',
        executable='ackermann_controller_node',
        name='ackermann_controller',
        parameters=[{
            'wheelbase': wheelbase,
            'max_steer_angle': max_steer_angle,
            'max_speed': max_speed,
            'min_speed': 0.1,
            'lookahead_distance': 0.5,
            'goal_tolerance': 0.2,
            'control_frequency': 10.0
        }],
        output='screen'
    )
    
    # 启动可视化
    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('ackermann_controller'),
            'config',
            'control.rviz'
        ])]
    )
    
    # 创建启动描述
    return LaunchDescription([
        ip_address_arg,
        wheelbase_arg,
        max_steer_angle_arg,
        max_speed_arg,
        use_sim_time_arg,
        serial_port_arg,
        baud_rate_arg,
        slamware_sdk_launch,
        nav2_launch,
        hardware_interface_launch,
        planner_node,
        controller_node,
        rviz_launch
    ]) 
