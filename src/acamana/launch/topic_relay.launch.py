import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取包共享目录
    acamana_dir = get_package_share_directory('acamana')
    
    # 声明参数文件路径的Launch Argument
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(acamana_dir, 'config', 'acamana_params.yaml'),
        description='Path to the Acamana parameters file'
    )
    
    # 启动Topic中继节点
    topic_relay_node = Node(
        package='acamana',
        executable='topic_relay_node.py',
        name='topic_relay_node',
        output='screen',
        parameters=[params_file] # 从YAML文件中加载所有参数
    )
    
    return LaunchDescription([
        declare_params_file_cmd,
        topic_relay_node
    ]) 