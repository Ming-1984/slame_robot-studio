#!/usr/bin/env python3

"""
简单地图保存器 - 直接使用ROS2 Python API
用于解决map_saver_cli的QoS兼容性问题
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
import os
import sys
import argparse
from datetime import datetime

class SimpleMapSaver(Node):
    def __init__(self):
        super().__init__('simple_map_saver')
        
        # 创建QoS配置，兼容不同的发布者
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE  # 改为VOLATILE以匹配发布者
        )
        
        # 订阅地图话题
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/slamware_ros_sdk_server_node/map',
            self.map_callback,
            qos_profile
        )
        
        self.latest_map = None
        self.get_logger().info('简单地图保存器已启动，等待地图数据...')
    
    def map_callback(self, msg):
        self.latest_map = msg
        self.get_logger().info(f'收到地图数据: {msg.info.width}x{msg.info.height}, 分辨率: {msg.info.resolution}')
    
    def save_map(self, filename):
        """保存地图到指定文件"""
        if self.latest_map is None:
            self.get_logger().error('没有可用的地图数据')
            return False
        
        try:
            # 扩展用户路径
            filename = os.path.expanduser(filename)
            
            # 确保目录存在
            directory = os.path.dirname(filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)
            
            # 生成文件名
            pgm_file = f"{filename}.pgm"
            yaml_file = f"{filename}.yaml"
            
            # 保存PGM文件
            self.save_pgm(self.latest_map, pgm_file)
            
            # 保存YAML文件
            self.save_yaml(self.latest_map, yaml_file, os.path.basename(pgm_file))
            
            self.get_logger().info(f'地图已保存: {yaml_file}, {pgm_file}')
            print(f'✅ 地图已保存: {yaml_file}, {pgm_file}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'保存地图失败: {str(e)}')
            print(f'❌ 保存地图失败: {str(e)}')
            return False
    
    def save_pgm(self, occupancy_grid, filename):
        """保存PGM格式的地图图像"""
        # 提取地图数据
        map_data = np.array(occupancy_grid.data, dtype=np.int8)
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # 重新整形为2D数组
        map_2d = map_data.reshape((height, width))
        
        # 转换为PGM格式
        # -1 (未知) -> 205 (灰色)
        # 0 (自由) -> 254 (白色)  
        # 100 (占用) -> 0 (黑色)
        pgm_data = np.zeros_like(map_2d, dtype=np.uint8)
        pgm_data[map_2d == -1] = 205  # 未知
        pgm_data[map_2d == 0] = 254   # 自由
        pgm_data[map_2d >= 65] = 0    # 占用 (阈值65，兼容不同的占用值)
        
        # 写入PGM文件
        with open(filename, 'wb') as f:
            # PGM头部
            header = f"P5\n{width} {height}\n255\n".encode()
            f.write(header)
            # 图像数据
            pgm_data.tobytes()
            f.write(pgm_data.tobytes())
    
    def save_yaml(self, occupancy_grid, filename, pgm_filename):
        """保存YAML格式的地图元数据"""
        yaml_data = {
            'image': pgm_filename,
            'resolution': float(occupancy_grid.info.resolution),
            'origin': [
                float(occupancy_grid.info.origin.position.x),
                float(occupancy_grid.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(filename, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

def main():
    parser = argparse.ArgumentParser(description='简单地图保存器')
    parser.add_argument('filename', help='保存的文件名（不含扩展名）')
    parser.add_argument('--timeout', type=int, default=10, help='等待地图数据的超时时间（秒）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = SimpleMapSaver()
        
        # 等待地图数据
        import time
        start_time = time.time()
        print(f'等待地图数据，超时时间: {args.timeout}秒...')
        
        while node.latest_map is None and (time.time() - start_time) < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        if node.latest_map is None:
            print(f'❌ 错误：在{args.timeout}秒内未收到地图数据')
            print('💡 请确保：')
            print('   1. /map 话题正在发布')
            print('   2. SLAM系统正在运行')
            print('   3. 机器人已开始建图')
            return False
        
        # 保存地图
        success = node.save_map(args.filename)
        
        node.destroy_node()
        rclpy.shutdown()
        
        return success
        
    except KeyboardInterrupt:
        print('\n用户中断')
        return False
    except Exception as e:
        print(f'❌ 发生错误: {str(e)}')
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
