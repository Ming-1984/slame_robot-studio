#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import numpy as np
import os
from datetime import datetime
import yaml

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        # 参数
        self.declare_parameter('save_path', '~/maps')
        self.declare_parameter('save_interval', 30.0)  # 每30秒保存一次地图
        self.declare_parameter('auto_save', True)
        
        self.save_path = self.get_parameter('save_path').value
        self.save_interval = self.get_parameter('save_interval').value
        self.auto_save = self.get_parameter('auto_save').value
        
        # 扩展用户路径 (如 ~/maps)
        self.save_path = os.path.expanduser(self.save_path)
        
        # 确保目录存在
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            
        # 最新地图数据
        self.latest_map = None
        
        # 订阅地图话题
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
            
        # 创建定时器进行自动保存
        if self.auto_save:
            self.timer = self.create_timer(
                self.save_interval,
                self.save_map_timer_callback)
                
        # 创建服务
        self.save_map_srv = self.create_service(
            Trigger,
            'save_map',
            self.save_map_service_callback)
        
        self.get_logger().info(f'地图保存器已启动，将保存到: {self.save_path}，保存间隔: {self.save_interval}秒')
    
    def map_callback(self, msg):
        self.latest_map = msg
    
    def save_map_timer_callback(self):
        if self.latest_map is not None:
            self.save_map()
    
    def save_map_service_callback(self, request, response):
        success = self.save_map()
        response.success = success
        if success:
            response.message = "地图保存成功"
        else:
            response.message = "地图保存失败"
        return response
    
    def save_map(self):
        if self.latest_map is None:
            self.get_logger().warn('没有可用的地图数据')
            return False
            
        try:
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            pgm_filename = f"map_{timestamp}.pgm"
            yaml_filename = f"map_{timestamp}.yaml"
            
            # 使用os.path.normpath确保路径安全
            pgm_path = os.path.normpath(os.path.join(self.save_path, pgm_filename))
            yaml_path = os.path.normpath(os.path.join(self.save_path, yaml_filename))
            
            # 检查路径是否在预期目录下
            if not pgm_path.startswith(self.save_path) or not yaml_path.startswith(self.save_path):
                self.get_logger().error(f'文件路径安全检查失败: {pgm_path}, {yaml_path}')
                return False
            
            # 保存PGM文件 (图像)
            self.save_map_as_pgm(self.latest_map, pgm_path)
            
            # 保存YAML文件 (元数据)
            self.save_map_as_yaml(self.latest_map, yaml_path, pgm_filename)
            
            self.get_logger().info(f'地图已保存为 {pgm_filename} 和 {yaml_filename}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'保存地图时出错: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

    def save_map_with_filename(self, base_filename):
        """使用指定的文件名保存地图"""
        if self.latest_map is None:
            print('没有可用的地图数据')
            return False

        try:
            # 扩展用户路径
            base_filename = os.path.expanduser(base_filename)

            # 确保目录存在
            directory = os.path.dirname(base_filename)
            if directory and not os.path.exists(directory):
                os.makedirs(directory)

            pgm_filename = f"{base_filename}.pgm"
            yaml_filename = f"{base_filename}.yaml"

            # 保存PGM文件 (图像)
            self.save_map_as_pgm(self.latest_map, pgm_filename)

            # 保存YAML文件 (元数据)
            self.save_map_as_yaml(self.latest_map, yaml_filename, os.path.basename(pgm_filename))

            print(f'地图已保存为 {yaml_filename} 和 {pgm_filename}')
            return True

        except Exception as e:
            print(f'保存地图时发生错误: {str(e)}')
            return False

    def save_map_as_pgm(self, occupancy_grid, filename):
        """将OccupancyGrid保存为PGM格式"""
        # 提取地图数据
        map_data = np.array(occupancy_grid.data)
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        
        # 将地图数据重新整形
        map_2d = map_data.reshape((height, width))
        
        # 将-1（未知）转换为灰色 (205)
        # 将0（自由空间）转换为白色 (254)
        # 将100（障碍物）转换为黑色 (0)
        pgm_data = np.zeros_like(map_2d, dtype=np.uint8)
        pgm_data[map_2d == -1] = 205  # 未知区域 = 灰色
        pgm_data[map_2d == 0] = 254   # 自由空间 = 白色
        pgm_data[map_2d == 100] = 0   # 障碍物 = 黑色
        
        # 写入PGM文件
        with open(filename, 'wb') as f:
            # 写入PGM头
            header = f"P5\n{width} {height}\n255\n".encode()
            f.write(header)
            
            # 写入图像数据
            pgm_data.tofile(f)
    
    def save_map_as_yaml(self, occupancy_grid, filename, pgm_filename):
        """保存地图元数据为YAML格式"""
        # 准备YAML数据
        yaml_data = {
            'image': pgm_filename,
            'resolution': occupancy_grid.info.resolution,
            'origin': [
                occupancy_grid.info.origin.position.x,
                occupancy_grid.info.origin.position.y,
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        # 写入YAML文件
        with open(filename, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

def main():
    import argparse

    parser = argparse.ArgumentParser(description='地图保存器')
    parser.add_argument('--save-once', action='store_true', help='保存一次地图后退出')
    parser.add_argument('--filename', type=str, help='保存的文件名（不含扩展名）')
    parser.add_argument('--timeout', type=int, default=10, help='等待地图数据的超时时间（秒）')

    args = parser.parse_args()

    rclpy.init()

    if args.save_once:
        # 一次性保存模式
        node = MapSaver()

        # 等待地图数据
        import time
        start_time = time.time()
        while node.latest_map is None and (time.time() - start_time) < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.latest_map is None:
            print(f"错误：在{args.timeout}秒内未收到地图数据")
            node.destroy_node()
            rclpy.shutdown()
            return False

        # 保存地图
        if args.filename:
            success = node.save_map_with_filename(args.filename)
        else:
            success = node.save_map()

        if success:
            print("地图保存成功")
        else:
            print("地图保存失败")

        node.destroy_node()
        rclpy.shutdown()
        return success
    else:
        # 正常服务模式
        node = MapSaver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    if '--save-once' in sys.argv:
        success = main()
        sys.exit(0 if success else 1)
    else:
        main()