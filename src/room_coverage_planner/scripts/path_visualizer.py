#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
from datetime import datetime

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # 参数
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('save_path', '~/path_images')
        self.declare_parameter('auto_save', True)
        self.declare_parameter('path_topic', 'coverage_path')
        
        self.save_path = self.get_parameter('save_path').value
        self.auto_save = self.get_parameter('auto_save').value
        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        
        # 扩展用户路径 (如 ~/path_images)
        self.save_path = os.path.expanduser(self.save_path)
        
        # 确保目录存在
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        
        # 变量初始化
        self.map_data = None
        self.path_data = None
        self.bridge = CvBridge()
        
        # 订阅地图话题
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10)
            
        # 订阅路径话题
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10)
            
        # 发布可视化图像
        self.vis_pub = self.create_publisher(
            Image,
            'path_visualization',
            10)
            
        # 定时器触发可视化
        self.timer = self.create_timer(5.0, self.visualize_timer_callback)
        
        self.get_logger().info('路径可视化器已启动')
    
    def map_callback(self, msg):
        self.map_data = msg
        
    def path_callback(self, msg):
        self.path_data = msg
        if self.auto_save and self.map_data is not None:
            self.visualize_path(save=True)
            
    def visualize_timer_callback(self):
        if self.map_data is not None and self.path_data is not None:
            self.visualize_path(save=False)
            
    def visualize_path(self, save=False):
        if self.map_data is None or self.path_data is None:
            return
            
        # 提取地图数据
        map_data = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
            
        # 创建RGB图像
        rgb_map = np.zeros((map_data.shape[0], map_data.shape[1], 3), dtype=np.uint8)
        
        # 设置颜色：未知=灰色，自由=白色，障碍=黑色
        rgb_map[map_data == -1] = [128, 128, 128]  # 未知 = 灰色
        rgb_map[map_data == 0] = [255, 255, 255]   # 自由 = 白色
        rgb_map[map_data == 100] = [0, 0, 0]       # 障碍 = 黑色
        
        # 提取路径点
        path_points = []
        for pose in self.path_data.poses:
            # 将世界坐标转换为像素坐标
            px = int((pose.pose.position.x - self.map_data.info.origin.position.x) / 
                      self.map_data.info.resolution)
            py = int((pose.pose.position.y - self.map_data.info.origin.position.y) / 
                      self.map_data.info.resolution)
            path_points.append((px, py))
            
        # 在图像上绘制路径
        for i in range(len(path_points) - 1):
            p1 = path_points[i]
            p2 = path_points[i + 1]
            
            # 确保点在图像范围内
            if (0 <= p1[0] < rgb_map.shape[1] and 0 <= p1[1] < rgb_map.shape[0] and
                0 <= p2[0] < rgb_map.shape[1] and 0 <= p2[1] < rgb_map.shape[0]):
                
                # 计算颜色渐变 (从绿到红)
                ratio = i / max(1, len(path_points) - 2)
                color = (
                    int(255 * ratio),    # B
                    int(255 * (1-ratio)), # G
                    0                    # R
                )
                
                cv2.line(rgb_map, p1, p2, color, 2)
        
        # 在路径点上添加标记
        for i, point in enumerate(path_points):
            if 0 <= point[0] < rgb_map.shape[1] and 0 <= point[1] < rgb_map.shape[0]:
                # 点大小与索引成比例
                size = max(3, min(10, 3 + (i // 5)))
                # 使用与线相同的颜色渐变
                ratio = i / max(1, len(path_points) - 1)
                color = (
                    int(255 * ratio),    # B
                    int(255 * (1-ratio)), # G
                    0                    # R
                )
                cv2.circle(rgb_map, point, size, color, -1)
                
                # 添加路径点编号
                if i % 5 == 0:  # 每5个点标记一次
                    cv2.putText(
                        rgb_map, str(i), 
                        (point[0] + 5, point[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1
                    )
        
        # 发布可视化图像
        img_msg = self.bridge.cv2_to_imgmsg(rgb_map, encoding='rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.map_data.header.frame_id
        self.vis_pub.publish(img_msg)
        
        # 保存图像
        if save:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"path_visualization_{timestamp}.png"
            filepath = os.path.join(self.save_path, filename)
            plt.figure(figsize=(12, 10))
            plt.imshow(cv2.cvtColor(rgb_map, cv2.COLOR_BGR2RGB))
            plt.title(f"Coverage Path - {len(path_points)} Waypoints")
            plt.axis('off')
            plt.savefig(filepath, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f'路径可视化已保存至 {filepath}')

def main():
    rclpy.init()
    visualizer = PathVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 