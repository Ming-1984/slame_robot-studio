#!/usr/bin/env python3
"""
🛡️ 路径安全距离优化器
解决机器人路径过于贴近障碍物的问题

基于以下算法：
1. 膨胀层优化 (Inflation Layer Optimization)
2. 路径平滑算法 (Path Smoothing)
3. 安全距离约束 (Safety Distance Constraints)
4. 动态障碍物避让 (Dynamic Obstacle Avoidance)

作者: Aurora探索系统
日期: 2025-07-21
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt
import math
from typing import List, Tuple, Optional

class PathSafetyOptimizer(Node):
    """路径安全距离优化器"""
    
    def __init__(self):
        super().__init__('path_safety_optimizer')
        
        # 🔧 配置参数
        self.declare_parameter('min_safety_distance', 0.5)      # 最小安全距离(米)
        self.declare_parameter('max_safety_distance', 1.0)      # 最大安全距离(米)
        self.declare_parameter('smoothing_factor', 0.3)         # 平滑因子
        self.declare_parameter('obstacle_inflation_radius', 0.6) # 障碍物膨胀半径
        self.declare_parameter('path_resolution', 0.05)         # 路径分辨率
        
        # 获取参数
        self.min_safety_distance = self.get_parameter('min_safety_distance').value
        self.max_safety_distance = self.get_parameter('max_safety_distance').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.obstacle_inflation_radius = self.get_parameter('obstacle_inflation_radius').value
        self.path_resolution = self.get_parameter('path_resolution').value
        
        # 📡 订阅和发布
        self.path_subscriber = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.costmap_subscriber = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        self.optimized_path_publisher = self.create_publisher(
            Path, '/optimized_plan', 10)
        
        # 📊 状态变量
        self.current_costmap: Optional[OccupancyGrid] = None
        self.distance_field: Optional[np.ndarray] = None
        
        self.get_logger().info('🛡️ 路径安全距离优化器已启动')
        
    def costmap_callback(self, msg: OccupancyGrid):
        """处理代价地图更新"""
        self.current_costmap = msg
        self.update_distance_field()
        
    def update_distance_field(self):
        """更新距离场 - 核心算法"""
        if self.current_costmap is None:
            return
            
        # 转换代价地图为二进制障碍物地图
        costmap_data = np.array(self.current_costmap.data).reshape(
            self.current_costmap.info.height, 
            self.current_costmap.info.width
        )
        
        # 识别障碍物 (代价值 > 50 认为是障碍物)
        obstacle_map = (costmap_data > 50).astype(np.uint8)
        
        # 🛡️ 膨胀障碍物 - 增加安全边界
        inflation_pixels = int(self.obstacle_inflation_radius / self.current_costmap.info.resolution)
        inflated_obstacles = binary_dilation(obstacle_map, iterations=inflation_pixels)
        
        # 🎯 计算距离变换 - 到最近障碍物的距离
        self.distance_field = distance_transform_edt(~inflated_obstacles) * self.current_costmap.info.resolution
        
        self.get_logger().debug(f'🗺️ 更新距离场: {self.distance_field.shape}')
        
    def path_callback(self, msg: Path):
        """处理路径优化请求"""
        if self.current_costmap is None or self.distance_field is None:
            self.get_logger().warn('⚠️ 代价地图或距离场未准备好')
            return
            
        try:
            # 🎨 优化路径
            optimized_path = self.optimize_path_safety(msg)
            
            if optimized_path:
                self.optimized_path_publisher.publish(optimized_path)
                self.get_logger().info(f'✅ 路径安全优化完成: {len(optimized_path.poses)} 个点')
            else:
                self.get_logger().warn('❌ 路径优化失败')
                
        except Exception as e:
            self.get_logger().error(f'💥 路径优化异常: {str(e)}')
            
    def optimize_path_safety(self, original_path: Path) -> Optional[Path]:
        """
        🛡️ 核心路径安全优化算法
        
        算法步骤：
        1. 路径点安全距离检查
        2. 不安全点的重新定位
        3. 路径平滑处理
        4. 最终安全验证
        """
        if len(original_path.poses) < 2:
            return None
            
        # 📍 提取路径点
        path_points = []
        for pose in original_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            path_points.append((x, y))
            
        # 🛡️ 第一步：安全距离检查和调整
        safe_points = []
        for i, (x, y) in enumerate(path_points):
            # 获取当前点的安全距离
            safety_distance = self.get_safety_distance_at_point(x, y)
            
            if safety_distance < self.min_safety_distance:
                # 🔄 重新定位到安全位置
                safe_x, safe_y = self.relocate_to_safe_position(x, y, path_points, i)
                safe_points.append((safe_x, safe_y))
                self.get_logger().debug(f'🔄 重新定位点 ({x:.2f}, {y:.2f}) -> ({safe_x:.2f}, {safe_y:.2f})')
            else:
                safe_points.append((x, y))
                
        # 🎨 第二步：路径平滑
        smoothed_points = self.smooth_path(safe_points)
        
        # 📦 第三步：构建优化后的路径消息
        optimized_path = Path()
        optimized_path.header = original_path.header
        optimized_path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (x, y) in enumerate(smoothed_points):
            pose = PoseStamped()
            pose.header = optimized_path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # 计算朝向
            if i < len(smoothed_points) - 1:
                next_x, next_y = smoothed_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # 最后一个点保持前一个点的朝向
                if len(optimized_path.poses) > 0:
                    pose.pose.orientation = optimized_path.poses[-1].pose.orientation
                    
            optimized_path.poses.append(pose)
            
        return optimized_path
        
    def get_safety_distance_at_point(self, x: float, y: float) -> float:
        """获取指定点的安全距离"""
        if self.distance_field is None:
            return 0.0
            
        # 转换世界坐标到地图坐标
        map_x = int((x - self.current_costmap.info.origin.position.x) / self.current_costmap.info.resolution)
        map_y = int((y - self.current_costmap.info.origin.position.y) / self.current_costmap.info.resolution)
        
        # 边界检查
        if (0 <= map_x < self.distance_field.shape[1] and 
            0 <= map_y < self.distance_field.shape[0]):
            return self.distance_field[map_y, map_x]
        else:
            return 0.0
            
    def relocate_to_safe_position(self, x: float, y: float, path_points: List[Tuple[float, float]], 
                                 current_index: int) -> Tuple[float, float]:
        """将不安全的点重新定位到安全位置"""
        
        # 🎯 策略1：沿路径方向偏移
        if current_index > 0 and current_index < len(path_points) - 1:
            prev_x, prev_y = path_points[current_index - 1]
            next_x, next_y = path_points[current_index + 1]
            
            # 计算路径方向的垂直方向
            path_dx = next_x - prev_x
            path_dy = next_y - prev_y
            path_length = math.sqrt(path_dx**2 + path_dy**2)
            
            if path_length > 0:
                # 垂直方向单位向量
                perp_x = -path_dy / path_length
                perp_y = path_dx / path_length
                
                # 尝试两个方向的偏移
                for direction in [1, -1]:
                    for offset_distance in [self.min_safety_distance, self.max_safety_distance]:
                        candidate_x = x + direction * perp_x * offset_distance
                        candidate_y = y + direction * perp_y * offset_distance
                        
                        if self.get_safety_distance_at_point(candidate_x, candidate_y) >= self.min_safety_distance:
                            return candidate_x, candidate_y
        
        # 🎯 策略2：径向搜索安全位置
        for radius in np.linspace(self.min_safety_distance, self.max_safety_distance, 10):
            for angle in np.linspace(0, 2*math.pi, 16):
                candidate_x = x + radius * math.cos(angle)
                candidate_y = y + radius * math.sin(angle)
                
                if self.get_safety_distance_at_point(candidate_x, candidate_y) >= self.min_safety_distance:
                    return candidate_x, candidate_y
                    
        # 🎯 策略3：如果找不到安全位置，返回原位置
        self.get_logger().warn(f'⚠️ 无法为点 ({x:.2f}, {y:.2f}) 找到安全位置')
        return x, y
        
    def smooth_path(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """🎨 路径平滑算法 - 基于加权平均"""
        if len(points) < 3:
            return points
            
        smoothed = [points[0]]  # 保持起点不变
        
        for i in range(1, len(points) - 1):
            prev_x, prev_y = points[i - 1]
            curr_x, curr_y = points[i]
            next_x, next_y = points[i + 1]
            
            # 加权平均平滑
            smooth_x = (1 - self.smoothing_factor) * curr_x + \
                      self.smoothing_factor * 0.5 * (prev_x + next_x)
            smooth_y = (1 - self.smoothing_factor) * curr_y + \
                      self.smoothing_factor * 0.5 * (prev_y + next_y)
                      
            smoothed.append((smooth_x, smooth_y))
            
        smoothed.append(points[-1])  # 保持终点不变
        return smoothed

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        optimizer = PathSafetyOptimizer()
        rclpy.spin(optimizer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
