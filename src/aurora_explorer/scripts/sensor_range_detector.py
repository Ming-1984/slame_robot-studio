#!/usr/bin/env python3
"""
🔍 传感器探测范围检测器
基于激光雷达的目标点可见性和探测范围计算模块

核心功能：
1. 计算目标点是否在传感器探测范围内
2. 检测视线遮挡（Line of Sight）
3. 评估目标点的可见性和可达性
4. 支持360度激光雷达和有限视野传感器

算法参考：
- "An Integrated Approach to Goal Selection in Mobile Robot Exploration" (MDPI Sensors 2019)
- Visibility-based frontier detection algorithms
- Ray-casting for occlusion detection

作者: Aurora探索系统
日期: 2025-07-21
"""

import math
import numpy as np
from typing import Tuple, List, Optional, Dict
from dataclasses import dataclass
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node

@dataclass
class SensorConfig:
    """传感器配置参数"""
    max_range: float = 10.0          # 最大探测距离(米)
    min_range: float = 0.1           # 最小探测距离(米)
    field_of_view: float = 360.0     # 视野角度(度)，360表示全向
    angular_resolution: float = 1.0  # 角度分辨率(度)
    range_accuracy: float = 0.05     # 距离精度(米)

@dataclass
class VisibilityResult:
    """可见性检测结果"""
    is_visible: bool = False         # 是否可见
    distance: float = 0.0            # 距离
    angle: float = 0.0               # 角度(弧度)
    occlusion_ratio: float = 0.0     # 遮挡比例(0-1)
    clear_path_ratio: float = 0.0    # 清晰路径比例(0-1)
    confidence: float = 0.0          # 可见性置信度(0-1)

class SensorRangeDetector:
    """传感器探测范围检测器"""
    
    def __init__(self, sensor_config: SensorConfig = None):
        """
        初始化传感器探测范围检测器
        
        Args:
            sensor_config: 传感器配置参数
        """
        self.config = sensor_config or SensorConfig()
        
        # 预计算角度相关参数
        self.fov_rad = math.radians(self.config.field_of_view)
        self.angular_res_rad = math.radians(self.config.angular_resolution)
        
        # 射线投射缓存
        self.ray_cache: Dict[Tuple[int, int, int, int], List[Tuple[int, int]]] = {}
        
    def is_target_in_sensor_range(self, 
                                  robot_pos: Tuple[float, float], 
                                  robot_yaw: float,
                                  target_pos: Tuple[float, float],
                                  costmap: OccupancyGrid = None) -> VisibilityResult:
        """
        🎯 核心方法：检测目标点是否在传感器探测范围内
        
        Args:
            robot_pos: 机器人位置 (x, y)
            robot_yaw: 机器人朝向角度(弧度)
            target_pos: 目标点位置 (x, y)
            costmap: 代价地图(用于遮挡检测)
            
        Returns:
            VisibilityResult: 详细的可见性检测结果
        """
        result = VisibilityResult()
        
        # 1. 计算距离
        dx = target_pos[0] - robot_pos[0]
        dy = target_pos[1] - robot_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        result.distance = distance
        
        # 2. 距离检查
        if distance < self.config.min_range or distance > self.config.max_range:
            return result
            
        # 3. 计算角度
        target_angle = math.atan2(dy, dx)
        result.angle = target_angle
        
        # 4. 视野角度检查
        if not self._is_within_field_of_view(robot_yaw, target_angle):
            return result
            
        # 5. 遮挡检测
        if costmap is not None:
            occlusion_result = self._check_line_of_sight(
                robot_pos, target_pos, costmap)
            result.occlusion_ratio = occlusion_result['occlusion_ratio']
            result.clear_path_ratio = occlusion_result['clear_path_ratio']
        else:
            result.clear_path_ratio = 1.0
            
        # 6. 综合可见性判断
        result.is_visible = self._evaluate_visibility(result)
        result.confidence = self._calculate_confidence(result)
        
        return result
        
    def _is_within_field_of_view(self, robot_yaw: float, target_angle: float) -> bool:
        """检查目标是否在视野范围内"""
        if self.config.field_of_view >= 360.0:
            return True  # 全向传感器
            
        # 计算角度差
        angle_diff = self._normalize_angle(target_angle - robot_yaw)
        half_fov = self.fov_rad / 2.0
        
        return abs(angle_diff) <= half_fov
        
    def _check_line_of_sight(self, 
                           start_pos: Tuple[float, float], 
                           end_pos: Tuple[float, float],
                           costmap: OccupancyGrid) -> Dict:
        """
        🔍 视线遮挡检测 - 使用改进的Bresenham射线投射算法
        
        Args:
            start_pos: 起始位置
            end_pos: 目标位置
            costmap: 代价地图
            
        Returns:
            Dict: 包含遮挡信息的字典
        """
        # 转换为地图坐标
        start_map = self._world_to_map(start_pos, costmap)
        end_map = self._world_to_map(end_pos, costmap)
        
        if start_map is None or end_map is None:
            return {'occlusion_ratio': 1.0, 'clear_path_ratio': 0.0}
            
        # 获取射线路径
        ray_points = self._get_ray_points(start_map, end_map)
        
        if not ray_points:
            return {'occlusion_ratio': 1.0, 'clear_path_ratio': 0.0}
            
        # 检查路径上的障碍物
        total_points = len(ray_points)
        obstacle_points = 0
        unknown_points = 0
        
        for map_x, map_y in ray_points:
            if (0 <= map_x < costmap.info.width and 
                0 <= map_y < costmap.info.height):
                
                cell_index = map_y * costmap.info.width + map_x
                if cell_index < len(costmap.data):
                    cell_value = costmap.data[cell_index]
                    
                    if cell_value > 50:  # 障碍物
                        obstacle_points += 1
                    elif cell_value == -1:  # 未知区域
                        unknown_points += 1
                        
        # 计算遮挡比例
        occlusion_ratio = obstacle_points / total_points if total_points > 0 else 1.0
        clear_ratio = (total_points - obstacle_points - unknown_points) / total_points if total_points > 0 else 0.0
        
        return {
            'occlusion_ratio': occlusion_ratio,
            'clear_path_ratio': clear_ratio,
            'unknown_ratio': unknown_points / total_points if total_points > 0 else 0.0
        }
        
    def _get_ray_points(self, start: Tuple[int, int], end: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        获取射线路径上的所有点 - 使用Bresenham算法
        
        Args:
            start: 起始点(地图坐标)
            end: 终点(地图坐标)
            
        Returns:
            List[Tuple[int, int]]: 路径上的所有点
        """
        # 检查缓存
        cache_key = (start[0], start[1], end[0], end[1])
        if cache_key in self.ray_cache:
            return self.ray_cache[cache_key]
            
        points = []
        x0, y0 = start
        x1, y1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        x_step = 1 if x0 < x1 else -1
        y_step = 1 if y0 < y1 else -1
        
        error = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            error2 = 2 * error
            
            if error2 > -dy:
                error -= dy
                x += x_step
                
            if error2 < dx:
                error += dx
                y += y_step
                
        # 缓存结果(限制缓存大小)
        if len(self.ray_cache) < 1000:
            self.ray_cache[cache_key] = points
            
        return points
        
    def _world_to_map(self, world_pos: Tuple[float, float], costmap: OccupancyGrid) -> Optional[Tuple[int, int]]:
        """世界坐标转地图坐标"""
        try:
            map_x = int((world_pos[0] - costmap.info.origin.position.x) / costmap.info.resolution)
            map_y = int((world_pos[1] - costmap.info.origin.position.y) / costmap.info.resolution)
            
            if (0 <= map_x < costmap.info.width and 0 <= map_y < costmap.info.height):
                return (map_x, map_y)
            else:
                return None
        except Exception:
            return None
            
    def _evaluate_visibility(self, result: VisibilityResult) -> bool:
        """
        评估目标点的可见性
        
        Args:
            result: 可见性检测结果
            
        Returns:
            bool: 是否可见
        """
        # 基本条件：距离在范围内
        if (result.distance < self.config.min_range or 
            result.distance > self.config.max_range):
            return False
            
        # 遮挡检查：清晰路径比例需要足够高
        min_clear_ratio = 0.7  # 至少70%的路径是清晰的
        if result.clear_path_ratio < min_clear_ratio:
            return False
            
        # 遮挡比例检查：障碍物遮挡不能太多
        max_occlusion_ratio = 0.3  # 最多30%的遮挡
        if result.occlusion_ratio > max_occlusion_ratio:
            return False
            
        return True
        
    def _calculate_confidence(self, result: VisibilityResult) -> float:
        """
        计算可见性置信度
        
        Args:
            result: 可见性检测结果
            
        Returns:
            float: 置信度(0-1)
        """
        if not result.is_visible:
            return 0.0
            
        # 距离因子：距离越近置信度越高
        distance_factor = max(0.0, 1.0 - result.distance / self.config.max_range)
        
        # 清晰度因子：路径越清晰置信度越高
        clarity_factor = result.clear_path_ratio
        
        # 遮挡因子：遮挡越少置信度越高
        occlusion_factor = 1.0 - result.occlusion_ratio
        
        # 综合置信度
        confidence = (distance_factor * 0.3 + 
                     clarity_factor * 0.4 + 
                     occlusion_factor * 0.3)
                     
        return min(1.0, max(0.0, confidence))
        
    def _normalize_angle(self, angle: float) -> float:
        """角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def get_sensor_coverage_area(self, 
                                robot_pos: Tuple[float, float], 
                                robot_yaw: float,
                                costmap: OccupancyGrid = None) -> List[Tuple[float, float]]:
        """
        获取传感器覆盖区域的边界点
        
        Args:
            robot_pos: 机器人位置
            robot_yaw: 机器人朝向
            costmap: 代价地图
            
        Returns:
            List[Tuple[float, float]]: 覆盖区域边界点
        """
        boundary_points = []
        
        # 确定角度范围
        if self.config.field_of_view >= 360.0:
            start_angle = 0.0
            end_angle = 2 * math.pi
        else:
            half_fov = self.fov_rad / 2.0
            start_angle = robot_yaw - half_fov
            end_angle = robot_yaw + half_fov
            
        # 生成边界点
        num_rays = int(self.config.field_of_view / self.config.angular_resolution)
        for i in range(num_rays + 1):
            if num_rays > 0:
                angle = start_angle + (end_angle - start_angle) * i / num_rays
            else:
                angle = robot_yaw
                
            # 计算射线终点
            end_x = robot_pos[0] + self.config.max_range * math.cos(angle)
            end_y = robot_pos[1] + self.config.max_range * math.sin(angle)
            
            # 如果有地图，检查实际可达距离
            if costmap is not None:
                actual_range = self._get_actual_range(robot_pos, angle, costmap)
                end_x = robot_pos[0] + actual_range * math.cos(angle)
                end_y = robot_pos[1] + actual_range * math.sin(angle)
                
            boundary_points.append((end_x, end_y))
            
        return boundary_points
        
    def _get_actual_range(self, 
                         robot_pos: Tuple[float, float], 
                         angle: float, 
                         costmap: OccupancyGrid) -> float:
        """获取指定角度上的实际探测距离"""
        step_size = costmap.info.resolution
        current_range = self.config.min_range
        
        while current_range <= self.config.max_range:
            test_x = robot_pos[0] + current_range * math.cos(angle)
            test_y = robot_pos[1] + current_range * math.sin(angle)
            
            map_pos = self._world_to_map((test_x, test_y), costmap)
            if map_pos is None:
                break
                
            map_x, map_y = map_pos
            cell_index = map_y * costmap.info.width + map_x
            
            if cell_index < len(costmap.data):
                if costmap.data[cell_index] > 50:  # 遇到障碍物
                    break
                    
            current_range += step_size
            
        return min(current_range, self.config.max_range)
