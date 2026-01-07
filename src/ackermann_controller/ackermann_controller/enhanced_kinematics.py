#!/usr/bin/env python3
"""
增强的阿克曼运动学模型
实现精确的运动学计算、转弯半径约束检查和路径曲率验证
Author: Acamana-Bot Development Team
Date: 2025-01-15
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path


class EnhancedAckermannKinematics:
    """增强的阿克曼运动学模型"""
    
    def __init__(self, wheelbase: float, max_steer_angle: float, min_turning_radius: float):
        """
        初始化阿克曼运动学模型
        
        Args:
            wheelbase: 轴距 (米)
            max_steer_angle: 最大转向角 (弧度)
            min_turning_radius: 最小转弯半径 (米)
        """
        self.wheelbase = wheelbase
        self.max_steer_angle = max_steer_angle
        self.min_turning_radius = min_turning_radius
        
        # 计算验证最小转弯半径
        calculated_min_radius = self.calculate_turning_radius(max_steer_angle)
        if abs(calculated_min_radius - min_turning_radius) > 0.01:
            print(f"警告: 配置的最小转弯半径({min_turning_radius:.3f}m) "
                  f"与计算值({calculated_min_radius:.3f}m)不匹配")
    
    def calculate_turning_radius(self, steer_angle: float) -> float:
        """
        计算给定转向角的转弯半径
        
        Args:
            steer_angle: 转向角 (弧度)
            
        Returns:
            转弯半径 (米)
        """
        if abs(steer_angle) < 1e-6:
            return float('inf')
        return self.wheelbase / math.tan(abs(steer_angle))
    
    def calculate_steer_angle(self, turning_radius: float) -> float:
        """
        计算达到指定转弯半径所需的转向角
        
        Args:
            turning_radius: 期望的转弯半径 (米)
            
        Returns:
            所需转向角 (弧度)
        """
        if turning_radius == float('inf'):
            return 0.0
        
        if abs(turning_radius) < self.min_turning_radius:
            # 转弯半径太小，返回最大转向角
            return math.copysign(self.max_steer_angle, turning_radius)
        
        steer_angle = math.atan(self.wheelbase / abs(turning_radius))
        return math.copysign(min(steer_angle, self.max_steer_angle), turning_radius)
    
    def calculate_curvature(self, steer_angle: float) -> float:
        """
        计算给定转向角的路径曲率
        
        Args:
            steer_angle: 转向角 (弧度)
            
        Returns:
            路径曲率 (1/米)
        """
        if abs(steer_angle) < 1e-6:
            return 0.0
        return math.tan(steer_angle) / self.wheelbase
    
    def validate_path_curvature(self, path: Path) -> Tuple[bool, List[int]]:
        """
        验证路径是否满足转弯半径约束
        
        Args:
            path: 要验证的路径
            
        Returns:
            (is_valid, invalid_indices): 路径是否有效，无效点的索引列表
        """
        if len(path.poses) < 3:
            return True, []
        
        invalid_indices = []
        
        for i in range(1, len(path.poses) - 1):
            # 计算三点间的曲率
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            p3 = path.poses[i+1].pose.position
            
            curvature = self._calculate_three_point_curvature(p1, p2, p3)
            
            if curvature > 0:  # 有效曲率
                turning_radius = 1.0 / curvature
                if turning_radius < self.min_turning_radius:
                    invalid_indices.append(i)
        
        return len(invalid_indices) == 0, invalid_indices
    
    def _calculate_three_point_curvature(self, p1: Point, p2: Point, p3: Point) -> float:
        """
        计算三点间的曲率
        
        Args:
            p1, p2, p3: 三个连续路径点
            
        Returns:
            曲率值 (1/米)，0表示直线
        """
        # 向量AB和BC
        ab = np.array([p2.x - p1.x, p2.y - p1.y])
        bc = np.array([p3.x - p2.x, p3.y - p2.y])
        
        # 边长
        len_ab = np.linalg.norm(ab)
        len_bc = np.linalg.norm(bc)
        len_ac = np.linalg.norm([p3.x - p1.x, p3.y - p1.y])
        
        if len_ab < 1e-6 or len_bc < 1e-6 or len_ac < 1e-6:
            return 0.0
        
        # 使用三角形面积公式计算曲率
        # 曲率 = 4 * 面积 / (边长乘积)
        cross_product = np.cross(ab, bc)
        area = abs(cross_product) / 2.0
        
        curvature = 4.0 * area / (len_ab * len_bc * len_ac)
        return curvature
    
    def smooth_path_curvature(self, path: Path, max_iterations: int = 100) -> Path:
        """
        平滑路径以满足转弯半径约束
        
        Args:
            path: 原始路径
            max_iterations: 最大迭代次数
            
        Returns:
            平滑后的路径
        """
        if len(path.poses) < 3:
            return path
        
        smoothed_path = Path()
        smoothed_path.header = path.header
        smoothed_path.poses = [pose for pose in path.poses]  # 深度复制
        
        for iteration in range(max_iterations):
            is_valid, invalid_indices = self.validate_path_curvature(smoothed_path)
            
            if is_valid:
                break
            
            # 对无效点进行平滑处理
            for idx in invalid_indices:
                if 1 <= idx < len(smoothed_path.poses) - 1:
                    # 简单的平均平滑
                    prev_pose = smoothed_path.poses[idx-1].pose.position
                    next_pose = smoothed_path.poses[idx+1].pose.position
                    
                    current_pose = smoothed_path.poses[idx].pose.position
                    current_pose.x = (prev_pose.x + next_pose.x) / 2.0
                    current_pose.y = (prev_pose.y + next_pose.y) / 2.0
        
        return smoothed_path
    
    def predict_trajectory(self, current_pose: Pose, linear_vel: float, 
                          steer_angle: float, prediction_time: float, 
                          dt: float = 0.1) -> List[Pose]:
        """
        基于阿克曼运动学模型预测车辆轨迹
        
        Args:
            current_pose: 当前位姿
            linear_vel: 线速度 (m/s)
            steer_angle: 转向角 (弧度)
            prediction_time: 预测时间 (秒)
            dt: 时间步长 (秒)
            
        Returns:
            预测的轨迹点列表
        """
        trajectory = []
        
        # 当前状态
        x = current_pose.position.x
        y = current_pose.position.y
        theta = self._get_yaw_from_pose(current_pose)
        
        # 计算角速度
        if abs(steer_angle) < 1e-6:
            angular_vel = 0.0
        else:
            turning_radius = self.calculate_turning_radius(steer_angle)
            angular_vel = linear_vel / turning_radius
            if steer_angle < 0:
                angular_vel = -angular_vel
        
        # 数值积分
        for t in np.arange(0, prediction_time, dt):
            # 更新状态
            x += linear_vel * math.cos(theta) * dt
            y += linear_vel * math.sin(theta) * dt
            theta += angular_vel * dt
            
            # 创建预测位姿
            predicted_pose = Pose()
            predicted_pose.position.x = x
            predicted_pose.position.y = y
            predicted_pose.position.z = current_pose.position.z
            predicted_pose.orientation = self._yaw_to_quaternion(theta)
            
            trajectory.append(predicted_pose)
        
        return trajectory
    
    def _get_yaw_from_pose(self, pose: Pose) -> float:
        """从位姿中提取航向角"""
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    
    def _yaw_to_quaternion(self, yaw: float):
        """将航向角转换为四元数"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q
    
    def check_trajectory_feasibility(self, trajectory: List[Pose]) -> bool:
        """
        检查轨迹是否在阿克曼约束下可行
        
        Args:
            trajectory: 要检查的轨迹
            
        Returns:
            轨迹是否可行
        """
        if len(trajectory) < 2:
            return True
        
        for i in range(len(trajectory) - 1):
            current = trajectory[i]
            next_pose = trajectory[i + 1]
            
            # 计算所需的转弯半径
            dx = next_pose.position.x - current.position.x
            dy = next_pose.position.y - current.position.y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 1e-6:
                continue
            
            current_yaw = self._get_yaw_from_pose(current)
            next_yaw = self._get_yaw_from_pose(next_pose)
            
            delta_yaw = next_yaw - current_yaw
            # 角度归一化到[-π, π]
            while delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            while delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            if abs(delta_yaw) > 1e-6:
                # 近似计算转弯半径
                required_radius = distance / (2 * math.sin(abs(delta_yaw) / 2))
                
                if required_radius < self.min_turning_radius:
                    return False
        
        return True 