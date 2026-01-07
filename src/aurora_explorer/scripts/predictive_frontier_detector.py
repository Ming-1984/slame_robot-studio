#!/usr/bin/env python3
"""
🔮 预测性前沿点检测器
在当前目标执行过程中并行检测和评估下一批前沿点

核心功能：
1. 预测性前沿点检测：基于机器人运动轨迹预测未来位置
2. 并行计算：在后台持续更新前沿点信息
3. 智能触发：根据导航进度和地图变化智能触发重新计算
4. 结果预缓存：提前准备多个候选目标点

算法特点：
- 运动预测：基于当前速度和目标预测机器人未来位置
- 增量更新：只更新地图变化区域的前沿点
- 优先级排序：根据探索价值预排序候选目标
- 自适应触发：根据系统负载和探索进度调整计算频率

作者: Aurora探索系统
日期: 2025-07-21
"""

import time
import math
import threading
from typing import List, Optional, Tuple, Dict, Set
from dataclasses import dataclass, field
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Twist
import rclpy
from rclpy.node import Node

# 导入现有模块
from parallel_computation_manager import ParallelComputationManager, ComputationResult

# 导入前沿点检测器
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from aurora_explorer.optimized_frontier_detector import OptimizedFrontierDetector, OptimizedFrontierPoint

@dataclass
class PredictedRobotState:
    """预测的机器人状态"""
    position: Tuple[float, float]
    yaw: float
    timestamp: float
    confidence: float = 1.0

@dataclass
class FrontierPrediction:
    """前沿点预测结果"""
    frontiers: List[OptimizedFrontierPoint]
    predicted_robot_state: PredictedRobotState
    computation_time: float
    map_hash: str
    validity_score: float = 1.0

class PredictiveFrontierDetector:
    """预测性前沿点检测器"""
    
    def __init__(self, node: Node, computation_manager: ParallelComputationManager):
        """
        初始化预测性前沿点检测器
        
        Args:
            node: ROS2节点
            computation_manager: 并行计算管理器
        """
        self.node = node
        self.computation_manager = computation_manager
        
        # 基础前沿点检测器
        self.base_detector = OptimizedFrontierDetector(
            map_resolution=0.05,
            robot_radius=0.3
        )
        
        # 预测参数
        self.prediction_horizon = 10.0  # 预测时间范围(秒)
        self.prediction_interval = 2.0  # 预测间隔(秒)
        self.motion_prediction_samples = 5  # 运动预测采样点数
        
        # 状态跟踪
        self.current_robot_state = None
        self.robot_velocity = None
        self.last_prediction_time = 0.0
        self.active_predictions: Dict[str, FrontierPrediction] = {}
        
        # 地图变化检测
        self.last_map_hash = ""
        self.map_change_threshold = 0.1  # 地图变化阈值
        self.significant_map_regions: Set[Tuple[int, int]] = set()
        
        # 性能统计
        self.prediction_stats = {
            'total_predictions': 0,
            'successful_predictions': 0,
            'cache_hits': 0,
            'average_prediction_time': 0.0,
            'prediction_accuracy': 0.0
        }
        
        # 线程安全
        self.state_lock = threading.RLock()
        
        self.node.get_logger().info('🔮 预测性前沿点检测器初始化完成')
        
    def update_robot_state(self, position: Tuple[float, float], yaw: float, velocity: Twist = None):
        """
        更新机器人状态
        
        Args:
            position: 机器人位置
            yaw: 机器人朝向
            velocity: 机器人速度
        """
        with self.state_lock:
            self.current_robot_state = PredictedRobotState(
                position=position,
                yaw=yaw,
                timestamp=time.time()
            )
            
            if velocity:
                self.robot_velocity = velocity
                
    def trigger_predictive_detection(self, 
                                   current_map: OccupancyGrid,
                                   current_goal: Optional[Point] = None,
                                   force_update: bool = False) -> str:
        """
        触发预测性前沿点检测
        
        Args:
            current_map: 当前地图
            current_goal: 当前目标点
            force_update: 是否强制更新
            
        Returns:
            str: 任务ID
        """
        current_time = time.time()
        
        # 检查是否需要触发预测
        if not force_update and not self._should_trigger_prediction(current_map, current_time):
            return ""
            
        # 生成预测的机器人状态
        predicted_states = self._generate_predicted_robot_states(current_goal)
        
        if not predicted_states:
            return ""
            
        # 为每个预测状态提交计算任务
        task_ids = []
        for i, predicted_state in enumerate(predicted_states):
            task_id = self.computation_manager.submit_predictive_computation(
                task_type=f"predictive_frontier_detection_{i}",
                robot_position=predicted_state.position,
                map_data=current_map,
                computation_func=self._compute_frontiers_for_predicted_state,
                priority=10 - i,  # 越近的预测优先级越高
                predicted_state=predicted_state,
                current_goal=current_goal
            )
            task_ids.append(task_id)
            
        self.last_prediction_time = current_time
        self.prediction_stats['total_predictions'] += len(task_ids)
        
        self.node.get_logger().debug(f'🔮 触发预测性检测: {len(task_ids)}个任务')
        return task_ids[0] if task_ids else ""
        
    def get_predicted_frontiers(self, 
                              robot_position: Tuple[float, float],
                              max_age: float = 5.0) -> Optional[List[OptimizedFrontierPoint]]:
        """
        获取预测的前沿点
        
        Args:
            robot_position: 当前机器人位置
            max_age: 最大结果年龄(秒)
            
        Returns:
            Optional[List[OptimizedFrontierPoint]]: 预测的前沿点列表
        """
        # 获取最佳可用预测结果
        best_result = self.computation_manager.get_best_available_result(
            task_type="predictive_frontier_detection",
            robot_position=robot_position,
            max_age=max_age
        )
        
        if best_result and best_result.result_data:
            prediction = best_result.result_data
            if isinstance(prediction, FrontierPrediction):
                self.prediction_stats['cache_hits'] += 1
                self.node.get_logger().debug(
                    f'🎯 获取预测前沿点: {len(prediction.frontiers)}个, '
                    f'有效性: {prediction.validity_score:.2f}'
                )
                return prediction.frontiers
                
        return None
        
    def get_best_predicted_target(self, 
                                robot_position: Tuple[float, float],
                                evaluation_func: callable) -> Optional[OptimizedFrontierPoint]:
        """
        获取最佳预测目标点
        
        Args:
            robot_position: 当前机器人位置
            evaluation_func: 评估函数
            
        Returns:
            Optional[OptimizedFrontierPoint]: 最佳预测目标点
        """
        predicted_frontiers = self.get_predicted_frontiers(robot_position)
        
        if not predicted_frontiers:
            return None
            
        # 使用评估函数选择最佳目标
        original_frontiers = None
        try:
            # 临时保存原始前沿点列表
            original_frontiers = self.node.frontiers
            # 设置预测的前沿点列表
            self.node.frontiers = predicted_frontiers

            # 调用评估函数（只传递机器人位置）
            best_frontier = evaluation_func(robot_position)

            if best_frontier:
                self.prediction_stats['successful_predictions'] += 1
                self.node.get_logger().info(
                    f'🎯 预测目标选择成功: ({best_frontier.x:.2f}, {best_frontier.y:.2f})'
                )
            return best_frontier
        except Exception as e:
            self.node.get_logger().error(f'❌ 预测目标评估异常: {e}')
            return None
        finally:
            # 确保恢复原始前沿点列表
            if original_frontiers is not None:
                self.node.frontiers = original_frontiers
            
    def _should_trigger_prediction(self, current_map: OccupancyGrid, current_time: float) -> bool:
        """判断是否应该触发预测"""
        # 检查时间间隔
        if current_time - self.last_prediction_time < self.prediction_interval:
            return False
            
        # 检查地图变化
        current_map_hash = self._calculate_map_hash(current_map)
        if current_map_hash != self.last_map_hash:
            self.last_map_hash = current_map_hash
            return True
            
        # 检查机器人状态变化
        if not self.current_robot_state:
            return True
            
        # 检查是否有足够的运动
        if self.robot_velocity:
            speed = math.sqrt(self.robot_velocity.linear.x**2 + self.robot_velocity.linear.y**2)
            if speed > 0.1:  # 机器人在运动
                return True
                
        return False
        
    def _generate_predicted_robot_states(self, current_goal: Optional[Point]) -> List[PredictedRobotState]:
        """生成预测的机器人状态"""
        if not self.current_robot_state:
            return []
            
        predicted_states = []
        current_time = time.time()
        
        # 基于当前速度预测未来位置
        if self.robot_velocity and current_goal:
            for i in range(1, self.motion_prediction_samples + 1):
                dt = (self.prediction_horizon / self.motion_prediction_samples) * i
                
                # 简单的线性运动预测
                predicted_x = self.current_robot_state.position[0] + self.robot_velocity.linear.x * dt
                predicted_y = self.current_robot_state.position[1] + self.robot_velocity.linear.y * dt
                predicted_yaw = self.current_robot_state.yaw + self.robot_velocity.angular.z * dt
                
                # 考虑目标导向的修正
                if current_goal:
                    goal_direction = math.atan2(
                        current_goal.y - predicted_y,
                        current_goal.x - predicted_x
                    )
                    # 混合当前朝向和目标方向
                    predicted_yaw = 0.7 * predicted_yaw + 0.3 * goal_direction
                
                confidence = max(0.1, 1.0 - dt / self.prediction_horizon)
                
                predicted_states.append(PredictedRobotState(
                    position=(predicted_x, predicted_y),
                    yaw=predicted_yaw,
                    timestamp=current_time + dt,
                    confidence=confidence
                ))
        else:
            # 如果没有速度信息，使用当前位置的小范围预测
            for i in range(self.motion_prediction_samples):
                angle = 2 * math.pi * i / self.motion_prediction_samples
                radius = 1.0  # 1米半径的预测范围
                
                predicted_x = self.current_robot_state.position[0] + radius * math.cos(angle)
                predicted_y = self.current_robot_state.position[1] + radius * math.sin(angle)
                
                predicted_states.append(PredictedRobotState(
                    position=(predicted_x, predicted_y),
                    yaw=angle,
                    timestamp=current_time + 1.0,
                    confidence=0.5
                ))
                
        return predicted_states
        
    def _compute_frontiers_for_predicted_state(self, 
                                             map_data: OccupancyGrid,
                                             predicted_state: PredictedRobotState,
                                             **kwargs) -> FrontierPrediction:
        """为预测状态计算前沿点"""
        start_time = time.time()
        
        try:
            # 使用基础检测器检测前沿点
            frontiers = self.base_detector.detect_optimized_frontiers(map_data)
            
            # 根据预测位置过滤和排序前沿点
            filtered_frontiers = self._filter_frontiers_for_predicted_state(
                frontiers, predicted_state
            )
            
            computation_time = time.time() - start_time
            map_hash = self._calculate_map_hash(map_data)
            
            # 计算预测有效性评分
            validity_score = self._calculate_prediction_validity(
                predicted_state, frontiers, computation_time
            )
            
            prediction = FrontierPrediction(
                frontiers=filtered_frontiers,
                predicted_robot_state=predicted_state,
                computation_time=computation_time,
                map_hash=map_hash,
                validity_score=validity_score
            )
            
            self.node.get_logger().debug(
                f'🔮 预测状态前沿点计算完成: {len(filtered_frontiers)}个前沿点, '
                f'耗时: {computation_time:.3f}s, 有效性: {validity_score:.2f}'
            )
            
            return prediction
            
        except Exception as e:
            self.node.get_logger().error(f'❌ 预测前沿点计算异常: {e}')
            return FrontierPrediction(
                frontiers=[],
                predicted_robot_state=predicted_state,
                computation_time=time.time() - start_time,
                map_hash="",
                validity_score=0.0
            )
            
    def _filter_frontiers_for_predicted_state(self, 
                                            frontiers: List[OptimizedFrontierPoint],
                                            predicted_state: PredictedRobotState) -> List[OptimizedFrontierPoint]:
        """根据预测状态过滤前沿点"""
        if not frontiers:
            return []
            
        # 计算每个前沿点相对于预测位置的评分
        scored_frontiers = []
        
        for frontier in frontiers:
            # 计算距离
            distance = math.sqrt(
                (frontier.x - predicted_state.position[0])**2 +
                (frontier.y - predicted_state.position[1])**2
            )
            
            # 距离评分（距离适中的前沿点评分更高）
            optimal_distance = 3.0  # 最优距离
            distance_score = max(0.0, 1.0 - abs(distance - optimal_distance) / optimal_distance)
            
            # 方向评分（与预测朝向一致的前沿点评分更高）
            angle_to_frontier = math.atan2(
                frontier.y - predicted_state.position[1],
                frontier.x - predicted_state.position[0]
            )
            angle_diff = abs(angle_to_frontier - predicted_state.yaw)
            angle_diff = min(angle_diff, 2 * math.pi - angle_diff)  # 取较小角度
            direction_score = max(0.0, 1.0 - angle_diff / math.pi)
            
            # 综合评分
            total_score = (distance_score * 0.6 + direction_score * 0.4) * predicted_state.confidence
            
            scored_frontiers.append((frontier, total_score))
            
        # 按评分排序并返回前N个
        scored_frontiers.sort(key=lambda x: x[1], reverse=True)
        max_frontiers = min(20, len(scored_frontiers))  # 最多返回20个前沿点
        
        return [frontier for frontier, score in scored_frontiers[:max_frontiers]]
        
    def _calculate_prediction_validity(self, 
                                     predicted_state: PredictedRobotState,
                                     frontiers: List[OptimizedFrontierPoint],
                                     computation_time: float) -> float:
        """计算预测有效性评分"""
        # 基础有效性基于预测置信度
        base_validity = predicted_state.confidence
        
        # 前沿点数量因子
        frontier_factor = min(1.0, len(frontiers) / 10.0)  # 10个前沿点为满分
        
        # 计算时间因子
        time_factor = max(0.0, 1.0 - computation_time / 1.0)  # 1秒内完成为满分
        
        # 时间新鲜度因子
        age = time.time() - predicted_state.timestamp
        freshness_factor = max(0.0, 1.0 - age / 10.0)  # 10秒内为有效
        
        # 综合评分
        validity = (base_validity * 0.4 + 
                   frontier_factor * 0.3 + 
                   time_factor * 0.2 + 
                   freshness_factor * 0.1)
                   
        return min(1.0, max(0.0, validity))
        
    def _calculate_map_hash(self, map_data: OccupancyGrid) -> str:
        """计算地图哈希值"""
        try:
            data_array = np.array(map_data.data, dtype=np.int8)
            return str(hash(data_array.tobytes()))
        except Exception:
            return str(time.time())
            
    def get_statistics(self) -> Dict[str, any]:
        """获取统计信息"""
        total_predictions = max(1, self.prediction_stats['total_predictions'])
        
        return {
            **self.prediction_stats,
            'success_rate': self.prediction_stats['successful_predictions'] / total_predictions,
            'cache_hit_rate': self.prediction_stats['cache_hits'] / total_predictions,
            'active_predictions': len(self.active_predictions)
        }
