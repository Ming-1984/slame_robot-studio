#!/usr/bin/env python3
"""
🏠 房间感知探索策略
实现基于房间结构的智能探索和路径规划优化
Author: Acamana-Bot Development Team
Date: 2025-01-15
"""

import math
import numpy as np
import cv2
from typing import List, Tuple, Dict, Optional, Set
from dataclasses import dataclass
from enum import Enum


class ExplorationStrategy(Enum):
    """探索策略枚举"""
    FRONTIER_BASED = "frontier_based"           # 基于前沿的探索
    ROOM_COMPLETION = "room_completion"         # 房间完成式探索  
    ROOM_TRANSITION = "room_transition"         # 房间间转移
    GLOBAL_COVERAGE = "global_coverage"         # 全局覆盖


@dataclass
class RoomExplorationState:
    """房间探索状态"""
    room_id: int
    completion_ratio: float = 0.0
    exploration_time: float = 0.0
    frontier_count: int = 0
    last_visit_time: float = 0.0
    is_accessible: bool = True
    priority_score: float = 0.0


class RoomAwareExplorer:
    """房间感知探索器"""
    
    def __init__(self):
        # 🏠 房间管理参数
        self.room_completion_threshold = 0.95   # 房间完成度阈值
        self.min_room_area = 0.5               # 最小房间面积 (m²)
        self.door_width_range = (0.6, 1.5)    # 门洞宽度范围
        
        # 🎯 探索策略参数
        self.strategy_weights = {
            'completion_bonus': 0.3,      # 完成度加成
            'accessibility': 0.25,        # 可达性权重
            'exploration_value': 0.25,    # 探索价值权重
            'distance_penalty': 0.2       # 距离惩罚权重
        }
        
        # 📊 状态管理
        self.room_states: Dict[int, RoomExplorationState] = {}
        self.visited_rooms: Set[int] = set()
        self.current_room_id: Optional[int] = None
        self.current_strategy = ExplorationStrategy.FRONTIER_BASED
        
        # 🔄 动态调整参数
        self.strategy_switch_threshold = 300.0  # 策略切换时间阈值(秒)
        self.room_revisit_interval = 600.0      # 房间重访间隔(秒)
        
        # 📈 性能统计
        self.exploration_stats = {
            'rooms_completed': 0,
            'strategy_switches': 0,
            'total_exploration_time': 0.0,
            'efficiency_improvements': 0
        }

    def update_room_info(self, room_centroids: Dict, room_stats: Dict, 
                        current_robot_pos: Tuple[float, float], current_time: float) -> None:
        """更新房间信息和状态"""
        
        # 🔄 更新当前所在房间
        self.current_room_id = self._determine_current_room(current_robot_pos, room_centroids)
        # 📊 更新房间状态
        for room_id, stats in room_stats.items():
            if room_id not in self.room_states:
                self.room_states[room_id] = RoomExplorationState(room_id=room_id)
            
            state = self.room_states[room_id]
            
            # 计算房间完成度
            unknown_ratio = stats.get('unknown_ratio', 1.0)
            state.completion_ratio = max(0.0, 1.0 - unknown_ratio)
            
            # 更新前沿计数
            state.frontier_count = self._count_room_frontiers(room_id, stats)
            
            # 更新访问时间
            if room_id == self.current_room_id:
                state.last_visit_time = current_time
                if room_id not in self.visited_rooms:
                    self.visited_rooms.add(room_id)
            
            # 计算优先级评分
            state.priority_score = self._calculate_room_priority(state, stats, current_robot_pos, room_centroids)

    def select_exploration_strategy(self, current_time: float) -> ExplorationStrategy:
        """选择最佳探索策略"""
        
        # 🏠 检查当前房间状态
        if self.current_room_id and self.current_room_id in self.room_states:
            current_room = self.room_states[self.current_room_id]
            
            # 当前房间已完成？
            if current_room.completion_ratio >= self.room_completion_threshold:
                self.exploration_stats['rooms_completed'] += 1
                return ExplorationStrategy.ROOM_TRANSITION
            
            # 当前房间还有探索价值？
            if current_room.frontier_count > 0 and current_room.completion_ratio < 0.8:
                return ExplorationStrategy.ROOM_COMPLETION
        
        # 🎯 检查是否需要切换到其他房间
        best_room = self._find_best_target_room(current_time)
        if best_room and best_room != self.current_room_id:
            return ExplorationStrategy.ROOM_TRANSITION
        
        # 📍 默认基于前沿的探索
        return ExplorationStrategy.FRONTIER_BASED

    def plan_room_transition(self, target_room_id: int, room_centroids: Dict, 
                           current_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """规划房间转移路径"""
        
        if target_room_id not in room_centroids:
            return None
        
        target_pos = room_centroids[target_room_id]
        
        # 🚪 寻找最佳门洞路径
        door_position = self._find_optimal_door(current_pos, target_pos, target_room_id)
        
        if door_position:
            return door_position
        else:
            # 后备方案：直接前往房间中心
            return target_pos

    def get_room_exploration_priority(self, room_id: int) -> float:
        """获取房间探索优先级"""
        if room_id in self.room_states:
            return self.room_states[room_id].priority_score
        return 0.0

    def is_room_exploration_complete(self, room_id: int) -> bool:
        """检查房间探索是否完成"""
        if room_id in self.room_states:
            state = self.room_states[room_id]
            return (state.completion_ratio >= self.room_completion_threshold and 
                   state.frontier_count == 0)
        return False

    def get_exploration_efficiency_metrics(self) -> Dict:
        """获取探索效率指标"""
        total_rooms = len(self.room_states)
        completed_rooms = self.exploration_stats['rooms_completed']
        
        efficiency_score = 0.0
        if total_rooms > 0:
            completion_rate = completed_rooms / total_rooms
            
            # 计算平均房间完成度
            avg_completion = np.mean([state.completion_ratio for state in self.room_states.values()])
            
            # 综合效率评分
            efficiency_score = (completion_rate * 0.6 + avg_completion * 0.4)
        
        return {
            'total_rooms': total_rooms,
            'completed_rooms': completed_rooms,
            'completion_rate': completed_rooms / max(1, total_rooms),
            'average_completion': avg_completion if total_rooms > 0 else 0.0,
            'efficiency_score': efficiency_score,
            'strategy_switches': self.exploration_stats['strategy_switches'],
            'current_strategy': self.current_strategy.value
        }

    def optimize_exploration_sequence(self, available_rooms: List[int], 
                                    room_centroids: Dict, current_pos: Tuple[float, float]) -> List[int]:
        """优化探索序列"""
        
        if not available_rooms:
            return []
        
        # 🎯 计算每个房间的综合评分
        room_scores = []
        for room_id in available_rooms:
            if room_id in self.room_states:
                state = self.room_states[room_id]
                
                # 距离因子
                if room_id in room_centroids:
                    room_pos = room_centroids[room_id]
                    distance = math.sqrt((current_pos[0] - room_pos[0])**2 + 
                                       (current_pos[1] - room_pos[1])**2)
                    distance_factor = 1.0 / (1.0 + distance / 5.0)
                else:
                    distance_factor = 0.1
                
                # 探索价值因子
                exploration_factor = (1.0 - state.completion_ratio) * (1.0 + state.frontier_count / 10.0)
                
                # 可达性因子
                accessibility_factor = 1.0 if state.is_accessible else 0.1
                
                # 综合评分
                total_score = (exploration_factor * 0.5 + 
                             distance_factor * 0.3 + 
                             accessibility_factor * 0.2)
                
                room_scores.append((room_id, total_score))
        
        # 📊 按评分排序
        room_scores.sort(key=lambda x: x[1], reverse=True)
        
        return [room_id for room_id, _ in room_scores]

    # ================== 私有方法 ==================

    def _determine_current_room(self, robot_pos: Tuple[float, float], 
                              room_centroids: Dict) -> Optional[int]:
        """确定机器人当前所在房间"""
        min_distance = float('inf')
        closest_room = None
        
        for room_id, centroid in room_centroids.items():
            distance = math.sqrt((robot_pos[0] - centroid[0])**2 + 
                               (robot_pos[1] - centroid[1])**2)
            
            # 如果距离足够近，认为在这个房间内
            if distance < 2.0 and distance < min_distance:
                min_distance = distance
                closest_room = room_id
        
        return closest_room

    def _count_room_frontiers(self, room_id: int, room_stats: Dict) -> int:
        """统计房间内的前沿点数量"""
        # 简化实现：基于边界因子估算前沿数量
        boundary_factor = room_stats.get('boundary_factor', 1.0)
        area = room_stats.get('area', 0)
        unknown_ratio = room_stats.get('unknown_ratio', 0.0)
        
        # 前沿数量估算
        estimated_frontiers = int(boundary_factor * unknown_ratio * max(1, area / 100))
        
        return estimated_frontiers

    def _calculate_room_priority(self, state: RoomExplorationState, room_stats: Dict,
                               robot_pos: Tuple[float, float], room_centroids: Dict) -> float:
        """计算房间探索优先级"""
        
        # 🎯 基础因子
        completion_factor = 1.0 - state.completion_ratio
        frontier_factor = min(1.0, state.frontier_count / 5.0)
        
        # 📍 距离因子
        distance_factor = 1.0
        if state.room_id in room_centroids:
            room_pos = room_centroids[state.room_id]
            distance = math.sqrt((robot_pos[0] - room_pos[0])**2 + (robot_pos[1] - room_pos[1])**2)
            distance_factor = 1.0 / (1.0 + distance / 3.0)
        
        # 🏠 房间特征因子
        area_factor = min(1.0, room_stats.get('area', 0) / 500.0)
        boundary_factor = min(1.0, room_stats.get('boundary_factor', 1.0) / 2.0)
        
        # 🔄 访问历史因子
        revisit_factor = 1.0
        if state.room_id in self.visited_rooms:
            revisit_factor = 0.7  # 已访问房间优先级稍低
        
        # 📊 综合优先级计算
        priority = (completion_factor * self.strategy_weights['completion_bonus'] +
                   frontier_factor * self.strategy_weights['exploration_value'] +
                   distance_factor * self.strategy_weights['distance_penalty'] +
                   (area_factor + boundary_factor) * self.strategy_weights['accessibility']) * revisit_factor
        
        return max(0.0, min(1.0, priority))

    def _find_best_target_room(self, current_time: float) -> Optional[int]:
        """寻找最佳目标房间"""
        best_room = None
        best_score = 0.0
        
        for room_id, state in self.room_states.items():
            # 跳过已完成的房间
            if state.completion_ratio >= self.room_completion_threshold:
                continue
            
            # 检查重访间隔
            if (room_id in self.visited_rooms and 
                current_time - state.last_visit_time < self.room_revisit_interval):
                continue
            
            # 评估房间价值
            if state.priority_score > best_score:
                best_score = state.priority_score
                best_room = room_id
        
        return best_room

    def _find_optimal_door(self, current_pos: Tuple[float, float], 
                         target_pos: Tuple[float, float], target_room_id: int) -> Optional[Tuple[float, float]]:
        """寻找最优门洞位置"""
        
        # 简化实现：在当前位置和目标位置之间寻找中点作为门洞
        # 实际实现中应该基于地图数据进行门洞检测
        
        mid_x = (current_pos[0] + target_pos[0]) / 2.0
        mid_y = (current_pos[1] + target_pos[1]) / 2.0
        
        return (mid_x, mid_y)

    def reset_exploration_state(self) -> None:
        """重置探索状态"""
        self.room_states.clear()
        self.visited_rooms.clear()
        self.current_room_id = None
        self.current_strategy = ExplorationStrategy.FRONTIER_BASED
        
        # 重置统计信息
        self.exploration_stats = {
            'rooms_completed': 0,
            'strategy_switches': 0,
            'total_exploration_time': 0.0,
            'efficiency_improvements': 0
        } 