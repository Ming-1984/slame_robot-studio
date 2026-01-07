#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
增强探索系统性能测试脚本
测试优化后的探索系统性能，包括探索效率、路径质量、系统稳定性等指标
"""

import os
import sys
import time
import json
import math
import threading
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict
import numpy as np

# 添加路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

@dataclass
class ExplorationMetrics:
    """探索指标数据类"""
    start_time: float
    end_time: float
    total_duration: float
    explored_area: float
    total_area: float
    coverage_ratio: float
    path_length: float
    average_speed: float
    frontier_detection_count: int
    navigation_success_rate: float
    system_stability_score: float
    exploration_efficiency: float

@dataclass
class PerformanceTestResult:
    """性能测试结果数据类"""
    test_name: str
    test_duration: float
    exploration_metrics: ExplorationMetrics
    system_metrics: Dict
    quality_scores: Dict
    success: bool
    error_messages: List[str]

class EnhancedExplorationTester(Node):
    """增强探索系统测试器"""
    
    def __init__(self):
        super().__init__('enhanced_exploration_tester')
        
        # 🎯 测试配置
        self.test_duration = 300.0  # 5分钟测试
        self.metrics_collection_interval = 1.0  # 1秒收集一次指标
        
        # 📊 数据收集
        self.map_data: Optional[OccupancyGrid] = None
        self.robot_poses: List[Tuple[float, float, float]] = []
        self.exploration_states: List[str] = []
        self.frontier_counts: List[int] = []
        self.navigation_attempts: int = 0
        self.navigation_successes: int = 0
        
        # 🔄 状态管理
        self.test_running = False
        self.test_start_time = 0.0
        self.last_metrics_time = 0.0
        
        # 📡 ROS订阅
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            PoseStamped, '/robot_pose', self._pose_callback, 10
        )
        self.exploration_state_subscriber = self.create_subscription(
            String, '/exploration_state', self._state_callback, 10
        )
        
        # 📊 指标定时器
        self.metrics_timer = self.create_timer(
            self.metrics_collection_interval, self._collect_metrics
        )
        
        self.get_logger().info('🧪 增强探索系统测试器初始化完成')

    def run_performance_test(self, test_name: str = "enhanced_exploration_test") -> PerformanceTestResult:
        """运行性能测试"""
        self.get_logger().info(f'🚀 开始性能测试: {test_name}')
        
        # 🎯 初始化测试
        self.test_running = True
        self.test_start_time = time.time()
        self._reset_metrics()
        
        try:
            # 🔄 运行测试
            self._run_test_loop()
            
            # 📊 计算最终指标
            exploration_metrics = self._calculate_exploration_metrics()
            system_metrics = self._calculate_system_metrics()
            quality_scores = self._calculate_quality_scores()
            
            # 🏆 创建测试结果
            result = PerformanceTestResult(
                test_name=test_name,
                test_duration=time.time() - self.test_start_time,
                exploration_metrics=exploration_metrics,
                system_metrics=system_metrics,
                quality_scores=quality_scores,
                success=True,
                error_messages=[]
            )
            
            self.get_logger().info(f'✅ 性能测试完成: {test_name}')
            return result
            
        except Exception as e:
            self.get_logger().error(f'❌ 性能测试异常: {e}')
            return PerformanceTestResult(
                test_name=test_name,
                test_duration=time.time() - self.test_start_time,
                exploration_metrics=ExplorationMetrics(0,0,0,0,0,0,0,0,0,0,0,0),
                system_metrics={},
                quality_scores={},
                success=False,
                error_messages=[str(e)]
            )
        finally:
            self.test_running = False

    def _run_test_loop(self) -> None:
        """运行测试循环"""
        while self.test_running and (time.time() - self.test_start_time) < self.test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

    def _reset_metrics(self) -> None:
        """重置指标"""
        self.robot_poses.clear()
        self.exploration_states.clear()
        self.frontier_counts.clear()
        self.navigation_attempts = 0
        self.navigation_successes = 0
        self.last_metrics_time = time.time()

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """地图回调"""
        self.map_data = msg

    def _pose_callback(self, msg: PoseStamped) -> None:
        """位置回调"""
        if self.test_running:
            pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                time.time()
            )
            self.robot_poses.append(pose)

    def _state_callback(self, msg: String) -> None:
        """状态回调"""
        if self.test_running:
            self.exploration_states.append(msg.data)
            
            # 统计导航尝试和成功
            if "navigation_started" in msg.data:
                self.navigation_attempts += 1
            elif "navigation_success" in msg.data:
                self.navigation_successes += 1

    def _collect_metrics(self) -> None:
        """收集指标"""
        if not self.test_running:
            return
        
        current_time = time.time()
        if current_time - self.last_metrics_time >= self.metrics_collection_interval:
            # 收集前沿点数量等指标
            # 这里可以添加更多指标收集逻辑
            self.last_metrics_time = current_time

    def _calculate_exploration_metrics(self) -> ExplorationMetrics:
        """计算探索指标"""
        try:
            end_time = time.time()
            total_duration = end_time - self.test_start_time
            
            # 🗺️ 计算探索面积
            explored_area, total_area, coverage_ratio = self._calculate_coverage()
            
            # 🛤️ 计算路径长度
            path_length = self._calculate_path_length()
            
            # 🚀 计算平均速度
            average_speed = path_length / max(total_duration, 1.0)
            
            # 🎯 计算导航成功率
            navigation_success_rate = (
                self.navigation_successes / max(self.navigation_attempts, 1)
            )
            
            # 🛡️ 计算系统稳定性评分
            system_stability_score = self._calculate_stability_score()
            
            # 📈 计算探索效率
            exploration_efficiency = explored_area / max(total_duration, 1.0)
            
            return ExplorationMetrics(
                start_time=self.test_start_time,
                end_time=end_time,
                total_duration=total_duration,
                explored_area=explored_area,
                total_area=total_area,
                coverage_ratio=coverage_ratio,
                path_length=path_length,
                average_speed=average_speed,
                frontier_detection_count=len(self.frontier_counts),
                navigation_success_rate=navigation_success_rate,
                system_stability_score=system_stability_score,
                exploration_efficiency=exploration_efficiency
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 探索指标计算异常: {e}')
            return ExplorationMetrics(0,0,0,0,0,0,0,0,0,0,0,0)

    def _calculate_coverage(self) -> Tuple[float, float, float]:
        """计算覆盖度"""
        if not self.map_data:
            return 0.0, 0.0, 0.0
        
        total_cells = len(self.map_data.data)
        known_cells = sum(1 for cell in self.map_data.data if cell != -1)
        
        cell_area = self.map_data.info.resolution ** 2
        explored_area = known_cells * cell_area
        total_area = total_cells * cell_area
        coverage_ratio = known_cells / max(total_cells, 1)
        
        return explored_area, total_area, coverage_ratio

    def _calculate_path_length(self) -> float:
        """计算路径长度"""
        if len(self.robot_poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(self.robot_poses)):
            prev_pose = self.robot_poses[i-1]
            curr_pose = self.robot_poses[i]
            
            distance = math.sqrt(
                (curr_pose[0] - prev_pose[0])**2 + 
                (curr_pose[1] - prev_pose[1])**2
            )
            total_length += distance
        
        return total_length

    def _calculate_stability_score(self) -> float:
        """计算系统稳定性评分"""
        # 基于错误状态的比例计算稳定性
        error_states = sum(1 for state in self.exploration_states if "error" in state.lower())
        total_states = len(self.exploration_states)
        
        if total_states == 0:
            return 1.0
        
        stability_score = 1.0 - (error_states / total_states)
        return max(0.0, stability_score)

    def _calculate_system_metrics(self) -> Dict:
        """计算系统指标"""
        return {
            'total_poses_recorded': len(self.robot_poses),
            'total_state_changes': len(self.exploration_states),
            'navigation_attempts': self.navigation_attempts,
            'navigation_successes': self.navigation_successes,
            'average_frontier_count': np.mean(self.frontier_counts) if self.frontier_counts else 0.0
        }

    def _calculate_quality_scores(self) -> Dict:
        """计算质量评分"""
        metrics = self._calculate_exploration_metrics()
        
        # 🎯 各项质量评分 (0-1)
        coverage_score = min(metrics.coverage_ratio / 0.8, 1.0)  # 80%覆盖为满分
        efficiency_score = min(metrics.exploration_efficiency / 0.5, 1.0)  # 0.5 m²/s为满分
        stability_score = metrics.system_stability_score
        navigation_score = metrics.navigation_success_rate
        
        # 🏆 综合质量评分
        overall_score = (
            coverage_score * 0.3 +
            efficiency_score * 0.25 +
            stability_score * 0.25 +
            navigation_score * 0.2
        )
        
        return {
            'coverage_score': coverage_score,
            'efficiency_score': efficiency_score,
            'stability_score': stability_score,
            'navigation_score': navigation_score,
            'overall_score': overall_score
        }

    def save_test_results(self, result: PerformanceTestResult, filename: str = None) -> str:
        """保存测试结果"""
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"exploration_test_results_{timestamp}.json"
        
        # 转换为可序列化的字典
        result_dict = {
            'test_name': result.test_name,
            'test_duration': result.test_duration,
            'exploration_metrics': asdict(result.exploration_metrics),
            'system_metrics': result.system_metrics,
            'quality_scores': result.quality_scores,
            'success': result.success,
            'error_messages': result.error_messages,
            'timestamp': time.time()
        }
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(result_dict, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f'📊 测试结果已保存: {filename}')
            return filename
            
        except Exception as e:
            self.get_logger().error(f'❌ 保存测试结果异常: {e}')
            return ""

def main():
    """主函数"""
    rclpy.init()
    
    try:
        tester = EnhancedExplorationTester()
        
        # 🧪 运行性能测试
        result = tester.run_performance_test("enhanced_exploration_performance_test")
        
        # 📊 保存结果
        filename = tester.save_test_results(result)
        
        # 📈 打印结果摘要
        print("\n" + "="*60)
        print("🧪 增强探索系统性能测试结果")
        print("="*60)
        print(f"测试名称: {result.test_name}")
        print(f"测试时长: {result.test_duration:.1f}秒")
        print(f"测试成功: {'✅' if result.success else '❌'}")
        print("\n📊 探索指标:")
        print(f"  覆盖率: {result.exploration_metrics.coverage_ratio:.1%}")
        print(f"  探索效率: {result.exploration_metrics.exploration_efficiency:.3f} m²/s")
        print(f"  导航成功率: {result.exploration_metrics.navigation_success_rate:.1%}")
        print(f"  系统稳定性: {result.exploration_metrics.system_stability_score:.1%}")
        print("\n🏆 质量评分:")
        for metric, score in result.quality_scores.items():
            print(f"  {metric}: {score:.3f}")
        print(f"\n📁 详细结果已保存至: {filename}")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
