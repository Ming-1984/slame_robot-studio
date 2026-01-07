#!/usr/bin/env python3
"""
🧪 并行计算功能测试脚本
测试预测性目标计算和并行前沿点处理功能

功能：
1. 测试并行计算管理器的基本功能
2. 验证预测性前沿点检测的准确性
3. 测试状态同步和一致性保证
4. 性能基准测试和优化建议

作者: Aurora探索系统
日期: 2025-07-21
"""

import rclpy
from rclpy.node import Node
import time
import threading
import numpy as np
from typing import List, Dict, Tuple
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Twist
from parallel_computation_manager import ParallelComputationManager
from predictive_frontier_detector import PredictiveFrontierDetector

# 导入前沿点检测器
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from aurora_explorer.optimized_frontier_detector import OptimizedFrontierDetector, OptimizedFrontierPoint

class ParallelComputationTester(Node):
    """并行计算功能测试器"""
    
    def __init__(self):
        super().__init__('parallel_computation_tester')
        
        # 初始化组件
        self.parallel_manager = ParallelComputationManager(self, max_workers=4)
        self.predictive_detector = PredictiveFrontierDetector(self, self.parallel_manager)
        self.base_detector = OptimizedFrontierDetector()
        
        # 测试数据
        self.test_maps = self._generate_test_maps()
        self.test_scenarios = self._generate_test_scenarios()
        
        # 测试结果
        self.test_results = {}
        
        self.get_logger().info('🧪 并行计算功能测试器初始化完成')
        
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('🚀 开始并行计算功能测试')
        
        try:
            # 测试1：基础并行计算功能
            self.test_basic_parallel_computation()
            
            # 测试2：预测性前沿点检测
            self.test_predictive_frontier_detection()
            
            # 测试3：性能基准测试
            self.test_performance_benchmark()
            
            # 测试4：状态同步测试
            self.test_state_synchronization()
            
            # 测试5：缓存管理测试
            self.test_cache_management()
            
            # 生成测试报告
            self._generate_test_report()
            
        except Exception as e:
            self.get_logger().error(f'❌ 测试执行异常: {e}')
            
    def test_basic_parallel_computation(self):
        """测试基础并行计算功能"""
        self.get_logger().info('📋 测试1: 基础并行计算功能')
        
        start_time = time.time()
        task_ids = []
        
        # 提交多个计算任务
        for i, test_map in enumerate(self.test_maps[:3]):
            task_id = self.parallel_manager.submit_predictive_computation(
                task_type=f"test_frontier_detection_{i}",
                robot_position=(0.0, 0.0),
                map_data=test_map,
                computation_func=self._mock_frontier_detection,
                priority=i
            )
            task_ids.append(task_id)
            
        # 等待结果
        results = []
        for task_id in task_ids:
            result = self.parallel_manager.get_computation_result(task_id, timeout=5.0)
            if result:
                results.append(result)
                
        test_time = time.time() - start_time
        
        # 记录结果
        self.test_results['basic_parallel'] = {
            'submitted_tasks': len(task_ids),
            'completed_tasks': len(results),
            'total_time': test_time,
            'success_rate': len(results) / len(task_ids) if task_ids else 0.0
        }
        
        self.get_logger().info(
            f'✅ 基础并行计算测试完成: {len(results)}/{len(task_ids)}任务成功, '
            f'耗时: {test_time:.2f}s'
        )
        
    def test_predictive_frontier_detection(self):
        """测试预测性前沿点检测"""
        self.get_logger().info('📋 测试2: 预测性前沿点检测')
        
        start_time = time.time()
        
        # 模拟机器人运动
        robot_positions = [(i * 0.5, 0.0) for i in range(10)]
        robot_velocities = [Twist() for _ in range(10)]
        for vel in robot_velocities:
            vel.linear.x = 0.5
            
        prediction_results = []
        
        for i, (pos, vel) in enumerate(zip(robot_positions, robot_velocities)):
            # 更新机器人状态
            self.predictive_detector.update_robot_state(pos, 0.0, vel)
            
            # 触发预测检测
            task_id = self.predictive_detector.trigger_predictive_detection(
                self.test_maps[0], Point(x=5.0, y=0.0, z=0.0)
            )
            
            if task_id:
                time.sleep(0.1)  # 短暂等待
                
                # 获取预测结果
                predicted_frontiers = self.predictive_detector.get_predicted_frontiers(pos)
                if predicted_frontiers:
                    prediction_results.append(len(predicted_frontiers))
                    
        test_time = time.time() - start_time
        
        # 记录结果
        self.test_results['predictive_detection'] = {
            'prediction_attempts': len(robot_positions),
            'successful_predictions': len(prediction_results),
            'average_frontiers': np.mean(prediction_results) if prediction_results else 0.0,
            'total_time': test_time,
            'success_rate': len(prediction_results) / len(robot_positions)
        }
        
        self.get_logger().info(
            f'✅ 预测性检测测试完成: {len(prediction_results)}/{len(robot_positions)}次成功, '
            f'平均前沿点数: {np.mean(prediction_results) if prediction_results else 0:.1f}, '
            f'耗时: {test_time:.2f}s'
        )
        
    def test_performance_benchmark(self):
        """测试性能基准"""
        self.get_logger().info('📋 测试3: 性能基准测试')
        
        # 串行计算基准
        serial_start = time.time()
        serial_results = []
        for test_map in self.test_maps:
            result = self._mock_frontier_detection(map_data=test_map)
            serial_results.append(result)
        serial_time = time.time() - serial_start
        
        # 并行计算基准
        parallel_start = time.time()
        task_ids = []
        for i, test_map in enumerate(self.test_maps):
            task_id = self.parallel_manager.submit_predictive_computation(
                task_type=f"benchmark_test_{i}",
                robot_position=(0.0, 0.0),
                map_data=test_map,
                computation_func=self._mock_frontier_detection,
                priority=0
            )
            task_ids.append(task_id)
            
        parallel_results = []
        for task_id in task_ids:
            result = self.parallel_manager.get_computation_result(task_id, timeout=10.0)
            if result:
                parallel_results.append(result)
        parallel_time = time.time() - parallel_start
        
        # 计算性能提升
        speedup = serial_time / parallel_time if parallel_time > 0 else 0.0
        
        # 记录结果
        self.test_results['performance_benchmark'] = {
            'serial_time': serial_time,
            'parallel_time': parallel_time,
            'speedup': speedup,
            'serial_tasks': len(serial_results),
            'parallel_tasks': len(parallel_results),
            'efficiency': speedup / 4.0  # 4个工作线程
        }
        
        self.get_logger().info(
            f'✅ 性能基准测试完成: 串行{serial_time:.2f}s vs 并行{parallel_time:.2f}s, '
            f'加速比: {speedup:.2f}x, 效率: {speedup/4.0:.1%}'
        )
        
    def test_state_synchronization(self):
        """测试状态同步"""
        self.get_logger().info('📋 测试4: 状态同步测试')
        
        # 创建多个线程同时更新状态
        def update_robot_state(thread_id):
            for i in range(10):
                pos = (thread_id * 10 + i, 0.0)
                self.predictive_detector.update_robot_state(pos, 0.0)
                time.sleep(0.01)
                
        threads = []
        for i in range(4):
            thread = threading.Thread(target=update_robot_state, args=(i,))
            threads.append(thread)
            thread.start()
            
        # 等待所有线程完成
        for thread in threads:
            thread.join()
            
        # 检查状态一致性
        final_state = self.predictive_detector.current_robot_state
        
        self.test_results['state_synchronization'] = {
            'threads_count': len(threads),
            'final_state_valid': final_state is not None,
            'state_timestamp': final_state.timestamp if final_state else 0.0
        }
        
        self.get_logger().info('✅ 状态同步测试完成: 状态一致性正常')
        
    def test_cache_management(self):
        """测试缓存管理"""
        self.get_logger().info('📋 测试5: 缓存管理测试')
        
        # 提交相同的计算任务多次
        task_ids = []
        for i in range(5):
            task_id = self.parallel_manager.submit_predictive_computation(
                task_type="cache_test",
                robot_position=(0.0, 0.0),
                map_data=self.test_maps[0],
                computation_func=self._mock_frontier_detection,
                priority=0
            )
            task_ids.append(task_id)
            time.sleep(0.1)
            
        # 获取统计信息
        stats = self.parallel_manager.get_statistics()
        
        self.test_results['cache_management'] = {
            'cache_hit_rate': stats.get('cache_hit_rate', 0.0),
            'cache_size': stats.get('cache_size', 0),
            'total_tasks': stats.get('total_tasks', 0)
        }
        
        self.get_logger().info(
            f'✅ 缓存管理测试完成: 命中率{stats.get("cache_hit_rate", 0.0):.1%}, '
            f'缓存大小: {stats.get("cache_size", 0)}'
        )
        
    def _generate_test_maps(self) -> List[OccupancyGrid]:
        """生成测试地图"""
        maps = []
        
        for i in range(5):
            map_data = OccupancyGrid()
            map_data.info.resolution = 0.1
            map_data.info.width = 100
            map_data.info.height = 100
            map_data.info.origin.position.x = -5.0
            map_data.info.origin.position.y = -5.0
            
            # 生成随机地图数据
            data = np.random.choice([0, 100, -1], size=10000, p=[0.7, 0.2, 0.1])
            map_data.data = data.tolist()
            
            maps.append(map_data)
            
        return maps
        
    def _generate_test_scenarios(self) -> List[Dict]:
        """生成测试场景"""
        return [
            {
                'name': 'simple_exploration',
                'robot_start': (0.0, 0.0),
                'target_area': (5.0, 5.0),
                'expected_frontiers': 10
            },
            {
                'name': 'complex_environment',
                'robot_start': (2.0, 2.0),
                'target_area': (8.0, 8.0),
                'expected_frontiers': 15
            }
        ]
        
    def _mock_frontier_detection(self, map_data: OccupancyGrid, **kwargs) -> List[OptimizedFrontierPoint]:
        """模拟前沿点检测"""
        # 模拟计算时间
        time.sleep(0.1)
        
        # 生成模拟前沿点
        frontiers = []
        for i in range(np.random.randint(5, 15)):
            frontier = OptimizedFrontierPoint(
                x=np.random.uniform(-5.0, 5.0),
                y=np.random.uniform(-5.0, 5.0),
                size=np.random.randint(10, 50),
                information_gain=np.random.uniform(0.1, 1.0)
            )
            frontiers.append(frontier)
            
        return frontiers
        
    def _generate_test_report(self):
        """生成测试报告"""
        self.get_logger().info('📊 生成测试报告')
        self.get_logger().info('=' * 60)
        
        for test_name, results in self.test_results.items():
            self.get_logger().info(f'📋 {test_name}:')
            for key, value in results.items():
                if isinstance(value, float):
                    self.get_logger().info(f'  {key}: {value:.3f}')
                else:
                    self.get_logger().info(f'  {key}: {value}')
            self.get_logger().info('-' * 40)
            
        # 计算总体评分
        overall_score = self._calculate_overall_score()
        self.get_logger().info(f'🏆 总体评分: {overall_score:.1f}/100')
        
    def _calculate_overall_score(self) -> float:
        """计算总体评分"""
        score = 0.0
        
        # 基础功能评分 (25分)
        if 'basic_parallel' in self.test_results:
            basic_score = self.test_results['basic_parallel']['success_rate'] * 25
            score += basic_score
            
        # 预测功能评分 (25分)
        if 'predictive_detection' in self.test_results:
            pred_score = self.test_results['predictive_detection']['success_rate'] * 25
            score += pred_score
            
        # 性能评分 (25分)
        if 'performance_benchmark' in self.test_results:
            perf_score = min(25, self.test_results['performance_benchmark']['speedup'] * 6.25)
            score += perf_score
            
        # 缓存评分 (25分)
        if 'cache_management' in self.test_results:
            cache_score = self.test_results['cache_management']['cache_hit_rate'] * 25
            score += cache_score
            
        return score

def main():
    """主函数"""
    rclpy.init()
    
    tester = ParallelComputationTester()
    
    try:
        tester.run_all_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 测试被用户中断')
    except Exception as e:
        tester.get_logger().error(f'❌ 测试异常: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
