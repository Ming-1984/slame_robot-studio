#!/usr/bin/env python3
"""
🧪 传感器探测范围切换逻辑测试脚本
测试新的基于探测范围的前沿点切换逻辑性能

功能：
1. 测试不同参数配置下的切换性能
2. 对比新旧切换逻辑的效率
3. 生成性能报告和参数调优建议

作者: Aurora探索系统
日期: 2025-07-21
"""

import rclpy
from rclpy.node import Node
import time
import math
import json
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass, asdict
from sensor_range_detector import SensorRangeDetector, SensorConfig, VisibilityResult
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

@dataclass
class TestConfig:
    """测试配置"""
    sensor_max_range: float = 10.0
    sensor_min_range: float = 0.1
    field_of_view: float = 360.0
    angular_resolution: float = 1.0
    min_goal_stay_time: float = 3.0
    switch_confidence_threshold: float = 0.8
    consecutive_checks_required: int = 3
    max_switch_frequency: float = 0.2

@dataclass
class TestResult:
    """测试结果"""
    config_name: str
    total_switches: int = 0
    successful_switches: int = 0
    false_positives: int = 0
    average_switch_time: float = 0.0
    exploration_efficiency: float = 0.0
    switch_accuracy: float = 0.0
    performance_score: float = 0.0

class SensorRangeSwitchingTester(Node):
    """传感器探测范围切换逻辑测试器"""
    
    def __init__(self):
        super().__init__('sensor_range_switching_tester')
        
        # 测试配置列表
        self.test_configs = self._generate_test_configs()
        
        # 测试结果
        self.test_results: List[TestResult] = []
        
        # 模拟数据
        self.simulation_scenarios = self._generate_simulation_scenarios()
        
        self.get_logger().info('🧪 传感器探测范围切换逻辑测试器初始化完成')
        
    def _generate_test_configs(self) -> List[TestConfig]:
        """生成测试配置"""
        configs = []
        
        # 基础配置
        base_config = TestConfig()
        configs.append(TestConfig(**asdict(base_config)))
        
        # 高灵敏度配置
        high_sensitivity = TestConfig(
            switch_confidence_threshold=0.6,
            consecutive_checks_required=2,
            min_goal_stay_time=2.0
        )
        configs.append(high_sensitivity)
        
        # 保守配置
        conservative = TestConfig(
            switch_confidence_threshold=0.9,
            consecutive_checks_required=5,
            min_goal_stay_time=5.0
        )
        configs.append(conservative)
        
        # 短距离优化配置
        short_range = TestConfig(
            sensor_max_range=5.0,
            switch_confidence_threshold=0.7,
            consecutive_checks_required=2
        )
        configs.append(short_range)
        
        # 长距离优化配置
        long_range = TestConfig(
            sensor_max_range=15.0,
            switch_confidence_threshold=0.8,
            consecutive_checks_required=4
        )
        configs.append(long_range)
        
        return configs
        
    def _generate_simulation_scenarios(self) -> List[Dict]:
        """生成模拟测试场景"""
        scenarios = []
        
        # 场景1：开放空间
        scenarios.append({
            'name': 'open_space',
            'robot_positions': [(0, 0), (2, 0), (4, 0), (6, 0), (8, 0)],
            'target_position': (10, 0),
            'obstacles': [],
            'expected_switches': [False, False, False, True, True]
        })
        
        # 场景2：有障碍物遮挡
        scenarios.append({
            'name': 'with_obstacles',
            'robot_positions': [(0, 0), (2, 0), (4, 0), (6, 0), (8, 0)],
            'target_position': (10, 0),
            'obstacles': [(5, -1), (5, 0), (5, 1)],  # 垂直障碍物
            'expected_switches': [False, False, False, False, True]
        })
        
        # 场景3：复杂环境
        scenarios.append({
            'name': 'complex_environment',
            'robot_positions': [(0, 0), (1, 1), (3, 2), (5, 1), (7, 0)],
            'target_position': (8, 0),
            'obstacles': [(2, 0), (2, 1), (4, 1), (4, 2), (6, 0)],
            'expected_switches': [False, False, False, True, True]
        })
        
        return scenarios
        
    def run_tests(self):
        """运行所有测试"""
        self.get_logger().info('🚀 开始运行传感器探测范围切换逻辑测试')
        
        for i, config in enumerate(self.test_configs):
            config_name = f'config_{i+1}'
            self.get_logger().info(f'📋 测试配置 {config_name}: {asdict(config)}')
            
            result = self._test_configuration(config, config_name)
            self.test_results.append(result)
            
            self.get_logger().info(f'✅ 配置 {config_name} 测试完成: {asdict(result)}')
            
        # 生成测试报告
        self._generate_test_report()
        
    def _test_configuration(self, config: TestConfig, config_name: str) -> TestResult:
        """测试特定配置"""
        result = TestResult(config_name=config_name)
        
        # 创建传感器检测器
        sensor_config = SensorConfig(
            max_range=config.sensor_max_range,
            min_range=config.sensor_min_range,
            field_of_view=config.field_of_view,
            angular_resolution=config.angular_resolution
        )
        detector = SensorRangeDetector(sensor_config)
        
        total_tests = 0
        correct_predictions = 0
        switch_times = []
        
        # 测试所有场景
        for scenario in self.simulation_scenarios:
            scenario_result = self._test_scenario(detector, config, scenario)
            
            total_tests += len(scenario_result['predictions'])
            correct_predictions += sum(1 for pred, exp in zip(
                scenario_result['predictions'], 
                scenario['expected_switches']
            ) if pred == exp)
            
            switch_times.extend(scenario_result['switch_times'])
            
        # 计算结果
        result.total_switches = total_tests
        result.successful_switches = correct_predictions
        result.switch_accuracy = correct_predictions / total_tests if total_tests > 0 else 0.0
        result.average_switch_time = np.mean(switch_times) if switch_times else 0.0
        result.exploration_efficiency = self._calculate_exploration_efficiency(result)
        result.performance_score = self._calculate_performance_score(result)
        
        return result
        
    def _test_scenario(self, detector: SensorRangeDetector, config: TestConfig, scenario: Dict) -> Dict:
        """测试单个场景"""
        predictions = []
        switch_times = []
        
        # 创建模拟地图
        costmap = self._create_mock_costmap(scenario['obstacles'])
        
        for i, robot_pos in enumerate(scenario['robot_positions']):
            start_time = time.time()
            
            # 检测可见性
            visibility_result = detector.is_target_in_sensor_range(
                robot_pos, 0.0, scenario['target_position'], costmap
            )
            
            # 模拟切换逻辑
            should_switch = self._simulate_switch_logic(visibility_result, config)
            predictions.append(should_switch)
            
            switch_time = time.time() - start_time
            switch_times.append(switch_time)
            
        return {
            'predictions': predictions,
            'switch_times': switch_times
        }
        
    def _create_mock_costmap(self, obstacles: List[Tuple[int, int]]) -> OccupancyGrid:
        """创建模拟代价地图"""
        costmap = OccupancyGrid()
        costmap.info.resolution = 0.1
        costmap.info.width = 200
        costmap.info.height = 200
        costmap.info.origin.position.x = -10.0
        costmap.info.origin.position.y = -10.0
        
        # 初始化为自由空间
        data = [0] * (costmap.info.width * costmap.info.height)
        
        # 添加障碍物
        for obs_x, obs_y in obstacles:
            map_x = int((obs_x - costmap.info.origin.position.x) / costmap.info.resolution)
            map_y = int((obs_y - costmap.info.origin.position.y) / costmap.info.resolution)
            
            if 0 <= map_x < costmap.info.width and 0 <= map_y < costmap.info.height:
                index = map_y * costmap.info.width + map_x
                data[index] = 100  # 障碍物
                
        costmap.data = data
        return costmap
        
    def _simulate_switch_logic(self, visibility_result: VisibilityResult, config: TestConfig) -> bool:
        """模拟切换逻辑"""
        # 高可见性切换
        if visibility_result.is_visible and visibility_result.confidence > config.switch_confidence_threshold:
            return True
            
        # 部分可见切换
        if (visibility_result.distance <= config.sensor_max_range and 
            visibility_result.clear_path_ratio > 0.5 and 
            visibility_result.distance <= 3.0):
            return True
            
        # 距离切换
        if visibility_result.distance <= 0.5:  # 目标容差
            return True
            
        return False
        
    def _calculate_exploration_efficiency(self, result: TestResult) -> float:
        """计算探索效率"""
        # 基于准确率和平均切换时间的综合评分
        accuracy_score = result.switch_accuracy
        time_score = max(0.0, 1.0 - result.average_switch_time / 0.1)  # 期望切换时间 < 0.1s
        
        return (accuracy_score * 0.7 + time_score * 0.3)
        
    def _calculate_performance_score(self, result: TestResult) -> float:
        """计算性能评分"""
        return result.exploration_efficiency * 100
        
    def _generate_test_report(self):
        """生成测试报告"""
        report = {
            'test_summary': {
                'total_configurations': len(self.test_configs),
                'test_time': time.strftime('%Y-%m-%d %H:%M:%S'),
                'best_config': None,
                'best_score': 0.0
            },
            'results': []
        }
        
        # 找到最佳配置
        best_result = max(self.test_results, key=lambda r: r.performance_score)
        report['test_summary']['best_config'] = best_result.config_name
        report['test_summary']['best_score'] = best_result.performance_score
        
        # 添加所有结果
        for result in self.test_results:
            report['results'].append(asdict(result))
            
        # 保存报告
        report_file = f'/tmp/sensor_range_switching_test_report_{int(time.time())}.json'
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2)
            
        self.get_logger().info(f'📊 测试报告已保存到: {report_file}')
        self.get_logger().info(f'🏆 最佳配置: {best_result.config_name}, 评分: {best_result.performance_score:.2f}')
        
        # 打印结果摘要
        self._print_results_summary()
        
    def _print_results_summary(self):
        """打印结果摘要"""
        self.get_logger().info('📋 测试结果摘要:')
        self.get_logger().info('-' * 80)
        
        for result in sorted(self.test_results, key=lambda r: r.performance_score, reverse=True):
            self.get_logger().info(
                f'{result.config_name}: 准确率={result.switch_accuracy:.2%}, '
                f'平均时间={result.average_switch_time:.3f}s, '
                f'效率={result.exploration_efficiency:.2%}, '
                f'评分={result.performance_score:.1f}'
            )

def main():
    """主函数"""
    rclpy.init()
    
    tester = SensorRangeSwitchingTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 测试被用户中断')
    except Exception as e:
        tester.get_logger().error(f'❌ 测试异常: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
