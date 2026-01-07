#!/usr/bin/env python3
"""
🛡️ 稳定探索节点
基于2024年最新研究的容错设计，集成行为树和状态机的优势
Author: Aurora Explorer Team  
Date: 2025-01-20
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import sys
import os
import time
import math
import threading
import numpy as np
import cv2
from typing import Optional, List, Tuple, Dict
from dataclasses import dataclass
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# 添加路径导入
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from aurora_explorer.robust_state_manager import (
    RobustStateManager, StateDefinition, StateTransition,
    StateType, TransitionTrigger
)
# 导入优化的前沿检测器 - 使用完整实现
from aurora_explorer.optimized_frontier_detector import OptimizedFrontierDetector, OptimizedFrontierPoint

# 导入地图优化器 - 使用完整实现
from aurora_explorer.real_time_map_optimizer import AdaptiveMapOptimizer

# 🔍 导入传感器探测范围检测器
sys.path.append(os.path.dirname(__file__))
# 导入传感器范围检测器 - 使用完整实现
from sensor_range_detector import SensorRangeDetector, SensorConfig, VisibilityResult

# 🔄 导入并行计算模块 - 使用完整实现
from parallel_computation_manager import ParallelComputationManager, ComputationState
from predictive_frontier_detector import PredictiveFrontierDetector

# 🏠 导入房间感知探索模块 - 使用完整实现
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'aurora_explorer'))
from room_aware_explorer import RoomAwareExplorer, ExplorationStrategy

# ROS2消息和动作
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray, Marker


class RobustExploreNode(Node):
    """稳定探索节点"""
    
    def __init__(self):
        super().__init__('robust_explore_node')
        
        # 声明参数
        self.declare_parameter('exploration_timeout', 300.0)
        self.declare_parameter('frontier_detection_interval', 2.0)  # 优化：减少到2秒
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('max_recovery_attempts', 3)
        self.declare_parameter('enable_map_optimization', False)  # 地图优化开关
        
        # 获取参数
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.frontier_detection_interval = self.get_parameter('frontier_detection_interval').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_recovery_attempts = self.get_parameter('max_recovery_attempts').value

        # 🚀 优化的导航参数
        self.declare_parameter('navigation_timeout', 30.0)  # 降低导航超时时间
        self.declare_parameter('improved_goal_tolerance', 0.3)  # 更精确的目标容差
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.improved_goal_tolerance = self.get_parameter('improved_goal_tolerance').value

        self.exploration_start_time = time.time()

        # 🎯 增强的智能完成判断参数
        self.declare_parameter('goal_tolerance', 1.2)  # 目标容差距离（放宽）
        self.declare_parameter('max_navigation_distance', 15.0)  # 最大导航距离
        self.declare_parameter('early_completion_enabled', True)  # 启用提前完成
        self.declare_parameter('completion_check_radius', 2.5)  # 完成检查半径
        self.declare_parameter('completion_threshold', 0.7)  # 完成阈值（70%已探索）
        self.declare_parameter('min_approach_distance', 1.5)  # 最小接近距离

        # 🌟 多维度完成判断参数
        self.declare_parameter('global_completion_threshold', 0.85)  # 全局完成阈值
        self.declare_parameter('frontier_density_threshold', 0.1)    # 前沿点密度阈值
        self.declare_parameter('exploration_efficiency_threshold', 0.3)  # 探索效率阈值
        self.declare_parameter('smart_early_completion', True)       # 智能提前完成

        # 🔄 动态前沿点更新参数
        self.declare_parameter('dynamic_frontier_update', True)  # 启用动态前沿点更新
        self.declare_parameter('map_change_threshold', 0.05)  # 地图变化阈值（5%）
        self.declare_parameter('frontier_update_interval', 5.0)  # 前沿点更新间隔（秒）
        self.declare_parameter('force_update_timeout', 30.0)  # 强制更新超时（秒）

        # 🚀 并行计算参数
        self.declare_parameter('enable_parallel_frontier_detection', True)  # 启用并行前沿点检测
        self.declare_parameter('parallel_computation_timeout', 3.0)  # 并行计算超时时间

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_navigation_distance = self.get_parameter('max_navigation_distance').value
        self.early_completion_enabled = self.get_parameter('early_completion_enabled').value
        self.completion_check_radius = self.get_parameter('completion_check_radius').value
        self.completion_threshold = self.get_parameter('completion_threshold').value
        self.min_approach_distance = self.get_parameter('min_approach_distance').value

        # 🌟 多维度完成判断参数
        self.global_completion_threshold = self.get_parameter('global_completion_threshold').value
        self.frontier_density_threshold = self.get_parameter('frontier_density_threshold').value
        self.exploration_efficiency_threshold = self.get_parameter('exploration_efficiency_threshold').value
        self.smart_early_completion = self.get_parameter('smart_early_completion').value

        # 🔄 探索间隔参数
        self.exploration_interval = 2.0  # 默认2秒探索间隔

        # 🛡️ 关闭标志
        self._shutdown_requested = False

        self.dynamic_frontier_update = self.get_parameter('dynamic_frontier_update').value
        self.map_change_threshold = self.get_parameter('map_change_threshold').value
        self.frontier_update_interval = self.get_parameter('frontier_update_interval').value
        self.force_update_timeout = self.get_parameter('force_update_timeout').value

        # 🚀 前沿点检测缓存
        self.frontier_cache = None
        self.frontier_cache_timestamp = 0
        self.frontier_cache_timeout = 10.0  # 缓存10秒

        # 🚀 并行计算参数
        self.enable_parallel_frontier_detection = self.get_parameter('enable_parallel_frontier_detection').value
        self.parallel_computation_timeout = self.get_parameter('parallel_computation_timeout').value

        # 🛡️ 防抖动和容错参数
        self.min_goal_stay_time = 3.0  # 最小目标停留时间(秒)
        self.switch_confidence_threshold = 0.8  # 切换置信度阈值
        self.consecutive_checks_required = 3  # 需要连续检查次数
        self.max_switch_frequency = 0.2  # 最大切换频率(次/秒)

        # 🔄 切换状态跟踪
        self.goal_start_time = None  # 目标开始时间
        self.last_switch_time = 0.0  # 上次切换时间
        self.consecutive_switch_checks = 0  # 连续切换检查计数
        self.switch_conditions_history = []  # 切换条件历史

        # 🚀 增强的错误处理和恢复机制
        self.consecutive_failures = 0  # 连续失败计数
        self.last_successful_navigation = time.time()  # 上次成功导航时间
        self.navigation_success_rate = 1.0  # 导航成功率
        self.total_navigation_attempts = 0  # 总导航尝试次数
        self.successful_navigations = 0  # 成功导航次数
        
        # 初始化状态管理器
        self.state_manager = RobustStateManager(logger=self.get_logger())
        self.setup_state_machine()
        
        # 初始化优化前沿检测器
        self.frontier_detector = OptimizedFrontierDetector(
            map_resolution=0.03,
            robot_radius=self.robot_radius,
            min_frontier_size=self.min_frontier_size
        )

        # 🔍 初始化传感器探测范围检测器
        sensor_config = SensorConfig(
            max_range=10.0,  # 激光雷达最大范围
            min_range=0.1,   # 最小探测距离
            field_of_view=360.0,  # 360度激光雷达
            angular_resolution=1.0  # 与实际LaserScan分辨率(约1°)保持一致
        )
        self.sensor_detector = SensorRangeDetector(sensor_config)

        # 🔄 初始化并行计算管理器（减少线程数避免资源竞争）
        self.parallel_manager = ParallelComputationManager(self, max_workers=2)

        # 🔮 初始化预测性前沿点检测器
        self.predictive_detector = PredictiveFrontierDetector(self, self.parallel_manager)

        # 🏠 初始化房间感知探索器
        self.room_explorer = RoomAwareExplorer()
        self.current_exploration_strategy = ExplorationStrategy.FRONTIER_BASED
        self.room_centroids: Dict = {}
        self.room_stats: Dict = {}

        # 🔄 自适应参数管理器
        self.adaptive_params = {
            'exploration_interval': self.exploration_interval,
            'min_frontier_size': 3,
            'navigation_timeout': self.navigation_timeout,
            'goal_tolerance': self.goal_tolerance,
            'max_navigation_distance': self.max_navigation_distance
        }
        self.param_adaptation_history = []
        self.environment_metrics = {
            'exploration_efficiency': 0.0,
            'navigation_success_rate': 0.0,
            'average_frontier_count': 0.0,
            'map_complexity': 0.0
        }

        # 前沿评估参数（集成TAD算法）
        self.tad_weights = {
            'trapezoid': 0.25,
            'adjacent': 0.15,
            'distance': 0.15,
            'info_gain': 0.30,
            'reachability': 0.15
        }

        # 性能统计
        self.performance_stats = {
            'total_detections': 0,
            'successful_navigations': 0,
            'failed_navigations': 0,
            'average_detection_time': 0.0,
            'exploration_efficiency': 0.0
        }

        # 地图优化器（可选）
        self.enable_map_optimization = self.get_parameter('enable_map_optimization').value
        if self.enable_map_optimization:
            self.map_optimizer = AdaptiveMapOptimizer()
            self.get_logger().info('✅ 实时地图优化器已启用')
        else:
            self.map_optimizer = None
        
        # ROS2订阅和发布
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        
        # 导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # TF缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 状态变量
        self.map_data: Optional[OccupancyGrid] = None
        self.frontiers: List[OptimizedFrontierPoint] = []
        self.current_goal: Optional[OptimizedFrontierPoint] = None
        self.recent_goals: List[Tuple[float, float]] = []  # 记录最近的目标点，避免重复选择
        self.navigation_future = None
        self.navigation_goal_handle = None
        self.navigation_result_future = None
        self.navigation_start_time = None
        self.recovery_attempts = 0

        # 🎯 增强的智能完成判断状态
        self.last_completion_check_time = 0.0
        self.completion_check_interval = 1.0  # 每秒检查一次
        self.goal_approach_start_time = None
        self.early_completion_triggered = False

        # 🌟 多维度完成度评估状态
        self.completion_metrics = {
            'global_coverage': 0.0,
            'local_coverage': 0.0,
            'frontier_density': 0.0,
            'exploration_efficiency': 0.0,
            'room_completion_rate': 0.0,
            'detail_completion_score': 0.0
        }
        self.completion_history = []
        self.last_global_completion_check = 0.0

        # 🔄 动态前沿点更新状态
        self.last_map_data = None
        self.last_frontier_update_time = 0.0
        self.last_frontier_detection_time = 0.0
        self.map_change_detected = False
        self.known_pixels_count = 0
        self.total_map_pixels = 0

        # 🏠 简化的房间感知功能
        self.exploration_regions = {}  # 探索区域记录
        self.current_region_id = None  # 当前区域ID
        self.region_completion_threshold = 0.85  # 区域完成阈值
        self.last_region_check_time = 0.0  # 上次区域检查时间
        
        # 线程锁
        self.data_lock = threading.RLock()
        
        # 启动状态机
        self.state_manager.start('INITIALIZING')

        # 性能监控定时器
        self.performance_timer = self.create_timer(30.0, self.performance_monitor_callback)
        self.exploration_start_time = time.time()

        self.get_logger().info('🛡️ 稳定探索节点已启动')
    
    def setup_state_machine(self):
        """设置状态机"""
        
        # 定义状态
        states = [
            StateDefinition(
                name='INITIALIZING',
                state_type=StateType.NORMAL,
                entry_action=self.on_enter_initializing,
                update_action=self.update_initializing,
                timeout=30.0
            ),
            StateDefinition(
                name='IDLE',
                state_type=StateType.NORMAL,
                entry_action=self.on_enter_idle,
                update_action=self.update_idle,
                timeout=5.0
            ),
            StateDefinition(
                name='DETECTING_FRONTIERS',
                state_type=StateType.NORMAL,
                entry_action=self.on_enter_detecting_frontiers,
                update_action=self.update_detecting_frontiers,
                timeout=10.0
            ),
            StateDefinition(
                name='SELECTING_FRONTIER',
                state_type=StateType.NORMAL,
                entry_action=self.on_enter_selecting_frontier,
                update_action=self.update_selecting_frontier,
                timeout=5.0
            ),
            StateDefinition(
                name='NAVIGATING',
                state_type=StateType.NORMAL,
                entry_action=self.on_enter_navigating,
                update_action=self.update_navigating,
                exit_action=self.on_exit_navigating,
                timeout=120.0  # 增加到2分钟
            ),
            StateDefinition(
                name='RECOVERY',
                state_type=StateType.RECOVERY,
                entry_action=self.on_enter_recovery,
                update_action=self.update_recovery,
                timeout=30.0,
                max_retries=1
            ),
            StateDefinition(
                name='COMPLETED',
                state_type=StateType.TERMINAL,
                entry_action=self.on_enter_completed
            ),
            StateDefinition(
                name='FAILED',
                state_type=StateType.TERMINAL,
                entry_action=self.on_enter_failed
            )
        ]
        
        # 添加状态
        for state in states:
            self.state_manager.add_state(state)
        
        # 定义状态转换
        transitions = [
            # 初始化转换
            StateTransition('INITIALIZING', 'IDLE', TransitionTrigger.SUCCESS),
            StateTransition('INITIALIZING', 'FAILED', TransitionTrigger.TIMEOUT),
            StateTransition('INITIALIZING', 'FAILED', TransitionTrigger.ERROR),
            
            # 空闲状态转换
            StateTransition('IDLE', 'DETECTING_FRONTIERS', TransitionTrigger.SUCCESS),
            StateTransition('IDLE', 'COMPLETED', TransitionTrigger.TIMEOUT),
            
            # 前沿检测转换
            StateTransition('DETECTING_FRONTIERS', 'SELECTING_FRONTIER', TransitionTrigger.SUCCESS),
            StateTransition('DETECTING_FRONTIERS', 'COMPLETED', TransitionTrigger.FAILURE),
            StateTransition('DETECTING_FRONTIERS', 'RECOVERY', TransitionTrigger.ERROR),
            StateTransition('DETECTING_FRONTIERS', 'RECOVERY', TransitionTrigger.TIMEOUT),
            
            # 前沿选择转换
            StateTransition('SELECTING_FRONTIER', 'NAVIGATING', TransitionTrigger.SUCCESS),
            StateTransition('SELECTING_FRONTIER', 'IDLE', TransitionTrigger.FAILURE),
            StateTransition('SELECTING_FRONTIER', 'RECOVERY', TransitionTrigger.ERROR),
            
            # 导航转换
            StateTransition('NAVIGATING', 'IDLE', TransitionTrigger.SUCCESS),
            StateTransition('NAVIGATING', 'RECOVERY', TransitionTrigger.FAILURE),
            StateTransition('NAVIGATING', 'RECOVERY', TransitionTrigger.TIMEOUT),
            StateTransition('NAVIGATING', 'RECOVERY', TransitionTrigger.ERROR),
            
            # 恢复转换
            StateTransition('RECOVERY', 'IDLE', TransitionTrigger.SUCCESS),
            StateTransition('RECOVERY', 'FAILED', TransitionTrigger.FAILURE),
            StateTransition('RECOVERY', 'FAILED', TransitionTrigger.TIMEOUT),
        ]
        
        # 添加转换
        for transition in transitions:
            self.state_manager.add_transition(transition)
        
        # 添加回调
        self.state_manager.add_state_change_callback(self.on_state_change)
        self.state_manager.add_error_callback(self.on_error)
    
    # 状态进入动作
    def on_enter_initializing(self):
        """进入初始化状态"""
        self.get_logger().info('🔄 初始化探索系统...')
        self.recovery_attempts = 0
    
    def on_enter_idle(self):
        """进入空闲状态"""
        self.get_logger().info('😴 进入空闲状态，等待下一次探索')
    
    def on_enter_detecting_frontiers(self):
        """进入前沿检测状态"""
        self.get_logger().info('🔍 开始检测前沿点...')
    
    def on_enter_selecting_frontier(self):
        """进入前沿选择状态"""
        self.get_logger().info('🎯 选择最佳前沿点...')
    
    def on_enter_navigating(self):
        """进入导航状态"""
        self.navigation_start_time = time.time()
        self.get_logger().info('🚀 开始导航到目标前沿点...')
    
    def on_enter_recovery(self):
        """进入恢复状态"""
        self.recovery_attempts += 1
        self.get_logger().warn(f'🔧 进入恢复状态 (尝试 {self.recovery_attempts}/{self.max_recovery_attempts})')
    
    def on_enter_completed(self):
        """进入完成状态"""
        self.get_logger().info('🎉 探索任务完成！')
        self.stop_robot()
    
    def on_enter_failed(self):
        """进入失败状态"""
        self.get_logger().error('❌ 探索任务失败！')
        self.stop_robot()

        # 🔄 添加自动重启机制
        self.get_logger().info('🔄 10秒后尝试重启探索任务...')
        self.create_timer(10.0, self._attempt_restart_exploration)

    def _attempt_restart_exploration(self):
        """尝试重启探索任务"""
        try:
            self.get_logger().info('🔄 重启探索任务...')

            # 重置恢复计数器
            self.recovery_attempts = 0
            self.consecutive_failures = 0

            # 强制更新前沿点
            self._trigger_frontier_update()

            # 转换到IDLE状态重新开始
            self.state_manager.transition_to('IDLE')

        except Exception as e:
            self.get_logger().error(f'❌ 重启探索任务失败: {e}')

    def on_exit_navigating(self):
        """退出导航状态"""
        # 取消正在进行的导航
        if self.navigation_goal_handle:
            try:
                self.navigation_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'取消导航目标失败: {e}')

        if self.navigation_future and not self.navigation_future.done():
            self.navigation_future.cancel()

        # 🔄 检查是否有待处理的地图变化
        if self.map_change_detected:
            self.get_logger().info('🔄 导航结束，处理待更新的前沿点')
            self.map_change_detected = False
            # 强制触发前沿点更新
            self.last_frontier_detection_time = 0.0  # 重置检测时间以强制更新
    
    # 状态更新动作
    def update_initializing(self) -> str:
        """更新初始化状态"""
        if self.map_data is not None and self.nav_client.wait_for_server(timeout_sec=1.0):
            return "success"
        return "running"
    
    def update_idle(self) -> str:
        """更新空闲状态"""
        # 检查是否有地图数据
        if self.map_data is None:
            return "running"
        
        # 触发前沿检测
        return "success"
    
    def update_detecting_frontiers(self) -> str:
        """更新前沿检测状态"""
        try:
            with self.data_lock:
                if self.map_data is None:
                    return "failure"

                # 🚀 检查缓存是否有效
                current_time = time.time()
                if (self.frontier_cache is not None and
                    current_time - self.frontier_cache_timestamp < self.frontier_cache_timeout):
                    self.frontiers = self.frontier_cache
                    self.get_logger().debug(f'🚀 使用缓存的前沿点 ({len(self.frontiers)}个)')
                    return "success"

                # 🚀 执行前沿检测（支持并行计算）
                detection_start = time.time()

                if self.enable_parallel_frontier_detection:
                    # 获取机器人位置用于并行计算
                    robot_pos = self.get_robot_position()
                    if robot_pos:
                        # 提交并行前沿检测任务，使用包装方法
                        task_id = self.parallel_manager.submit_predictive_computation(
                            task_type='frontier_detection',
                            robot_position=robot_pos,
                            map_data=self.map_data,
                            computation_func=self._frontier_detection_wrapper,
                            priority=1,
                            room_info=None  # 暂时不传递房间信息
                        )

                        # 🔄 添加重试机制
                        max_retries = 2
                        retry_count = 0
                        parallel_success = False

                        while retry_count <= max_retries and not parallel_success:
                            # 等待结果
                            result = self.parallel_manager.get_computation_result(task_id, timeout=self.parallel_computation_timeout)

                            if result and hasattr(result, 'state'):
                                # 检查状态（使用正确的枚举比较）
                                try:
                                    state_ready = (result.state == ComputationState.READY)
                                except:
                                    # 回退到字符串比较
                                    state_ready = (str(result.state) == 'ComputationState.READY' or
                                                 result.state == 'ready')

                                if state_ready and result.result_data is not None:
                                    self.frontiers = result.result_data
                                    self.get_logger().debug('🚀 并行前沿检测成功')
                                    parallel_success = True
                                else:
                                    # 记录详细的失败信息
                                    error_msg = getattr(result, 'error_message', '未知错误')
                                    retry_count += 1
                                    if retry_count <= max_retries:
                                        self.get_logger().warn(f'🔄 并行计算失败(重试{retry_count}/{max_retries}): 状态={result.state}, 错误={error_msg}')
                                        # 重新提交任务
                                        task_id = self.parallel_manager.submit_predictive_computation(
                                            task_type='frontier_detection',
                                            robot_position=robot_pos,
                                            map_data=self.map_data,
                                            computation_func=self._frontier_detection_wrapper,
                                            priority=1,
                                            room_info=None
                                        )
                                    else:
                                        self.get_logger().warn(f'🔄 并行计算重试失败，回退到同步计算')
                                        self.frontiers = self.frontier_detector.detect_optimized_frontiers(self.map_data, None)
                                        parallel_success = True
                            else:
                                # 如果并行计算超时或返回None
                                retry_count += 1
                                if retry_count <= max_retries:
                                    self.get_logger().warn(f'🔄 并行计算超时(重试{retry_count}/{max_retries})，重新尝试')
                                    # 重新提交任务
                                    task_id = self.parallel_manager.submit_predictive_computation(
                                        task_type='frontier_detection',
                                        robot_position=robot_pos,
                                        map_data=self.map_data,
                                        computation_func=self._frontier_detection_wrapper,
                                        priority=1,
                                        room_info=None
                                    )
                                else:
                                    self.get_logger().warn(f'🔄 并行计算重试超时，回退到同步计算')
                                    self.frontiers = self.frontier_detector.detect_optimized_frontiers(self.map_data, None)
                                    parallel_success = True
                    else:
                        # 没有机器人位置，使用同步计算
                        self.frontiers = self.frontier_detector.detect_optimized_frontiers(self.map_data, None)
                else:
                    # 禁用并行计算，使用同步计算
                    self.frontiers = self.frontier_detector.detect_optimized_frontiers(self.map_data, None)

                detection_time = time.time() - detection_start

                # 🚀 更新前沿点缓存
                self.frontier_cache = self.frontiers.copy() if self.frontiers else []
                self.frontier_cache_timestamp = current_time

                # 🔄 记录前沿点检测时间
                self.last_frontier_detection_time = detection_start

                # 更新性能统计
                self.update_performance_stats('detection')
                self.performance_stats['average_detection_time'] = (
                    (self.performance_stats['average_detection_time'] * (self.performance_stats['total_detections'] - 1) + detection_time)
                    / self.performance_stats['total_detections']
                )

                # 发布前沿标记
                self.publish_frontier_markers()

                if self.frontiers:
                    # 详细的性能日志
                    parallel_used = self.enable_parallel_frontier_detection and hasattr(self, 'parallel_manager')
                    self.get_logger().info(
                        f'🔍 检测到 {len(self.frontiers)} 个前沿点 (耗时: {detection_time:.3f}s, 并行: {parallel_used})'
                    )

                    # 🔄 输出地图探索统计
                    if self.total_map_pixels > 0:
                        exploration_ratio = self.known_pixels_count / self.total_map_pixels
                        self.get_logger().info(f'🗺️ 地图探索进度: {exploration_ratio:.1%} ({self.known_pixels_count}/{self.total_map_pixels}像素)')

                    # 🌟 多维度完成度检查
                    completion_status = self._check_multidimensional_completion()
                    if completion_status['is_complete']:
                        self.get_logger().info(f'🌟 探索完成: {completion_status["reason"]} (置信度: {completion_status["confidence"]:.2f})')
                        return "complete"
                    elif completion_status['should_switch_area']:
                        self.get_logger().info('🏠 当前区域探索完成，建议切换到其他区域')

                    return "success"
                else:
                    self.get_logger().info('🏁 未检测到前沿点，探索可能已完成')

                    # 🏠 最后检查是否有其他区域可探索
                    if self._check_region_completion():
                        self.get_logger().info('🏠 当前区域已完成，但可能还有其他区域需要探索')

                    return "failure"

        except Exception as e:
            self.get_logger().error(f'❌ 前沿检测异常: {e}')
            import traceback
            self.get_logger().error(f'❌ 异常详情: {traceback.format_exc()}')
            return "error"
    
    def update_selecting_frontier(self) -> str:
        """更新前沿选择状态 - 集成预测性计算和TAD算法评估"""
        try:
            # 获取机器人位置
            robot_pos = self.get_robot_position()
            if not robot_pos:
                return "failure"

            # 🔮 首先尝试获取预测的目标点
            predicted_target = self.predictive_detector.get_best_predicted_target(
                robot_pos, self.evaluate_frontiers_with_tad
            )

            if predicted_target:
                self.current_goal = predicted_target
                # 🛡️ 重置目标开始时间和切换状态
                self.goal_start_time = time.time()
                self.consecutive_switch_checks = 0
                self.get_logger().info(
                    f'🔮 使用预测目标点: ({predicted_target.x:.2f}, {predicted_target.y:.2f})'
                )

                # 🔄 触发下一轮预测计算
                self._trigger_next_prediction()
                return "success"

            # 🔄 如果没有预测结果，使用传统方法
            if not self.frontiers:
                return "failure"

            # 🏠 更新房间探索信息
            self._update_room_exploration_info(robot_pos)

            # 🔄 更新自适应参数
            self._update_adaptive_parameters()

            # 🚀 并行前沿点评估
            best_frontier = self._parallel_frontier_evaluation(robot_pos)

            # 🔄 如果并行评估失败，回退到传统方法
            if not best_frontier:
                # 🎯 房间感知前沿点选择
                best_frontier = self._apply_room_aware_frontier_selection(self.frontiers, robot_pos)

                # 🔄 如果房间感知选择失败，回退到TAD算法
                if not best_frontier:
                    best_frontier = self.evaluate_frontiers_with_tad(robot_pos)

            if best_frontier:
                # 🎯 动态局部目标调整
                adjusted_frontier = self._adjust_local_goal_frontier(best_frontier, robot_pos)
                self.current_goal = adjusted_frontier

                # 🔄 记录最后选择的前沿点（用于多样性计算）
                self.last_selected_frontier = best_frontier

                # 🛡️ 重置目标开始时间和切换状态
                self.goal_start_time = time.time()
                self.consecutive_switch_checks = 0
                self.get_logger().info(f'🎯 TAD算法选择前沿点: ({best_frontier.x:.2f}, {best_frontier.y:.2f})')

                # 🔄 触发预测计算
                self._trigger_next_prediction()
                return "success"
            else:
                return "failure"

        except Exception as e:
            self.get_logger().error(f'❌ 前沿选择异常: {e}')
            import traceback
            self.get_logger().error(f'❌ 异常详情: {traceback.format_exc()}')
            return "error"
    
    def update_navigating(self) -> str:
        """更新导航状态"""
        try:
            # 🔄 更新机器人状态到预测器
            self._update_robot_state_for_prediction()

            # 第一步：如果还没有开始导航，开始导航
            if self.navigation_future is None:
                if not self.start_navigation():
                    return "failure"
                return "running"

            # 第二步：检查goal handle是否准备好
            if self.navigation_goal_handle is None:
                if self.navigation_future.done():
                    try:
                        goal_handle = self.navigation_future.result()
                        if goal_handle and goal_handle.accepted:
                            self.navigation_goal_handle = goal_handle
                            self.navigation_result_future = goal_handle.get_result_async()
                            self.get_logger().info('🎯 导航目标已被接受，开始执行')
                        else:
                            self.get_logger().warn('❌ 导航目标被拒绝')
                            self._cleanup_navigation()
                            return "failure"
                    except Exception as e:
                        self.get_logger().error(f'❌ 获取goal handle异常: {e}')
                        self._cleanup_navigation()
                        return "error"
                return "running"

            # 🚀 改进的导航超时检查
            if (self.navigation_start_time and
                time.time() - self.navigation_start_time > self.navigation_timeout):
                elapsed_time = time.time() - self.navigation_start_time
                self.get_logger().warn(f'⏰ 导航超时 ({elapsed_time:.1f}s > {self.navigation_timeout}s)')
                self._cleanup_navigation()
                self._record_navigation_failure()
                return "failure"

            # 🎯 智能完成检查（在检查导航结果之前）
            if self.early_completion_enabled and self._check_early_completion():
                elapsed_time = time.time() - self.navigation_start_time if self.navigation_start_time else 0
                self.get_logger().info(f'🎯 智能提前完成导航 (耗时: {elapsed_time:.1f}s)')
                self.update_performance_stats('navigation_success')
                self._add_to_recent_goals()  # 添加到最近目标列表
                self._remove_visited_frontier()  # 移除已访问的前沿点
                self._cleanup_navigation()
                self._record_navigation_success()
                return "success"

            # 第三步：检查导航结果
            if self.navigation_result_future and self.navigation_result_future.done():
                try:
                    result = self.navigation_result_future.result()
                    status = result.status

                    # 检查导航状态 (4 = SUCCESS)
                    if status == 4:
                        elapsed_time = time.time() - self.navigation_start_time if self.navigation_start_time else 0
                        self.get_logger().info(f'✅ 导航成功完成 (耗时: {elapsed_time:.1f}s)')
                        self.update_performance_stats('navigation_success')
                        self._add_to_recent_goals()  # 添加到最近目标列表
                        self._remove_visited_frontier()  # 移除已访问的前沿点
                        self._cleanup_navigation()
                        self._record_navigation_success()
                        return "success"
                    else:
                        elapsed_time = time.time() - self.navigation_start_time if self.navigation_start_time else 0
                        self.get_logger().warn(f'❌ 导航失败，状态码: {status} (耗时: {elapsed_time:.1f}s)')
                        self.update_performance_stats('navigation_failure')
                        self._cleanup_navigation()
                        self._record_navigation_failure()
                        return "failure"
                except Exception as e:
                    self.get_logger().error(f'❌ 处理导航结果异常: {e}')
                    self._cleanup_navigation()
                    return "error"

            return "running"

        except Exception as e:
            self.get_logger().error(f'❌ 导航更新异常: {e}')
            self._cleanup_navigation()
            self._record_navigation_failure()
            return "error"
    
    def update_recovery(self) -> str:
        """更新恢复状态"""
        try:
            # 停止机器人
            self.stop_robot()

            # 清空导航状态
            self._cleanup_navigation()

            # 检查恢复次数
            if self.recovery_attempts >= self.max_recovery_attempts:
                self.get_logger().error('❌ 超过最大恢复次数，任务失败')
                return "failure"

            # 等待一段时间后重试
            time.sleep(2.0)
            self.get_logger().info('🔄 恢复完成，准备重试')
            return "success"

        except Exception as e:
            self.get_logger().error(f'❌ 恢复异常: {e}')
            return "error"
    
    # 辅助方法
    def start_navigation(self) -> bool:
        """开始导航"""
        if not self.current_goal:
            return False
        
        try:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = self.current_goal.x
            goal_msg.pose.pose.position.y = self.current_goal.y
            goal_msg.pose.pose.orientation.w = 1.0
            
            self.navigation_future = self.nav_client.send_goal_async(goal_msg)
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 启动导航失败: {e}')
            return False
    
    def stop_robot(self):
        """停止机器人"""
        try:
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
        except Exception as e:
            self.get_logger().error(f'❌ 停止机器人失败: {e}')

    def _cleanup_navigation(self):
        """清理导航状态"""
        self.navigation_future = None
        self.navigation_goal_handle = None
        self.navigation_result_future = None
        self.navigation_start_time = None
        self.current_goal = None
        self.early_completion_triggered = False
        self.goal_approach_start_time = None

        # 清理状态变量
        pass  # 其他清理操作已在上面完成

    def _check_early_completion(self) -> bool:
        """
        🎯 智能提前完成检查 - 基于传感器探测范围的高效切换策略
        核心逻辑：当目标点已在探测范围内则触发更新下一个目标点
        集成防抖动机制和容错处理
        """
        if not self.current_goal or not self.map_data or self.early_completion_triggered:
            return False

        current_time = time.time()

        # 🛡️ 防抖动检查1：最小停留时间
        if self.goal_start_time is None:
            self.goal_start_time = current_time

        goal_stay_time = current_time - self.goal_start_time
        if goal_stay_time < self.min_goal_stay_time:
            return False

        # 🛡️ 防抖动检查2：切换频率限制
        if current_time - self.last_switch_time < (1.0 / self.max_switch_frequency):
            return False

        # 限制检查频率
        if current_time - self.last_completion_check_time < self.completion_check_interval:
            return False

        self.last_completion_check_time = current_time

        try:
            # 获取机器人当前位置和朝向
            robot_pos = self._get_robot_position()
            if not robot_pos:
                return False

            robot_yaw = self._get_robot_yaw()
            if robot_yaw is None:
                robot_yaw = 0.0  # 默认朝向

            # 🎯 核心策略：传感器探测范围检测
            target_pos = (self.current_goal.x, self.current_goal.y)
            visibility_result = self.sensor_detector.is_target_in_sensor_range(
                robot_pos, robot_yaw, target_pos, self.map_data
            )

            # 计算到目标的距离
            goal_distance = visibility_result.distance

            # 🛡️ 评估切换条件并记录历史
            switch_conditions = self._evaluate_switch_conditions(visibility_result, goal_distance)

            # 🛡️ 连续检查机制：需要连续满足条件才切换
            if switch_conditions['should_switch']:
                self.consecutive_switch_checks += 1
                self.switch_conditions_history.append(switch_conditions)

                # 保持历史记录长度
                if len(self.switch_conditions_history) > 10:
                    self.switch_conditions_history.pop(0)

                # 🎯 策略1：高置信度立即切换
                if (switch_conditions['confidence'] >= self.switch_confidence_threshold and
                    switch_conditions['reason'] == 'high_visibility'):
                    self.get_logger().info(
                        f'🎯 高置信度切换: {switch_conditions["reason"]}, '
                        f'置信度={switch_conditions["confidence"]:.2f}, 距离={goal_distance:.2f}m'
                    )
                    return self._execute_switch(current_time, switch_conditions)

                # 🎯 策略2：连续检查通过后切换
                elif self.consecutive_switch_checks >= self.consecutive_checks_required:
                    avg_confidence = sum(c['confidence'] for c in self.switch_conditions_history[-3:]) / 3
                    if avg_confidence >= 0.6:  # 平均置信度阈值
                        self.get_logger().info(
                            f'🎯 连续检查切换: {switch_conditions["reason"]}, '
                            f'连续次数={self.consecutive_switch_checks}, 平均置信度={avg_confidence:.2f}'
                        )
                        return self._execute_switch(current_time, switch_conditions)
            else:
                # 重置连续检查计数
                self.consecutive_switch_checks = 0

            # 🎯 策略3：目标点非常接近 - 强制切换
            if goal_distance <= self.goal_tolerance:
                self.get_logger().info(f'🎯 距离强制切换: {goal_distance:.2f}m ≤ {self.goal_tolerance:.2f}m')
                self.early_completion_triggered = True
                return True

            # 🎯 策略4：目标区域探索完成度检查 - 辅助切换
            if goal_distance <= self.min_approach_distance:
                if self.goal_approach_start_time is None:
                    self.goal_approach_start_time = current_time

                # 检查目标周围区域的探索完成度
                exploration_ratio = self._calculate_exploration_ratio_around_goal()

                # 🎯 结合可见性的动态阈值
                base_threshold = 0.6  # 基础阈值60%
                visibility_bonus = visibility_result.confidence * 0.2  # 可见性加成
                adjusted_threshold = base_threshold - visibility_bonus

                if exploration_ratio >= adjusted_threshold:
                    self.get_logger().info(
                        f'🎯 探索完成切换: 距离={goal_distance:.2f}m, '
                        f'探索完成度={exploration_ratio:.1%} ≥ {adjusted_threshold:.1%}, '
                        f'可见性={visibility_result.confidence:.2f}'
                    )
                    self.early_completion_triggered = True
                    return True

                # 🎯 策略5：时间触发（更激进的时间阈值）
                approach_time = current_time - self.goal_approach_start_time
                time_threshold = 8.0 if visibility_result.confidence < 0.3 else 4.0  # 可见性低时延长等待

                if approach_time > time_threshold and exploration_ratio >= 0.3:
                    self.get_logger().info(
                        f'🎯 时间触发切换: 接近时间={approach_time:.1f}s > {time_threshold:.1f}s, '
                        f'探索完成度={exploration_ratio:.1%}'
                    )
                    self.early_completion_triggered = True
                    return True
            else:
                # 重置接近时间
                self.goal_approach_start_time = None

            # 🎯 策略6：前沿点价值衰减检查 - 智能切换
            if goal_distance <= 5.0:  # 在合理距离内检查价值衰减
                frontier_value_decay = self._check_frontier_value_decay()
                if frontier_value_decay:
                    self.get_logger().info(
                        f'🎯 价值衰减切换: 前沿点价值下降, 距离={goal_distance:.2f}m'
                    )
                    self.early_completion_triggered = True
                    return True

            return False

        except Exception as e:
            self.get_logger().error(f'❌ 智能完成检查异常: {e}')
            return False

    def _evaluate_switch_conditions(self, visibility_result: VisibilityResult, goal_distance: float) -> dict:
        """
        🛡️ 评估切换条件并计算置信度

        Args:
            visibility_result: 可见性检测结果
            goal_distance: 到目标的距离

        Returns:
            dict: 包含切换决策信息的字典
        """
        conditions = {
            'should_switch': False,
            'reason': '',
            'confidence': 0.0,
            'details': {}
        }

        try:
            # 条件1：目标点在探测范围内且高可见性
            if visibility_result.is_visible and visibility_result.confidence > 0.7:
                conditions.update({
                    'should_switch': True,
                    'reason': 'high_visibility',
                    'confidence': visibility_result.confidence,
                    'details': {
                        'distance': goal_distance,
                        'clear_path_ratio': visibility_result.clear_path_ratio,
                        'occlusion_ratio': visibility_result.occlusion_ratio
                    }
                })
                return conditions

            # 条件2：目标点在探测范围内但部分遮挡
            if (visibility_result.distance <= self.sensor_detector.config.max_range and
                visibility_result.clear_path_ratio > 0.5 and
                goal_distance <= 3.0):
                conditions.update({
                    'should_switch': True,
                    'reason': 'partial_visibility',
                    'confidence': visibility_result.clear_path_ratio * 0.8,  # 降低置信度
                    'details': {
                        'distance': goal_distance,
                        'clear_path_ratio': visibility_result.clear_path_ratio
                    }
                })
                return conditions

            # 条件3：非常接近目标点
            if goal_distance <= self.goal_tolerance:
                conditions.update({
                    'should_switch': True,
                    'reason': 'close_distance',
                    'confidence': max(0.9, 1.0 - goal_distance / self.goal_tolerance),
                    'details': {'distance': goal_distance}
                })
                return conditions

            # 条件4：探索完成度检查
            if goal_distance <= self.min_approach_distance:
                exploration_ratio = self._calculate_exploration_ratio_around_goal()
                base_threshold = 0.6
                visibility_bonus = visibility_result.confidence * 0.2
                adjusted_threshold = base_threshold - visibility_bonus

                if exploration_ratio >= adjusted_threshold:
                    conditions.update({
                        'should_switch': True,
                        'reason': 'exploration_complete',
                        'confidence': exploration_ratio * 0.9,
                        'details': {
                            'exploration_ratio': exploration_ratio,
                            'threshold': adjusted_threshold
                        }
                    })
                    return conditions

            return conditions

        except Exception as e:
            self.get_logger().error(f'❌ 切换条件评估异常: {e}')
            return conditions

    def _execute_switch(self, current_time: float, switch_conditions: dict) -> bool:
        """
        🎯 执行目标切换

        Args:
            current_time: 当前时间
            switch_conditions: 切换条件信息

        Returns:
            bool: 是否成功执行切换
        """
        try:
            # 更新切换状态
            self.early_completion_triggered = True
            self.last_switch_time = current_time
            self.consecutive_switch_checks = 0
            self.goal_start_time = None  # 重置目标开始时间

            # 记录切换信息
            self.get_logger().info(
                f'🎯 执行目标切换: 原因={switch_conditions["reason"]}, '
                f'置信度={switch_conditions["confidence"]:.2f}, '
                f'详情={switch_conditions["details"]}'
            )

            return True

        except Exception as e:
            self.get_logger().error(f'❌ 执行切换异常: {e}')
            return False

    def _trigger_next_prediction(self):
        """触发下一轮预测计算"""
        try:
            if self.map_data and self.current_goal:
                # 更新机器人状态到预测器
                robot_pos = self.get_robot_position()
                robot_yaw = self._get_robot_yaw()

                if robot_pos and robot_yaw is not None:
                    # 获取机器人速度（如果有的话）
                    velocity = getattr(self, 'robot_velocity', None)

                    self.predictive_detector.update_robot_state(
                        robot_pos, robot_yaw, velocity
                    )

                    # 触发预测性检测
                    task_id = self.predictive_detector.trigger_predictive_detection(
                        self.map_data, self.current_goal
                    )

                    if task_id:
                        self.get_logger().debug(f'🔮 触发预测计算: {task_id}')

        except Exception as e:
            self.get_logger().error(f'❌ 触发预测计算异常: {e}')

    def _update_robot_state_for_prediction(self):
        """更新机器人状态用于预测计算"""
        try:
            robot_pos = self.get_robot_position()
            robot_yaw = self._get_robot_yaw()

            if robot_pos and robot_yaw is not None:
                # 获取机器人速度（如果订阅了速度话题）
                velocity = getattr(self, 'robot_velocity', None)

                self.predictive_detector.update_robot_state(
                    robot_pos, robot_yaw, velocity
                )

        except Exception as e:
            self.get_logger().debug(f'更新机器人状态异常: {e}')

    def _frontier_detection_wrapper(self, map_data, room_info=None, **kwargs):
        """前沿点检测包装器，只传递需要的参数"""
        return self.frontier_detector.detect_optimized_frontiers(map_data, room_info)

    def _safe_log(self, level: str, message: str):
        """安全的日志记录，避免ROS上下文失效问题"""
        try:
            if level == 'info':
                self.get_logger().info(message)
            elif level == 'warn':
                self.get_logger().warn(message)
            elif level == 'error':
                self.get_logger().error(message)
            elif level == 'debug':
                self.get_logger().debug(message)
        except Exception:
            # 如果ROS日志失败，使用print作为备选
            print(f"[{level.upper()}] {message}")

    def destroy_node(self):
        """销毁节点时清理资源"""
        try:
            # 设置关闭标志
            self._shutdown_requested = True

            # 记录关闭开始（使用print避免ROS上下文问题）
            print("🔄 开始关闭探索节点...")

            # 停止所有定时器（优先级最高）
            if hasattr(self, 'exploration_timer') and self.exploration_timer:
                self.exploration_timer.cancel()
                print("✅ 探索定时器已停止")

            # 停止状态机
            if hasattr(self, 'state_machine') and self.state_machine:
                try:
                    self.state_machine.stop()
                    print("✅ 状态机已停止")
                except Exception as e:
                    print(f"⚠️ 状态机停止异常: {e}")

            # 关闭并行计算管理器
            if hasattr(self, 'parallel_manager') and self.parallel_manager:
                try:
                    self.parallel_manager.shutdown()
                    print("✅ 并行计算管理器已关闭")
                except Exception as e:
                    print(f"⚠️ 并行计算管理器关闭异常: {e}")

            # 等待短时间确保所有异步操作完成
            import time
            time.sleep(0.1)

            # 调用父类的销毁方法
            super().destroy_node()
            print("✅ 节点销毁完成")

        except Exception as e:
            # 使用print而不是logger，避免ROS上下文问题
            print(f'❌ 节点销毁异常: {e}')
            import traceback
            print(f'❌ 详细错误: {traceback.format_exc()}')

    def _calculate_exploration_ratio_around_goal(self) -> float:
        """
        计算目标点周围区域的探索完成度
        返回已探索区域的比例 (0.0 - 1.0)
        """
        if not self.current_goal or not self.map_data:
            return 0.0

        try:
            # 将目标点转换为地图坐标
            goal_map_x = int((self.current_goal.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            goal_map_y = int((self.current_goal.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 计算检查半径（地图像素）
            radius_pixels = int(self.completion_check_radius / self.map_data.info.resolution)

            # 统计区域内的像素
            total_pixels = 0
            known_pixels = 0  # 已知区域（自由空间 + 障碍物）

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-radius_pixels, radius_pixels + 1):
                for dx in range(-radius_pixels, radius_pixels + 1):
                    # 检查是否在圆形区域内
                    if dx*dx + dy*dy <= radius_pixels*radius_pixels:
                        map_x = goal_map_x + dx
                        map_y = goal_map_y + dy

                        # 检查边界
                        if 0 <= map_x < map_width and 0 <= map_y < map_height:
                            total_pixels += 1

                            # 获取像素值
                            pixel_index = map_y * map_width + map_x
                            if pixel_index < len(self.map_data.data):
                                pixel_value = self.map_data.data[pixel_index]

                                # 已知区域：0(自由) 或 100(障碍)，未知区域：-1
                                if pixel_value != -1:
                                    known_pixels += 1

            if total_pixels == 0:
                return 0.0

            exploration_ratio = known_pixels / total_pixels

            # 调试日志（仅在接近目标时输出）
            robot_pos = self._get_robot_position()
            if robot_pos:
                goal_distance = math.sqrt(
                    (self.current_goal.x - robot_pos[0])**2 +
                    (self.current_goal.y - robot_pos[1])**2
                )
                if goal_distance <= self.min_approach_distance:
                    self.get_logger().debug(
                        f'🎯 目标区域探索度: {exploration_ratio:.1%} '
                        f'({known_pixels}/{total_pixels}像素, 半径{self.completion_check_radius:.1f}m)'
                    )

            return exploration_ratio

        except Exception as e:
            self.get_logger().error(f'❌ 计算探索完成度异常: {e}')
            return 0.0

    def _calculate_information_gain_at_goal(self) -> float:
        """
        🎯 计算目标点的信息增益
        基于目标点周围未知区域的密度和可观测性
        """
        if not self.current_goal or not self.map_data:
            return 0.0

        try:
            # 转换目标点到地图坐标
            goal_map_x = int((self.current_goal.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            goal_map_y = int((self.current_goal.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 信息增益检查半径（更大范围）
            gain_radius_pixels = int(3.0 / self.map_data.info.resolution)  # 3米半径

            total_cells = 0
            unknown_cells = 0
            frontier_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-gain_radius_pixels, gain_radius_pixels + 1):
                for dx in range(-gain_radius_pixels, gain_radius_pixels + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= gain_radius_pixels:
                        check_x = goal_map_x + dx
                        check_y = goal_map_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_cells += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                cell_value = self.map_data.data[pixel_index]

                                if cell_value == -1:  # 未知区域
                                    unknown_cells += 1
                                    # 检查是否是前沿点（未知区域旁边有已知区域）
                                    if self._is_frontier_cell_at_position(check_x, check_y):
                                        frontier_cells += 1

            if total_cells == 0:
                return 0.0

            # 计算信息增益：未知区域比例 + 前沿点密度加权
            unknown_ratio = unknown_cells / total_cells
            frontier_density = frontier_cells / max(unknown_cells, 1)

            # 综合信息增益评分
            information_gain = unknown_ratio * 0.7 + frontier_density * 0.3

            return min(information_gain, 1.0)

        except Exception as e:
            self.get_logger().error(f'❌ 计算信息增益异常: {e}')
            return 0.0

    def _calculate_local_coverage_around_goal(self) -> float:
        """
        🎯 计算目标点周围的局部覆盖度
        专注于机器人传感器范围内的覆盖情况
        """
        if not self.current_goal or not self.map_data:
            return 0.0

        try:
            # 转换目标点到地图坐标
            goal_map_x = int((self.current_goal.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            goal_map_y = int((self.current_goal.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 传感器范围（激光雷达范围）
            sensor_range_pixels = int(2.5 / self.map_data.info.resolution)  # 2.5米传感器范围

            total_observable = 0
            covered_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-sensor_range_pixels, sensor_range_pixels + 1):
                for dx in range(-sensor_range_pixels, sensor_range_pixels + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= sensor_range_pixels:
                        check_x = goal_map_x + dx
                        check_y = goal_map_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_observable += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                cell_value = self.map_data.data[pixel_index]

                                # 已知区域（自由空间或障碍物）
                                if cell_value != -1:
                                    covered_cells += 1

            if total_observable > 0:
                return covered_cells / total_observable
            else:
                return 0.0

        except Exception as e:
            self.get_logger().error(f'❌ 计算局部覆盖度异常: {e}')
            return 0.0

    def _check_frontier_value_decay(self) -> bool:
        """
        🎯 检查前沿点价值是否衰减
        通过比较当前前沿点与其他可用前沿点的价值
        """
        if not self.current_goal or not hasattr(self, 'frontiers') or not self.frontiers:
            return False

        try:
            # 获取机器人位置
            robot_pos = self._get_robot_position()
            if not robot_pos:
                return False

            # 重新评估当前目标的价值
            current_goal_score = self._evaluate_frontier_score(self.current_goal, robot_pos)

            # 评估其他前沿点的价值
            better_frontiers = 0
            total_frontiers = 0

            for frontier in self.frontiers:
                if frontier != self.current_goal:
                    frontier_score = self._evaluate_frontier_score(frontier, robot_pos)
                    total_frontiers += 1

                    # 如果其他前沿点的价值显著更高
                    if frontier_score > current_goal_score * 1.3:  # 30%更高
                        better_frontiers += 1

            # 如果超过50%的前沿点价值更高，认为当前目标价值衰减
            if total_frontiers > 0 and better_frontiers / total_frontiers > 0.5:
                return True

            return False

        except Exception as e:
            self.get_logger().error(f'❌ 检查前沿点价值衰减异常: {e}')
            return False

    def _is_frontier_cell_at_position(self, x: int, y: int) -> bool:
        """检查指定位置是否为前沿点单元格"""
        try:
            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            pixel_index = y * map_width + x
            if pixel_index >= len(self.map_data.data):
                return False

            if self.map_data.data[pixel_index] != -1:  # 不是未知区域
                return False

            # 检查8邻域是否有已知区域
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue

                    nx, ny = x + dx, y + dy
                    if (0 <= nx < map_width and 0 <= ny < map_height):
                        neighbor_index = ny * map_width + nx
                        if neighbor_index < len(self.map_data.data):
                            if self.map_data.data[neighbor_index] == 0:  # 自由空间
                                return True
            return False
        except Exception:
            return False

    def _evaluate_frontier_score(self, frontier, robot_pos) -> float:
        """评估前沿点的综合得分"""
        if not frontier or not robot_pos:
            return 0.0

        # 距离因子
        distance = math.sqrt(
            (frontier.x - robot_pos[0])**2 +
            (frontier.y - robot_pos[1])**2
        )
        distance_score = 1.0 / (1.0 + distance * 0.1)

        # 信息增益因子（如果前沿点有质量评分）
        info_gain_score = getattr(frontier, 'quality_score', 0.5)

        # 综合评分
        total_score = distance_score * 0.4 + info_gain_score * 0.6

        return total_score

    def _get_robot_position(self) -> Optional[Tuple[float, float]]:
        """获取机器人当前位置"""
        try:
            # 获取机器人在地图坐标系中的位置
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return (x, y)

        except Exception as e:
            self.get_logger().debug(f'获取机器人位置失败: {e}')
            return None

    def _get_robot_yaw(self) -> Optional[float]:
        """获取机器人当前朝向角度(弧度)"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            # 从四元数转换为欧拉角
            quat = transform.transform.rotation
            # 计算yaw角度
            siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
            cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return yaw
        except Exception as e:
            self.get_logger().debug(f'获取机器人朝向失败: {e}')
            return None



    def _check_map_changes(self, old_map: OccupancyGrid, new_map: OccupancyGrid):
        """
        🔄 检测地图变化并触发前沿点更新
        """
        try:
            current_time = time.time()

            # 限制检查频率
            if current_time - self.last_frontier_update_time < self.frontier_update_interval:
                return

            # 检查地图尺寸是否一致
            if (old_map.info.width != new_map.info.width or
                old_map.info.height != new_map.info.height):
                self.get_logger().info('🔄 地图尺寸变化，触发前沿点更新')
                self._trigger_frontier_update()
                return

            # 计算地图变化程度
            old_data = np.array(old_map.data)
            new_data = np.array(new_map.data)

            # 统计已知像素（非-1的像素）
            old_known = np.sum(old_data != -1)
            new_known = np.sum(new_data != -1)

            # 计算新探索的像素数量
            newly_explored = new_known - old_known
            total_pixels = len(new_data)

            # 更新统计信息
            self.known_pixels_count = new_known
            self.total_map_pixels = total_pixels

            if newly_explored > 0:
                change_ratio = newly_explored / total_pixels

                self.get_logger().debug(
                    f'🔄 地图变化检测: 新探索像素={newly_explored}, '
                    f'变化比例={change_ratio:.3f}, 阈值={self.map_change_threshold:.3f}'
                )

                # 如果变化超过阈值，触发前沿点更新
                if change_ratio >= self.map_change_threshold:
                    self.get_logger().info(
                        f'🔄 地图显著变化({change_ratio:.1%})，触发前沿点更新'
                    )
                    self._trigger_frontier_update()
                    return

            # 检查强制更新超时（仅在非终端状态下执行）
            current_state = self.state_manager.get_current_state()
            if current_state not in ['FAILED', 'COMPLETED']:
                time_since_last_detection = current_time - self.last_frontier_detection_time
                if time_since_last_detection > self.force_update_timeout:
                    self.get_logger().info(
                        f'🔄 前沿点检测超时({time_since_last_detection:.1f}s)，强制更新'
                    )
                    self._trigger_frontier_update()

        except Exception as e:
            self.get_logger().error(f'❌ 地图变化检测异常: {e}')

    def _trigger_frontier_update(self):
        """
        🔄 触发前沿点更新
        """
        try:
            current_time = time.time()
            self.last_frontier_update_time = current_time

            # 如果当前正在导航，标记需要更新但不立即执行
            if self.state_manager.current_state == 'NAVIGATING':
                self.map_change_detected = True
                self.get_logger().info('🔄 导航中检测到地图变化，标记待更新')
                return

            # 如果当前在空闲状态，立即触发前沿点检测
            if self.state_manager.current_state == 'IDLE':
                self.get_logger().info('🔄 触发立即前沿点检测')
                # 触发成功转换到检测状态
                from aurora_explorer.robust_state_manager import TransitionTrigger
                self.state_manager.trigger_transition(TransitionTrigger.SUCCESS)

        except Exception as e:
            self.get_logger().error(f'❌ 触发前沿点更新异常: {e}')
    
    def publish_frontier_markers(self):
        """发布前沿标记"""
        try:
            # 检查发布器是否有效
            if not hasattr(self, 'marker_publisher') or self.marker_publisher is None:
                return

            marker_array = MarkerArray()

            for i, frontier in enumerate(self.frontiers):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'frontiers'
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = frontier.x
                marker.pose.position.y = frontier.y
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

            # 安全发布
            try:
                self.marker_publisher.publish(marker_array)
            except Exception as pub_error:
                # 如果发布失败，可能是上下文无效，忽略错误
                if "context is invalid" not in str(pub_error):
                    raise pub_error

        except Exception as e:
            # 只记录非上下文无效的错误
            if "context is invalid" not in str(e):
                self.get_logger().error(f'❌ 发布前沿标记失败: {e}')
    
    # 回调函数
    def map_callback(self, msg: OccupancyGrid):
        """地图回调 - 集成地图优化和动态前沿点更新"""
        with self.data_lock:
            # 保存上一帧地图用于变化检测
            previous_map = self.map_data

            # 应用地图优化（如果启用）
            if self.map_optimizer:
                try:
                    optimized_msg = self.map_optimizer.optimize_map(msg)
                    self.map_data = optimized_msg
                    self.get_logger().debug('🗺️ 地图已优化')
                except Exception as e:
                    self.get_logger().warn(f'地图优化失败: {e}')
                    self.map_data = msg
            else:
                self.map_data = msg

            self.frontier_detector.map_resolution = self.map_data.info.resolution

            # 🔄 检测地图变化并触发动态前沿点更新
            if self.dynamic_frontier_update and previous_map is not None:
                self._check_map_changes(previous_map, self.map_data)
    
    def on_state_change(self, old_state: str, new_state: str):
        """状态变化回调"""
        self.get_logger().info(f'🔄 状态变化: {old_state} -> {new_state}')
    
    def on_error(self, error: Exception):
        """错误回调"""
        self.get_logger().error(f'❌ 系统错误: {error}')

    # TAD算法评估方法
    def evaluate_frontiers_with_tad(self, robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """使用增强TAD算法评估前沿点"""
        try:
            # 🎯 多层次前沿点评估
            frontier_scores = []
            filtered_count = 0
            total_count = len(self.frontiers)

            # 🔄 第一阶段：基础过滤和评分
            for frontier in self.frontiers:
                # 🎯 动态距离过滤：跳过过远的前沿点
                distance_to_frontier = math.sqrt(
                    (frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2
                )
                max_distance = self._calculate_dynamic_max_distance(robot_pos)
                if distance_to_frontier > max_distance:
                    filtered_count += 1
                    continue

                # 🔍 局部前沿点质量筛选
                if not self._is_local_frontier_viable(frontier, robot_pos, distance_to_frontier):
                    filtered_count += 1
                    continue

                # 🚫 避免重复选择最近访问过的前沿点
                is_recent = False
                for recent_goal in self.recent_goals:
                    recent_distance = math.sqrt(
                        (frontier.x - recent_goal[0])**2 + (frontier.y - recent_goal[1])**2
                    )
                    if recent_distance < 1.5:  # 1.5米内认为是重复目标
                        is_recent = True
                        break

                if is_recent:
                    filtered_count += 1
                    continue

                # TAD算法三个参数评估
                trapezoid_score = self.calculate_trapezoid_parameter(frontier, robot_pos)
                adjacent_score = self.calculate_adjacent_parameter(frontier)
                distance_score = self.calculate_distance_parameter(frontier, robot_pos)

                # 信息增益和可达性评估
                info_gain = self.calculate_information_gain(frontier)
                reachability_score = self.calculate_reachability_score(frontier, robot_pos)

                # 🎯 增强的综合评分
                base_score = (
                    trapezoid_score * self.tad_weights['trapezoid'] +
                    adjacent_score * self.tad_weights['adjacent'] +
                    distance_score * self.tad_weights['distance'] +
                    info_gain * self.tad_weights['info_gain'] +
                    reachability_score * self.tad_weights['reachability']
                )

                # 🌟 多层次优先级加权
                exploration_priority = self._calculate_exploration_priority(frontier, robot_pos)
                global_coverage_bonus = self._calculate_global_coverage_bonus(frontier)

                # 最终评分
                final_score = base_score * (1.0 + exploration_priority * 0.3 + global_coverage_bonus * 0.2)

                frontier_scores.append((frontier, final_score, {
                    'base_score': base_score,
                    'exploration_priority': exploration_priority,
                    'global_coverage_bonus': global_coverage_bonus,
                    'distance': distance_to_frontier
                }))

            # 🏆 第二阶段：全局排序和最优选择
            if frontier_scores:
                # 按评分排序
                frontier_scores.sort(key=lambda x: x[1], reverse=True)

                # 🎯 智能选择策略
                best_frontier = self._select_optimal_frontier(frontier_scores, robot_pos)

                if best_frontier:
                    best_score = frontier_scores[0][1]
                    self.get_logger().info(f'🎯 最佳前沿评分: {best_score:.3f}')

                    # 📊 记录评分详情
                    best_details = frontier_scores[0][2]
                    self.get_logger().debug(f'📊 评分详情: {best_details}')
                else:
                    self.get_logger().warning(f'⚠️ 智能选择未找到合适前沿点')
            else:
                best_frontier = None
                self.get_logger().warning(f'⚠️ 未找到合适的前沿点 (总数: {total_count}, 距离过滤: {filtered_count})')

            return best_frontier

        except Exception as e:
            self.get_logger().error(f'❌ TAD评估异常: {e}')
            # 回退到简单选择
            return self.frontiers[0] if self.frontiers else None

    def _calculate_exploration_priority(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> float:
        """计算探索优先级"""
        try:
            # 🎯 基于探索历史的优先级
            history_priority = self._get_exploration_history_priority(frontier)

            # 🌍 基于全局探索状态的优先级
            global_priority = self._get_global_exploration_priority(frontier)

            # 🏠 基于房间结构的优先级
            room_priority = self._get_room_structure_priority(frontier)

            return (history_priority * 0.4 + global_priority * 0.4 + room_priority * 0.2)
        except Exception:
            return 0.5

    def _calculate_global_coverage_bonus(self, frontier: OptimizedFrontierPoint) -> float:
        """计算全局覆盖度奖励"""
        try:
            # 🗺️ 计算当前全局探索覆盖度
            if not self.map_data:
                return 0.0

            total_cells = len(self.map_data.data)
            known_cells = sum(1 for cell in self.map_data.data if cell != -1)
            coverage_ratio = known_cells / max(total_cells, 1)

            # 🎯 根据覆盖度调整奖励策略
            if coverage_ratio < 0.3:
                # 早期探索：优先大前沿点
                return getattr(frontier, 'size', 1.0) / 10.0
            elif coverage_ratio < 0.7:
                # 中期探索：平衡大小和位置
                size_bonus = getattr(frontier, 'size', 1.0) / 20.0
                position_bonus = self._calculate_position_diversity_bonus(frontier)
                return size_bonus + position_bonus
            else:
                # 后期探索：优先细节完善
                return self._calculate_detail_completion_bonus(frontier)
        except Exception:
            return 0.0

    def _select_optimal_frontier(self, frontier_scores: list, robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """智能选择最优前沿点"""
        try:
            if not frontier_scores:
                return None

            # 🎯 多策略选择
            top_candidates = frontier_scores[:min(3, len(frontier_scores))]

            # 🔄 动态选择策略
            if len(top_candidates) == 1:
                return top_candidates[0][0]

            # 📊 考虑多个因素的最终选择
            best_candidate = None
            best_final_score = -float('inf')

            for frontier, score, details in top_candidates:
                # 🎯 最终评估因子
                distance_factor = 1.0 / (1.0 + details['distance'] / 10.0)  # 距离因子
                diversity_factor = self._calculate_selection_diversity(frontier)  # 多样性因子

                final_score = score * distance_factor * diversity_factor

                if final_score > best_final_score:
                    best_final_score = final_score
                    best_candidate = frontier

            return best_candidate
        except Exception:
            return frontier_scores[0][0] if frontier_scores else None

    def _get_exploration_history_priority(self, frontier: OptimizedFrontierPoint) -> float:
        """获取基于探索历史的优先级"""
        try:
            # 检查是否远离最近访问的区域
            if hasattr(self, 'recent_goals') and self.recent_goals:
                min_distance = float('inf')
                for recent_goal in self.recent_goals[-5:]:  # 最近5个目标
                    distance = math.sqrt(
                        (frontier.x - recent_goal[0])**2 +
                        (frontier.y - recent_goal[1])**2
                    )
                    min_distance = min(min_distance, distance)

                # 距离越远，优先级越高
                return min(min_distance / 8.0, 1.0)
            return 0.8
        except Exception:
            return 0.5

    def _get_global_exploration_priority(self, frontier: OptimizedFrontierPoint) -> float:
        """获取全局探索优先级"""
        try:
            # 基于前沿点在全局探索中的重要性
            if not self.map_data:
                return 0.5

            # 计算前沿点周围的未探索密度
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 大范围检查（5米半径）
            radius_pixels = int(5.0 / self.map_data.info.resolution)
            unknown_density = self._calculate_unknown_density_in_radius(map_x, map_y, radius_pixels)

            return min(unknown_density * 2.0, 1.0)
        except Exception:
            return 0.5

    def _get_room_structure_priority(self, frontier: OptimizedFrontierPoint) -> float:
        """获取房间结构优先级"""
        try:
            # 简化的房间结构评估
            # 检查前沿点是否可能连接不同的房间区域
            connectivity_score = self._assess_room_connectivity(frontier)
            boundary_score = self._assess_room_boundary_proximity(frontier)

            return (connectivity_score * 0.6 + boundary_score * 0.4)
        except Exception:
            return 0.5

    def _calculate_position_diversity_bonus(self, frontier: OptimizedFrontierPoint) -> float:
        """计算位置多样性奖励"""
        try:
            # 检查前沿点是否在探索地图的边缘或角落
            if not self.map_data:
                return 0.0

            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 计算到地图边界的距离
            edge_distance = min(
                map_x,
                map_y,
                self.map_data.info.width - map_x,
                self.map_data.info.height - map_y
            )

            # 边缘位置获得更高奖励
            edge_bonus = max(0, (50 - edge_distance) / 50.0)
            return min(edge_bonus, 0.3)
        except Exception:
            return 0.0

    def _calculate_detail_completion_bonus(self, frontier: OptimizedFrontierPoint) -> float:
        """计算细节完善奖励"""
        try:
            # 后期探索阶段，优先小而精确的前沿点
            size = getattr(frontier, 'size', 1.0)

            # 小前沿点在后期获得更高奖励
            detail_bonus = max(0, (5.0 - size) / 5.0)
            return min(detail_bonus, 0.5)
        except Exception:
            return 0.0

    def _calculate_selection_diversity(self, frontier: OptimizedFrontierPoint) -> float:
        """计算选择多样性"""
        try:
            # 避免连续选择相似位置的前沿点
            if hasattr(self, 'last_selected_frontier') and self.last_selected_frontier:
                distance = math.sqrt(
                    (frontier.x - self.last_selected_frontier.x)**2 +
                    (frontier.y - self.last_selected_frontier.y)**2
                )
                # 距离上次选择点越远，多样性越高
                diversity = min(distance / 5.0, 1.0)
                return 0.5 + diversity * 0.5
            return 1.0
        except Exception:
            return 1.0

    def _calculate_unknown_density_in_radius(self, center_x: int, center_y: int, radius: int) -> float:
        """计算指定半径内的未知区域密度"""
        try:
            total_cells = 0
            unknown_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= radius:
                        check_x = center_x + dx
                        check_y = center_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_cells += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                if self.map_data.data[pixel_index] == -1:  # 未知区域
                                    unknown_cells += 1

            return unknown_cells / max(total_cells, 1)
        except Exception:
            return 0.0

    def _assess_room_connectivity(self, frontier: OptimizedFrontierPoint) -> float:
        """评估房间连通性"""
        try:
            # 简化的连通性评估：检查前沿点是否在通道或门洞附近
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 检查周围的空间结构
            passage_score = self._detect_narrow_passage(map_x, map_y)

            # 通道附近的前沿点具有更高的连通性价值
            return min(passage_score * 1.5, 1.0)
        except Exception:
            return 0.5

    def _assess_room_boundary_proximity(self, frontier: OptimizedFrontierPoint) -> float:
        """评估房间边界邻近性"""
        try:
            # 检查前沿点是否靠近房间边界
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 检查周围的障碍物密度
            obstacle_density = self._calculate_obstacle_density(map_x, map_y, 10)  # 10像素半径

            # 适中的障碍物密度表示可能在房间边界附近
            if 0.2 <= obstacle_density <= 0.6:
                return 0.8
            else:
                return 0.3
        except Exception:
            return 0.5

    def _calculate_obstacle_density(self, center_x: int, center_y: int, radius: int) -> float:
        """计算指定半径内的障碍物密度"""
        try:
            total_cells = 0
            obstacle_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= radius:
                        check_x = center_x + dx
                        check_y = center_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_cells += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                if self.map_data.data[pixel_index] == 100:  # 障碍物
                                    obstacle_cells += 1

            return obstacle_cells / max(total_cells, 1)
        except Exception:
            return 0.0

    def _calculate_dynamic_max_distance(self, robot_pos: Tuple[float, float]) -> float:
        """计算动态最大探索距离"""
        try:
            # 🎯 基于当前探索状态调整最大距离
            if not self.map_data:
                return self.max_navigation_distance

            # 计算当前探索覆盖度
            total_cells = len(self.map_data.data)
            known_cells = sum(1 for cell in self.map_data.data if cell != -1)
            coverage_ratio = known_cells / max(total_cells, 1)

            # 🔄 动态调整策略
            if coverage_ratio < 0.2:
                # 早期探索：允许更远距离
                return self.max_navigation_distance * 1.2
            elif coverage_ratio < 0.6:
                # 中期探索：标准距离
                return self.max_navigation_distance
            else:
                # 后期探索：缩短距离，专注细节
                return self.max_navigation_distance * 0.8
        except Exception:
            return self.max_navigation_distance

    def _is_local_frontier_viable(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float], distance: float) -> bool:
        """判断局部前沿点是否可行"""
        try:
            # 🎯 多维度局部前沿点质量评估

            # 1. 大小检查：过小的前沿点在近距离时不值得探索
            if distance < 3.0 and hasattr(frontier, 'size') and frontier.size < 2:
                return False

            # 2. 局部信息增益检查
            local_info_gain = self._calculate_local_information_gain(frontier)
            if local_info_gain < 0.1:  # 局部信息增益过低
                return False

            # 3. 路径可达性预检查
            if not self._is_locally_reachable(frontier, robot_pos):
                return False

            # 4. 局部障碍物密度检查
            if self._has_excessive_local_obstacles(frontier):
                return False

            return True
        except Exception:
            return True  # 默认认为可行

    def _is_locally_reachable(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> bool:
        """检查局部可达性"""
        try:
            # 简化的局部可达性检查：直线路径上是否有明显障碍
            if not self.map_data:
                return True

            # 采样检查直线路径
            steps = int(math.sqrt((frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2) / 0.2)
            steps = max(steps, 5)

            for i in range(1, steps):
                t = i / steps
                check_x = robot_pos[0] + t * (frontier.x - robot_pos[0])
                check_y = robot_pos[1] + t * (frontier.y - robot_pos[1])

                # 转换到地图坐标
                map_x = int((check_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                map_y = int((check_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

                if (0 <= map_x < self.map_data.info.width and 0 <= map_y < self.map_data.info.height):
                    pixel_index = map_y * self.map_data.info.width + map_x
                    if pixel_index < len(self.map_data.data):
                        if self.map_data.data[pixel_index] == 100:  # 障碍物
                            return False

            return True
        except Exception:
            return True

    def _has_excessive_local_obstacles(self, frontier: OptimizedFrontierPoint) -> bool:
        """检查是否有过多局部障碍物"""
        try:
            # 检查前沿点周围的障碍物密度
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            obstacle_density = self._calculate_obstacle_density(map_x, map_y, 15)  # 1.5米半径

            # 如果障碍物密度过高，认为不适合探索
            return obstacle_density > 0.7
        except Exception:
            return False

    def _adjust_local_goal_frontier(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> OptimizedFrontierPoint:
        """动态调整局部目标前沿点"""
        try:
            # 🎯 创建调整后的前沿点副本
            adjusted_frontier = OptimizedFrontierPoint(
                x=frontier.x,
                y=frontier.y,
                size=getattr(frontier, 'size', 1),
                door_weight=getattr(frontier, 'door_weight', 1.0),
                quality_score=getattr(frontier, 'quality_score', 0.0),
                accessibility_score=getattr(frontier, 'accessibility_score', 0.0),
                room_priority=getattr(frontier, 'room_priority', 0.0),
                exploration_value=getattr(frontier, 'exploration_value', 0.0)
            )

            # 复制原始属性
            if hasattr(frontier, 'size'):
                adjusted_frontier.size = frontier.size
            if hasattr(frontier, 'room_priority'):
                adjusted_frontier.room_priority = frontier.room_priority

            # 🔍 局部路径优化
            optimized_position = self._optimize_local_path(frontier, robot_pos)
            if optimized_position:
                adjusted_frontier.x = optimized_position[0]
                adjusted_frontier.y = optimized_position[1]

            # 🛡️ 安全距离调整
            safe_position = self._ensure_safe_goal_position(adjusted_frontier, robot_pos)
            if safe_position:
                adjusted_frontier.x = safe_position[0]
                adjusted_frontier.y = safe_position[1]

            return adjusted_frontier
        except Exception as e:
            self.get_logger().error(f'❌ 局部目标调整异常: {e}')
            return frontier

    def _optimize_local_path(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """优化局部路径"""
        try:
            # 🎯 检查是否需要路径优化
            distance = math.sqrt((frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2)

            if distance < 2.0:  # 近距离目标需要精确调整
                # 🔍 寻找更好的局部位置
                best_position = None
                best_score = -float('inf')

                # 在前沿点周围搜索更好的位置
                search_radius = 0.5  # 50cm搜索半径
                search_steps = 8

                for i in range(search_steps):
                    angle = 2 * math.pi * i / search_steps
                    test_x = frontier.x + search_radius * math.cos(angle)
                    test_y = frontier.y + search_radius * math.sin(angle)

                    # 评估测试位置
                    score = self._evaluate_local_position(test_x, test_y, robot_pos)
                    if score > best_score:
                        best_score = score
                        best_position = (test_x, test_y)

                return best_position

            return None  # 远距离目标不需要调整
        except Exception:
            return None

    def _ensure_safe_goal_position(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """确保目标位置安全"""
        try:
            if not self.map_data:
                return None

            # 转换到地图坐标
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 检查目标位置是否安全
            if self._is_position_safe(map_x, map_y):
                return None  # 位置已经安全，不需要调整

            # 🔍 寻找附近的安全位置
            safe_radius = 10  # 搜索半径（像素）
            for radius in range(1, safe_radius + 1):
                for angle_step in range(0, 360, 30):  # 每30度检查一次
                    angle = math.radians(angle_step)
                    test_map_x = int(map_x + radius * math.cos(angle))
                    test_map_y = int(map_y + radius * math.sin(angle))

                    if self._is_position_safe(test_map_x, test_map_y):
                        # 转换回世界坐标
                        safe_x = test_map_x * self.map_data.info.resolution + self.map_data.info.origin.position.x
                        safe_y = test_map_y * self.map_data.info.resolution + self.map_data.info.origin.position.y
                        return (safe_x, safe_y)

            return None  # 找不到安全位置
        except Exception:
            return None

    def _evaluate_local_position(self, x: float, y: float, robot_pos: Tuple[float, float]) -> float:
        """评估局部位置质量"""
        try:
            score = 0.0

            # 🎯 距离因子（适中距离最好）
            distance = math.sqrt((x - robot_pos[0])**2 + (y - robot_pos[1])**2)
            if 1.0 <= distance <= 3.0:
                score += 0.3

            # 🛡️ 安全因子
            if self._is_world_position_safe(x, y):
                score += 0.4

            # 🔍 信息增益因子
            info_gain = self._calculate_position_info_gain(x, y)
            score += info_gain * 0.3

            return score
        except Exception:
            return 0.0

    def _is_position_safe(self, map_x: int, map_y: int) -> bool:
        """检查地图位置是否安全"""
        try:
            if not self.map_data:
                return False

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            # 检查边界
            if not (0 <= map_x < map_width and 0 <= map_y < map_height):
                return False

            # 检查周围区域是否安全
            safety_radius = 3  # 安全半径（像素）
            for dy in range(-safety_radius, safety_radius + 1):
                for dx in range(-safety_radius, safety_radius + 1):
                    check_x = map_x + dx
                    check_y = map_y + dy

                    if (0 <= check_x < map_width and 0 <= check_y < map_height):
                        pixel_index = check_y * map_width + check_x
                        if pixel_index < len(self.map_data.data):
                            if self.map_data.data[pixel_index] == 100:  # 障碍物
                                return False

            return True
        except Exception:
            return False

    def _is_world_position_safe(self, x: float, y: float) -> bool:
        """检查世界坐标位置是否安全"""
        try:
            if not self.map_data:
                return False

            map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            return self._is_position_safe(map_x, map_y)
        except Exception:
            return False

    def _calculate_position_info_gain(self, x: float, y: float) -> float:
        """计算位置的信息增益"""
        try:
            if not self.map_data:
                return 0.0

            # 转换到地图坐标
            map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 计算周围未知区域比例
            radius = int(1.0 / self.map_data.info.resolution)  # 1米半径
            return self._calculate_unknown_density_in_radius(map_x, map_y, radius)
        except Exception:
            return 0.0

    def _update_room_exploration_info(self, robot_pos: Tuple[float, float]) -> None:
        """更新房间探索信息"""
        try:
            if not self.map_data:
                return

            current_time = time.time()

            # 🏠 分析房间结构
            self._analyze_room_structure()

            # 📊 更新房间统计信息
            self._update_room_statistics()

            # 🔄 更新房间探索器状态
            self.room_explorer.update_room_info(
                self.room_centroids,
                self.room_stats,
                robot_pos,
                current_time
            )

            # 🎯 选择探索策略
            self.current_exploration_strategy = self.room_explorer.select_exploration_strategy(current_time)

        except Exception as e:
            self.get_logger().error(f'❌ 更新房间探索信息异常: {e}')

    def _analyze_room_structure(self) -> None:
        """分析房间结构"""
        try:
            if not self.map_data:
                return

            # 🗺️ 转换地图数据为numpy数组
            map_array = np.array(self.map_data.data).reshape(
                self.map_data.info.height,
                self.map_data.info.width
            )

            # 🔍 简化的房间检测
            # 将未知区域(-1)设为障碍物，自由空间(0)保持，障碍物(100)保持
            processed_map = np.where(map_array == -1, 100, map_array)
            processed_map = np.where(processed_map == 0, 255, 0).astype(np.uint8)

            # 🏠 使用连通组件分析检测房间
            num_labels, labels = cv2.connectedComponents(processed_map)

            # 📊 分析每个连通区域
            self.room_centroids = {}
            for label in range(1, num_labels):  # 跳过背景(0)
                # 计算区域质心
                y_coords, x_coords = np.where(labels == label)
                if len(x_coords) > 50:  # 最小区域大小过滤
                    # 转换到世界坐标
                    centroid_x = np.mean(x_coords) * self.map_data.info.resolution + self.map_data.info.origin.position.x
                    centroid_y = np.mean(y_coords) * self.map_data.info.resolution + self.map_data.info.origin.position.y
                    self.room_centroids[label] = (centroid_x, centroid_y)

        except Exception as e:
            self.get_logger().error(f'❌ 房间结构分析异常: {e}')

    def _update_room_statistics(self) -> None:
        """更新房间统计信息"""
        try:
            if not self.map_data or not self.room_centroids:
                return

            map_array = np.array(self.map_data.data).reshape(
                self.map_data.info.height,
                self.map_data.info.width
            )

            self.room_stats = {}

            for room_id, centroid in self.room_centroids.items():
                # 🔍 计算房间周围的探索统计
                map_x = int((centroid[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                map_y = int((centroid[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

                # 在房间中心周围5米半径内统计
                radius = int(5.0 / self.map_data.info.resolution)

                total_cells = 0
                unknown_cells = 0
                free_cells = 0
                obstacle_cells = 0

                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if dx*dx + dy*dy <= radius*radius:
                            check_x = map_x + dx
                            check_y = map_y + dy

                            if (0 <= check_x < self.map_data.info.width and
                                0 <= check_y < self.map_data.info.height):
                                total_cells += 1
                                cell_value = map_array[check_y, check_x]

                                if cell_value == -1:
                                    unknown_cells += 1
                                elif cell_value == 0:
                                    free_cells += 1
                                elif cell_value == 100:
                                    obstacle_cells += 1

                # 📊 计算房间统计
                self.room_stats[room_id] = {
                    'total_cells': total_cells,
                    'unknown_cells': unknown_cells,
                    'free_cells': free_cells,
                    'obstacle_cells': obstacle_cells,
                    'unknown_ratio': unknown_cells / max(total_cells, 1),
                    'free_ratio': free_cells / max(total_cells, 1),
                    'obstacle_ratio': obstacle_cells / max(total_cells, 1)
                }

        except Exception as e:
            self.get_logger().error(f'❌ 房间统计更新异常: {e}')

    def _apply_room_aware_frontier_selection(self, frontiers: List[OptimizedFrontierPoint], robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """应用房间感知的前沿点选择"""
        try:
            if not frontiers or self.current_exploration_strategy == ExplorationStrategy.FRONTIER_BASED:
                return None  # 使用默认TAD算法

            # 🏠 房间完成策略
            if self.current_exploration_strategy == ExplorationStrategy.ROOM_COMPLETION:
                return self._select_room_completion_frontier(frontiers, robot_pos)

            # 🚪 房间转移策略
            elif self.current_exploration_strategy == ExplorationStrategy.ROOM_TRANSITION:
                return self._select_room_transition_frontier(frontiers, robot_pos)

            # 🌍 全局覆盖策略
            elif self.current_exploration_strategy == ExplorationStrategy.GLOBAL_COVERAGE:
                return self._select_global_coverage_frontier(frontiers, robot_pos)

            return None
        except Exception as e:
            self.get_logger().error(f'❌ 房间感知前沿点选择异常: {e}')
            return None

    def _select_room_completion_frontier(self, frontiers: List[OptimizedFrontierPoint], robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """选择房间完成策略的前沿点"""
        try:
            current_room_id = self.room_explorer.current_room_id
            if not current_room_id or current_room_id not in self.room_centroids:
                return None

            room_center = self.room_centroids[current_room_id]

            # 🎯 优先选择当前房间内的前沿点
            room_frontiers = []
            for frontier in frontiers:
                # 检查前沿点是否在当前房间附近
                distance_to_room = math.sqrt(
                    (frontier.x - room_center[0])**2 +
                    (frontier.y - room_center[1])**2
                )
                if distance_to_room < 8.0:  # 8米范围内认为是同一房间
                    room_frontiers.append((frontier, distance_to_room))

            if room_frontiers:
                # 选择距离房间中心最近的前沿点
                room_frontiers.sort(key=lambda x: x[1])
                return room_frontiers[0][0]

            return None
        except Exception:
            return None

    def _select_room_transition_frontier(self, frontiers: List[OptimizedFrontierPoint], robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """选择房间转移策略的前沿点"""
        try:
            # 🚪 寻找最佳目标房间
            best_room_id = None
            best_room_score = -float('inf')

            for room_id in self.room_centroids:
                if room_id != self.room_explorer.current_room_id:
                    priority = self.room_explorer.get_room_exploration_priority(room_id)
                    if priority > best_room_score:
                        best_room_score = priority
                        best_room_id = room_id

            if not best_room_id:
                return None

            # 🎯 选择指向目标房间的前沿点
            target_room_center = self.room_centroids[best_room_id]

            best_frontier = None
            best_alignment_score = -float('inf')

            for frontier in frontiers:
                # 计算前沿点与目标房间的对齐度
                alignment_score = self._calculate_room_alignment_score(
                    frontier, robot_pos, target_room_center
                )

                if alignment_score > best_alignment_score:
                    best_alignment_score = alignment_score
                    best_frontier = frontier

            return best_frontier
        except Exception:
            return None

    def _select_global_coverage_frontier(self, frontiers: List[OptimizedFrontierPoint], robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """选择全局覆盖策略的前沿点"""
        try:
            # 🌍 选择能最大化全局覆盖的前沿点
            best_frontier = None
            best_coverage_score = -float('inf')

            for frontier in frontiers:
                # 计算全局覆盖贡献
                coverage_score = self._calculate_global_coverage_contribution(frontier)

                # 结合距离因子
                distance = math.sqrt((frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2)
                distance_factor = 1.0 / (1.0 + distance / 10.0)

                total_score = coverage_score * distance_factor

                if total_score > best_coverage_score:
                    best_coverage_score = total_score
                    best_frontier = frontier

            return best_frontier
        except Exception:
            return None

    def _calculate_room_alignment_score(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float], target_pos: Tuple[float, float]) -> float:
        """计算房间对齐评分"""
        try:
            # 🎯 计算机器人到目标房间的方向
            target_direction = math.atan2(
                target_pos[1] - robot_pos[1],
                target_pos[0] - robot_pos[0]
            )

            # 🎯 计算机器人到前沿点的方向
            frontier_direction = math.atan2(
                frontier.y - robot_pos[1],
                frontier.x - robot_pos[0]
            )

            # 计算方向差异
            direction_diff = abs(target_direction - frontier_direction)
            direction_diff = min(direction_diff, 2 * math.pi - direction_diff)

            # 方向对齐度评分（0-1）
            alignment_score = 1.0 - (direction_diff / math.pi)

            # 🔍 距离因子
            distance_to_target = math.sqrt(
                (frontier.x - target_pos[0])**2 +
                (frontier.y - target_pos[1])**2
            )
            distance_factor = 1.0 / (1.0 + distance_to_target / 15.0)

            return alignment_score * 0.7 + distance_factor * 0.3
        except Exception:
            return 0.0

    def _calculate_global_coverage_contribution(self, frontier: OptimizedFrontierPoint) -> float:
        """计算全局覆盖贡献"""
        try:
            if not self.map_data:
                return 0.0

            # 🌍 计算前沿点周围的未探索区域密度
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 大范围检查（8米半径）
            radius = int(8.0 / self.map_data.info.resolution)
            unknown_density = self._calculate_unknown_density_in_radius(map_x, map_y, radius)

            # 🎯 结合前沿点大小
            size_factor = getattr(frontier, 'size', 1.0) / 10.0

            return unknown_density * 0.8 + size_factor * 0.2
        except Exception:
            return 0.0

    def _update_adaptive_parameters(self) -> None:
        """更新自适应探索参数"""
        try:
            # 🔄 更新环境指标
            self._update_environment_metrics()

            # 🎯 基于环境指标调整参数
            self._adjust_exploration_interval()
            self._adjust_frontier_size_threshold()
            self._adjust_navigation_parameters()
            self._adjust_goal_tolerance()

            # 📊 记录参数调整历史
            self._record_parameter_adaptation()

        except Exception as e:
            self.get_logger().error(f'❌ 自适应参数更新异常: {e}')

    def _update_environment_metrics(self) -> None:
        """更新环境指标"""
        try:
            # 🎯 计算探索效率
            if hasattr(self, 'performance_stats'):
                total_time = self.performance_stats.get('total_exploration_time', 1.0)
                explored_area = self._calculate_explored_area()
                self.environment_metrics['exploration_efficiency'] = explored_area / total_time

                # 🎯 计算导航成功率
                total_nav = self.performance_stats.get('total_navigations', 1)
                successful_nav = self.performance_stats.get('successful_navigations', 0)
                self.environment_metrics['navigation_success_rate'] = successful_nav / total_nav

            # 🎯 计算平均前沿点数量
            if hasattr(self, 'frontiers') and self.frontiers:
                self.environment_metrics['average_frontier_count'] = len(self.frontiers)

            # 🎯 计算地图复杂度
            if self.map_data:
                self.environment_metrics['map_complexity'] = self._calculate_current_map_complexity()

        except Exception as e:
            self.get_logger().error(f'❌ 环境指标更新异常: {e}')

    def _adjust_exploration_interval(self) -> None:
        """调整探索间隔"""
        try:
            efficiency = self.environment_metrics['exploration_efficiency']
            frontier_count = self.environment_metrics['average_frontier_count']

            # 🔄 基于探索效率调整
            if efficiency > 0.5 and frontier_count > 10:
                # 高效率且前沿点多：可以加快探索
                new_interval = max(0.5, self.adaptive_params['exploration_interval'] * 0.9)
            elif efficiency < 0.2 or frontier_count < 3:
                # 低效率或前沿点少：放慢探索，给更多时间处理
                new_interval = min(3.0, self.adaptive_params['exploration_interval'] * 1.1)
            else:
                # 保持当前间隔
                new_interval = self.adaptive_params['exploration_interval']

            self.adaptive_params['exploration_interval'] = new_interval
            self.exploration_interval = new_interval

        except Exception:
            pass

    def _adjust_frontier_size_threshold(self) -> None:
        """调整前沿点大小阈值"""
        try:
            map_complexity = self.environment_metrics['map_complexity']
            exploration_progress = self._get_exploration_progress()

            # 🔄 基于地图复杂度和探索进度调整
            if exploration_progress > 0.8:
                # 后期探索：降低阈值，捕获更小的前沿点
                new_threshold = max(2, self.adaptive_params['min_frontier_size'] - 1)
            elif map_complexity > 0.6:
                # 复杂环境：提高阈值，避免噪声
                new_threshold = min(6, self.adaptive_params['min_frontier_size'] + 1)
            else:
                # 保持当前阈值
                new_threshold = self.adaptive_params['min_frontier_size']

            self.adaptive_params['min_frontier_size'] = new_threshold

            # 🔄 更新前沿检测器的阈值
            if hasattr(self, 'frontier_detector'):
                self.frontier_detector.min_frontier_size = new_threshold

        except Exception:
            pass

    def _adjust_navigation_parameters(self) -> None:
        """调整导航参数"""
        try:
            success_rate = self.environment_metrics['navigation_success_rate']
            map_complexity = self.environment_metrics['map_complexity']

            # 🔄 基于成功率调整导航超时
            if success_rate < 0.7:
                # 成功率低：增加超时时间
                new_timeout = min(60.0, self.adaptive_params['navigation_timeout'] * 1.2)
            elif success_rate > 0.9 and map_complexity < 0.4:
                # 成功率高且环境简单：可以缩短超时
                new_timeout = max(15.0, self.adaptive_params['navigation_timeout'] * 0.9)
            else:
                new_timeout = self.adaptive_params['navigation_timeout']

            self.adaptive_params['navigation_timeout'] = new_timeout
            self.navigation_timeout = new_timeout

            # 🔄 调整最大导航距离
            if success_rate < 0.6:
                # 成功率低：缩短导航距离
                new_max_distance = max(8.0, self.adaptive_params['max_navigation_distance'] * 0.9)
            elif success_rate > 0.9:
                # 成功率高：可以尝试更远距离
                new_max_distance = min(20.0, self.adaptive_params['max_navigation_distance'] * 1.1)
            else:
                new_max_distance = self.adaptive_params['max_navigation_distance']

            self.adaptive_params['max_navigation_distance'] = new_max_distance
            self.max_navigation_distance = new_max_distance

        except Exception:
            pass

    def _adjust_goal_tolerance(self) -> None:
        """调整目标容差"""
        try:
            success_rate = self.environment_metrics['navigation_success_rate']
            map_complexity = self.environment_metrics['map_complexity']

            # 🔄 基于成功率和复杂度调整容差
            if success_rate < 0.6 or map_complexity > 0.7:
                # 成功率低或环境复杂：放宽容差
                new_tolerance = min(2.0, self.adaptive_params['goal_tolerance'] * 1.1)
            elif success_rate > 0.9 and map_complexity < 0.3:
                # 成功率高且环境简单：可以收紧容差
                new_tolerance = max(0.8, self.adaptive_params['goal_tolerance'] * 0.95)
            else:
                new_tolerance = self.adaptive_params['goal_tolerance']

            self.adaptive_params['goal_tolerance'] = new_tolerance
            self.goal_tolerance = new_tolerance

        except Exception:
            pass

    def _calculate_explored_area(self) -> float:
        """计算已探索区域面积"""
        try:
            if not self.map_data:
                return 0.0

            # 计算已知区域（自由空间+障碍物）的面积
            total_cells = len(self.map_data.data)
            known_cells = sum(1 for cell in self.map_data.data if cell != -1)

            # 转换为实际面积（平方米）
            cell_area = self.map_data.info.resolution ** 2
            explored_area = known_cells * cell_area

            return explored_area
        except Exception:
            return 0.0

    def _calculate_current_map_complexity(self) -> float:
        """计算当前地图复杂度"""
        try:
            if not self.map_data:
                return 0.0

            # 转换地图数据
            map_array = np.array(self.map_data.data).reshape(
                self.map_data.info.height,
                self.map_data.info.width
            )

            # 计算障碍物边缘密度作为复杂度指标
            obstacle_map = np.where(map_array == 100, 255, 0).astype(np.uint8)
            edges = cv2.Canny(obstacle_map, 50, 150)
            edge_density = np.sum(edges > 0) / edges.size

            return min(edge_density * 5, 1.0)  # 归一化到0-1
        except Exception:
            return 0.5

    def _get_exploration_progress(self) -> float:
        """获取探索进度"""
        try:
            if not self.map_data:
                return 0.0

            total_cells = len(self.map_data.data)
            unknown_cells = sum(1 for cell in self.map_data.data if cell == -1)

            return 1.0 - (unknown_cells / max(total_cells, 1))
        except Exception:
            return 0.0

    def _record_parameter_adaptation(self) -> None:
        """记录参数适应历史"""
        try:
            adaptation_record = {
                'timestamp': time.time(),
                'parameters': self.adaptive_params.copy(),
                'metrics': self.environment_metrics.copy()
            }

            self.param_adaptation_history.append(adaptation_record)

            # 保持历史记录不超过100条
            if len(self.param_adaptation_history) > 100:
                self.param_adaptation_history.pop(0)

            # 📊 定期输出参数调整信息
            if len(self.param_adaptation_history) % 10 == 0:
                self.get_logger().info(
                    f'🔄 自适应参数更新: '
                    f'探索间隔={self.adaptive_params["exploration_interval"]:.1f}s, '
                    f'前沿阈值={self.adaptive_params["min_frontier_size"]}, '
                    f'导航超时={self.adaptive_params["navigation_timeout"]:.1f}s'
                )

        except Exception as e:
            self.get_logger().error(f'❌ 参数适应记录异常: {e}')

    def _parallel_frontier_evaluation(self, robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """并行前沿点评估"""
        try:
            if not self.frontiers or len(self.frontiers) < 3:
                return None  # 前沿点太少，不值得并行处理

            # 🚀 提交并行信息增益计算
            info_gain_task_id = self.parallel_manager.submit_parallel_info_gain_calculation(
                self.frontiers, robot_pos, self.map_data, self._calculate_frontier_info_gain_wrapper
            )

            # 🚀 提交并行前沿点评估
            evaluation_task_id = self.parallel_manager.submit_batch_frontier_evaluation(
                self.frontiers, robot_pos, self.map_data, self._evaluate_frontier_wrapper
            )

            # 🔄 获取并行计算结果
            info_gain_result = self.parallel_manager.get_computation_result(info_gain_task_id, timeout=10.0)
            evaluation_result = self.parallel_manager.get_computation_result(evaluation_task_id, timeout=10.0)

            # 🎯 合并结果并选择最佳前沿点
            if info_gain_result and evaluation_result:
                return self._merge_parallel_results(info_gain_result, evaluation_result, robot_pos)

            return None

        except Exception as e:
            self.get_logger().error(f'❌ 并行前沿点评估异常: {e}')
            return None

    def _calculate_frontier_info_gain_wrapper(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float], map_data, **kwargs) -> float:
        """前沿点信息增益计算包装器"""
        try:
            # 临时设置地图数据
            original_map = self.map_data
            self.map_data = map_data

            # 计算信息增益
            info_gain = self.calculate_information_gain(frontier)

            # 恢复原始地图数据
            self.map_data = original_map

            return info_gain
        except Exception:
            return 0.0

    def _evaluate_frontier_wrapper(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float], map_data, **kwargs) -> float:
        """前沿点评估包装器"""
        try:
            # 临时设置地图数据
            original_map = self.map_data
            self.map_data = map_data

            # 计算TAD评分
            trapezoid_score = self.calculate_trapezoid_parameter(frontier, robot_pos)
            adjacent_score = self.calculate_adjacent_parameter(frontier, robot_pos)
            distance_score = self.calculate_distance_parameter(frontier, robot_pos)
            reachability_score = self.calculate_reachability_score(frontier, robot_pos)

            # 综合评分
            total_score = (
                trapezoid_score * self.tad_weights['trapezoid'] +
                adjacent_score * self.tad_weights['adjacent'] +
                distance_score * self.tad_weights['distance'] +
                reachability_score * self.tad_weights['reachability']
            )

            # 恢复原始地图数据
            self.map_data = original_map

            return total_score
        except Exception:
            return 0.0

    def _merge_parallel_results(self, info_gain_result, evaluation_result, robot_pos: Tuple[float, float]) -> Optional[OptimizedFrontierPoint]:
        """合并并行计算结果"""
        try:
            # 获取信息增益数据
            info_gain_data = info_gain_result.result_data
            info_gains = info_gain_data.get('info_gains', [])

            # 获取评估数据
            evaluation_data = evaluation_result.result_data

            # 🎯 创建综合评分字典
            frontier_scores = {}

            # 处理信息增益结果
            for i, frontier in enumerate(self.frontiers):
                if i < len(info_gains):
                    info_gain = info_gains[i]
                else:
                    info_gain = 0.0

                frontier_key = (frontier.x, frontier.y)
                frontier_scores[frontier_key] = {
                    'frontier': frontier,
                    'info_gain': info_gain,
                    'evaluation_score': 0.0
                }

            # 处理评估结果
            for frontier, eval_score in evaluation_data:
                frontier_key = (frontier.x, frontier.y)
                if frontier_key in frontier_scores:
                    frontier_scores[frontier_key]['evaluation_score'] = eval_score

            # 🏆 计算最终评分并选择最佳前沿点
            best_frontier = None
            best_final_score = -float('inf')

            for frontier_key, scores in frontier_scores.items():
                frontier = scores['frontier']
                info_gain = scores['info_gain']
                eval_score = scores['evaluation_score']

                # 🎯 应用探索优先级和全局覆盖度奖励
                exploration_priority = self._calculate_exploration_priority(frontier, robot_pos)
                global_coverage_bonus = self._calculate_global_coverage_bonus(frontier)

                # 最终综合评分
                final_score = (
                    eval_score * 0.4 +                    # TAD评分
                    info_gain * self.tad_weights['info_gain'] * 0.4 +  # 信息增益
                    exploration_priority * 0.1 +          # 探索优先级
                    global_coverage_bonus * 0.1           # 全局覆盖度奖励
                )

                if final_score > best_final_score:
                    best_final_score = final_score
                    best_frontier = frontier

            if best_frontier:
                self.get_logger().info(f'🚀 并行评估选择最佳前沿点，评分: {best_final_score:.3f}')

            return best_frontier

        except Exception as e:
            self.get_logger().error(f'❌ 并行结果合并异常: {e}')
            return None

    def _check_multidimensional_completion(self) -> Dict:
        """多维度完成度检查"""
        try:
            current_time = time.time()

            # 🔄 限制检查频率（每5秒检查一次）
            if current_time - self.last_global_completion_check < 5.0:
                return {'is_complete': False, 'should_switch_area': False, 'reason': 'check_throttled'}

            self.last_global_completion_check = current_time

            # 🌟 更新完成度指标
            self._update_completion_metrics()

            # 🎯 多维度完成判断
            completion_scores = self._calculate_completion_scores()

            # 🏆 综合评估
            overall_completion = self._evaluate_overall_completion(completion_scores)

            # 📊 记录完成度历史
            self._record_completion_history(completion_scores, overall_completion)

            return overall_completion

        except Exception as e:
            self.get_logger().error(f'❌ 多维度完成度检查异常: {e}')
            return {'is_complete': False, 'should_switch_area': False, 'reason': 'error'}

    def _update_completion_metrics(self) -> None:
        """更新完成度指标"""
        try:
            if not self.map_data:
                return

            # 🌍 全局覆盖度
            total_cells = len(self.map_data.data)
            known_cells = sum(1 for cell in self.map_data.data if cell != -1)
            self.completion_metrics['global_coverage'] = known_cells / max(total_cells, 1)

            # 🎯 局部覆盖度（机器人周围区域）
            robot_pos = self._get_robot_position()
            if robot_pos:
                local_coverage = self._calculate_local_coverage(robot_pos)
                self.completion_metrics['local_coverage'] = local_coverage

            # 🔍 前沿点密度
            if hasattr(self, 'frontiers') and self.frontiers:
                map_area = total_cells * (self.map_data.info.resolution ** 2)
                frontier_density = len(self.frontiers) / max(map_area, 1)
                self.completion_metrics['frontier_density'] = frontier_density
            else:
                self.completion_metrics['frontier_density'] = 0.0

            # 📈 探索效率
            if hasattr(self, 'performance_stats'):
                total_time = self.performance_stats.get('total_exploration_time', 1.0)
                explored_area = self._calculate_explored_area()
                self.completion_metrics['exploration_efficiency'] = explored_area / total_time

            # 🏠 房间完成率
            if hasattr(self, 'room_explorer') and self.room_explorer:
                room_metrics = self.room_explorer.get_exploration_efficiency_metrics()
                self.completion_metrics['room_completion_rate'] = room_metrics.get('completion_rate', 0.0)

            # 🔍 细节完成评分
            self.completion_metrics['detail_completion_score'] = self._calculate_detail_completion_score()

        except Exception as e:
            self.get_logger().error(f'❌ 完成度指标更新异常: {e}')

    def _calculate_completion_scores(self) -> Dict:
        """计算各维度完成评分"""
        try:
            scores = {}

            # 🌍 全局覆盖度评分
            global_coverage = self.completion_metrics['global_coverage']
            if global_coverage >= self.global_completion_threshold:
                scores['global_coverage'] = 1.0
            elif global_coverage >= 0.7:
                scores['global_coverage'] = (global_coverage - 0.7) / (self.global_completion_threshold - 0.7) * 0.8
            else:
                scores['global_coverage'] = 0.0

            # 🔍 前沿点密度评分
            frontier_density = self.completion_metrics['frontier_density']
            if frontier_density <= self.frontier_density_threshold:
                scores['frontier_density'] = 1.0
            else:
                scores['frontier_density'] = max(0.0, 1.0 - (frontier_density - self.frontier_density_threshold) * 5)

            # 📈 探索效率评分
            exploration_efficiency = self.completion_metrics['exploration_efficiency']
            if exploration_efficiency >= self.exploration_efficiency_threshold:
                scores['exploration_efficiency'] = 1.0
            else:
                scores['exploration_efficiency'] = exploration_efficiency / self.exploration_efficiency_threshold

            # 🏠 房间完成评分
            room_completion = self.completion_metrics['room_completion_rate']
            scores['room_completion'] = room_completion

            # 🔍 细节完成评分
            detail_completion = self.completion_metrics['detail_completion_score']
            scores['detail_completion'] = detail_completion

            # 🎯 局部覆盖度评分
            local_coverage = self.completion_metrics['local_coverage']
            scores['local_coverage'] = local_coverage

            return scores

        except Exception as e:
            self.get_logger().error(f'❌ 完成评分计算异常: {e}')
            return {}

    def _evaluate_overall_completion(self, completion_scores: Dict) -> Dict:
        """评估整体完成度"""
        try:
            if not completion_scores:
                return {'is_complete': False, 'should_switch_area': False, 'reason': 'no_scores'}

            # 🎯 权重配置
            weights = {
                'global_coverage': 0.3,
                'frontier_density': 0.25,
                'room_completion': 0.2,
                'detail_completion': 0.15,
                'local_coverage': 0.1
            }

            # 🏆 计算加权总分
            weighted_score = 0.0
            total_weight = 0.0

            for metric, weight in weights.items():
                if metric in completion_scores:
                    weighted_score += completion_scores[metric] * weight
                    total_weight += weight

            overall_score = weighted_score / max(total_weight, 0.1)

            # 🎯 完成判断逻辑
            is_complete = False
            should_switch_area = False
            reason = "in_progress"
            confidence = overall_score

            # 🌟 高置信度完成
            if overall_score >= 0.9:
                is_complete = True
                reason = "high_confidence_completion"
                confidence = overall_score

            # 🎯 全局覆盖度完成
            elif completion_scores.get('global_coverage', 0) >= 0.95:
                is_complete = True
                reason = "global_coverage_complete"
                confidence = completion_scores['global_coverage']

            # 🔍 前沿点稀少且覆盖度高
            elif (completion_scores.get('frontier_density', 1) >= 0.8 and
                  completion_scores.get('global_coverage', 0) >= 0.8):
                is_complete = True
                reason = "low_frontier_high_coverage"
                confidence = min(completion_scores['frontier_density'], completion_scores['global_coverage'])

            # 🏠 房间探索完成
            elif (completion_scores.get('room_completion', 0) >= 0.9 and
                  completion_scores.get('global_coverage', 0) >= 0.75):
                should_switch_area = True
                reason = "room_exploration_complete"
                confidence = completion_scores['room_completion']

            # 🎯 局部区域完成
            elif (completion_scores.get('local_coverage', 0) >= 0.9 and
                  completion_scores.get('frontier_density', 1) >= 0.7):
                should_switch_area = True
                reason = "local_area_complete"
                confidence = completion_scores['local_coverage']

            return {
                'is_complete': is_complete,
                'should_switch_area': should_switch_area,
                'reason': reason,
                'confidence': confidence,
                'overall_score': overall_score,
                'detailed_scores': completion_scores
            }

        except Exception as e:
            self.get_logger().error(f'❌ 整体完成度评估异常: {e}')
            return {'is_complete': False, 'should_switch_area': False, 'reason': 'evaluation_error'}

    def _calculate_local_coverage(self, robot_pos: Tuple[float, float], radius: float = 5.0) -> float:
        """计算机器人周围的局部覆盖度"""
        try:
            if not self.map_data:
                return 0.0

            # 转换到地图坐标
            map_x = int((robot_pos[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((robot_pos[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 计算半径内的覆盖度
            radius_pixels = int(radius / self.map_data.info.resolution)

            total_cells = 0
            known_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-radius_pixels, radius_pixels + 1):
                for dx in range(-radius_pixels, radius_pixels + 1):
                    if dx*dx + dy*dy <= radius_pixels*radius_pixels:
                        check_x = map_x + dx
                        check_y = map_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_cells += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                if self.map_data.data[pixel_index] != -1:  # 已知区域
                                    known_cells += 1

            return known_cells / max(total_cells, 1)
        except Exception:
            return 0.0

    def _calculate_detail_completion_score(self) -> float:
        """计算细节完成评分"""
        try:
            if not self.map_data:
                return 0.0

            # 🔍 基于小前沿点的数量评估细节完成度
            small_frontiers = 0
            total_frontiers = 0

            if hasattr(self, 'frontiers') and self.frontiers:
                for frontier in self.frontiers:
                    total_frontiers += 1
                    if hasattr(frontier, 'size') and frontier.size <= 3:
                        small_frontiers += 1

            if total_frontiers == 0:
                return 1.0  # 没有前沿点，认为细节完成

            # 🎯 小前沿点比例越低，细节完成度越高
            small_frontier_ratio = small_frontiers / total_frontiers
            detail_score = 1.0 - small_frontier_ratio

            # 🔍 结合地图分辨率和复杂度
            map_complexity = self._calculate_current_map_complexity()
            if map_complexity > 0.5:  # 复杂环境需要更高的细节完成度
                detail_score *= 0.8

            return min(detail_score, 1.0)
        except Exception:
            return 0.5

    def _record_completion_history(self, completion_scores: Dict, overall_completion: Dict) -> None:
        """记录完成度历史"""
        try:
            history_record = {
                'timestamp': time.time(),
                'scores': completion_scores.copy(),
                'overall': overall_completion.copy(),
                'metrics': self.completion_metrics.copy()
            }

            self.completion_history.append(history_record)

            # 保持历史记录不超过50条
            if len(self.completion_history) > 50:
                self.completion_history.pop(0)

            # 📊 定期输出完成度信息
            if len(self.completion_history) % 5 == 0:
                overall_score = overall_completion.get('overall_score', 0.0)
                global_coverage = self.completion_metrics.get('global_coverage', 0.0)
                frontier_count = len(self.frontiers) if hasattr(self, 'frontiers') else 0

                self.get_logger().info(
                    f'📊 探索进度: 整体完成度={overall_score:.1%}, '
                    f'全局覆盖={global_coverage:.1%}, '
                    f'前沿点数量={frontier_count}'
                )

        except Exception as e:
            self.get_logger().error(f'❌ 完成度历史记录异常: {e}')

    def _get_completion_trend(self) -> Dict:
        """获取完成度趋势"""
        try:
            if len(self.completion_history) < 3:
                return {'trend': 'insufficient_data', 'rate': 0.0}

            # 计算最近几次的完成度变化趋势
            recent_scores = [record['overall']['overall_score'] for record in self.completion_history[-5:]]

            if len(recent_scores) >= 2:
                # 计算平均变化率
                changes = [recent_scores[i] - recent_scores[i-1] for i in range(1, len(recent_scores))]
                avg_change = sum(changes) / len(changes)

                if avg_change > 0.02:
                    trend = 'improving'
                elif avg_change < -0.01:
                    trend = 'declining'
                else:
                    trend = 'stable'

                return {'trend': trend, 'rate': avg_change}

            return {'trend': 'stable', 'rate': 0.0}
        except Exception:
            return {'trend': 'unknown', 'rate': 0.0}

    def calculate_trapezoid_parameter(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> float:
        """计算梯形参数"""
        try:
            # 基于前沿点大小的评分
            base_score = min(frontier.size / 50.0, 1.0) if hasattr(frontier, 'size') else 0.5
            return base_score
        except Exception:
            return 0.1

    def calculate_adjacent_parameter(self, frontier: OptimizedFrontierPoint) -> float:
        """计算邻接参数"""
        try:
            if self.map_data is None:
                return 0.5

            # 简化的邻接度计算
            return 0.7  # 默认较高的邻接度
        except Exception:
            return 0.5

    def calculate_distance_parameter(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> float:
        """计算距离参数"""
        try:
            distance = math.sqrt((frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2)
            max_distance = 20.0
            return max(0.0, 1.0 - (distance / max_distance))
        except Exception:
            return 0.0

    def calculate_information_gain(self, frontier: OptimizedFrontierPoint) -> float:
        """计算增强的信息增益"""
        try:
            if not self.map_data:
                return 0.5

            # 🎯 多尺度信息增益计算
            local_gain = self._calculate_local_information_gain(frontier)
            global_gain = self._calculate_global_information_gain(frontier)
            semantic_gain = self._calculate_semantic_information_gain(frontier)

            # 🔄 自适应权重分配
            total_gain = (
                local_gain * 0.4 +      # 局部信息增益
                global_gain * 0.4 +     # 全局信息增益
                semantic_gain * 0.2     # 语义信息增益
            )

            return min(total_gain, 1.0)
        except Exception as e:
            self.get_logger().error(f'❌ 信息增益计算异常: {e}')
            return 0.0

    def _calculate_local_information_gain(self, frontier: OptimizedFrontierPoint) -> float:
        """计算局部信息增益"""
        try:
            # 转换前沿点到地图坐标
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 局部检查半径（2米）
            radius_pixels = int(2.0 / self.map_data.info.resolution)

            total_cells = 0
            unknown_cells = 0

            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            for dy in range(-radius_pixels, radius_pixels + 1):
                for dx in range(-radius_pixels, radius_pixels + 1):
                    distance = math.sqrt(dx*dx + dy*dy)
                    if distance <= radius_pixels:
                        check_x = map_x + dx
                        check_y = map_y + dy

                        if (0 <= check_x < map_width and 0 <= check_y < map_height):
                            total_cells += 1
                            pixel_index = check_y * map_width + check_x
                            if pixel_index < len(self.map_data.data):
                                if self.map_data.data[pixel_index] == -1:  # 未知区域
                                    unknown_cells += 1

            return unknown_cells / max(total_cells, 1)
        except Exception:
            return 0.3

    def _calculate_global_information_gain(self, frontier: OptimizedFrontierPoint) -> float:
        """计算全局信息增益"""
        try:
            # 🌍 全局未探索区域密度
            total_unknown = sum(1 for cell in self.map_data.data if cell == -1)
            total_cells = len(self.map_data.data)
            global_unknown_ratio = total_unknown / max(total_cells, 1)

            # 🎯 前沿点在全局探索中的重要性
            if hasattr(self, 'frontiers') and self.frontiers:
                # 前沿点相对大小
                max_size = max(f.size for f in self.frontiers if hasattr(f, 'size'))
                relative_size = frontier.size / max(max_size, 1) if hasattr(frontier, 'size') else 0.5

                # 前沿点分布密度
                frontier_density = self._calculate_frontier_density(frontier)

                return global_unknown_ratio * 0.6 + relative_size * 0.3 + frontier_density * 0.1

            return global_unknown_ratio
        except Exception:
            return 0.3

    def _calculate_semantic_information_gain(self, frontier: OptimizedFrontierPoint) -> float:
        """计算语义信息增益（基于房间结构）"""
        try:
            # 🏠 检查是否靠近房间边界或门洞
            door_proximity = self._check_door_proximity(frontier)
            room_transition_value = self._check_room_transition_value(frontier)

            # 🔍 探索新区域的价值
            novelty_score = self._calculate_exploration_novelty(frontier)

            return door_proximity * 0.4 + room_transition_value * 0.3 + novelty_score * 0.3
        except Exception:
            return 0.2

    def _calculate_frontier_density(self, frontier: OptimizedFrontierPoint) -> float:
        """计算前沿点密度"""
        try:
            if not hasattr(self, 'frontiers') or not self.frontiers:
                return 0.5

            # 计算周围前沿点密度
            nearby_count = 0
            for other_frontier in self.frontiers:
                distance = math.sqrt(
                    (frontier.x - other_frontier.x)**2 +
                    (frontier.y - other_frontier.y)**2
                )
                if distance < 3.0:  # 3米范围内
                    nearby_count += 1

            # 归一化密度
            max_density = min(len(self.frontiers), 10)
            return min(nearby_count / max_density, 1.0)
        except Exception:
            return 0.5

    def _check_door_proximity(self, frontier: OptimizedFrontierPoint) -> float:
        """检查门洞邻近性"""
        try:
            # 简化的门洞检测：检查前沿点周围的狭窄通道
            map_x = int((frontier.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((frontier.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

            # 检查是否在狭窄通道附近
            narrow_passage_score = self._detect_narrow_passage(map_x, map_y)
            return min(narrow_passage_score, 1.0)
        except Exception:
            return 0.3

    def _check_room_transition_value(self, frontier: OptimizedFrontierPoint) -> float:
        """检查房间转移价值"""
        try:
            # 基于前沿点位置判断是否可能连接不同房间
            # 这里使用简化的启发式方法
            if hasattr(frontier, 'room_priority'):
                return min(frontier.room_priority, 1.0)
            return 0.5
        except Exception:
            return 0.5

    def _calculate_exploration_novelty(self, frontier: OptimizedFrontierPoint) -> float:
        """计算探索新颖性"""
        try:
            # 检查前沿点是否指向未探索的新区域
            if hasattr(self, 'recent_goals') and self.recent_goals:
                min_distance_to_recent = float('inf')
                for recent_goal in self.recent_goals:
                    distance = math.sqrt(
                        (frontier.x - recent_goal[0])**2 +
                        (frontier.y - recent_goal[1])**2
                    )
                    min_distance_to_recent = min(min_distance_to_recent, distance)

                # 距离最近访问点越远，新颖性越高
                novelty = min(min_distance_to_recent / 10.0, 1.0)
                return novelty

            return 0.8  # 如果没有历史记录，认为有较高新颖性
        except Exception:
            return 0.5

    def _detect_narrow_passage(self, map_x: int, map_y: int) -> float:
        """检测狭窄通道"""
        try:
            map_width = self.map_data.info.width
            map_height = self.map_data.info.height

            # 检查周围的自由空间分布
            free_directions = 0
            total_directions = 8

            directions = [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]

            for dx, dy in directions:
                check_x = map_x + dx
                check_y = map_y + dy

                if (0 <= check_x < map_width and 0 <= check_y < map_height):
                    pixel_index = check_y * map_width + check_x
                    if pixel_index < len(self.map_data.data):
                        if self.map_data.data[pixel_index] == 0:  # 自由空间
                            free_directions += 1

            # 自由方向较少表示可能是狭窄通道
            passage_score = 1.0 - (free_directions / total_directions)
            return passage_score
        except Exception:
            return 0.3

    def calculate_reachability_score(self, frontier: OptimizedFrontierPoint, robot_pos: Tuple[float, float]) -> float:
        """计算可达性得分"""
        try:
            # 距离评估
            distance = math.sqrt((frontier.x - robot_pos[0])**2 + (frontier.y - robot_pos[1])**2)

            # 距离过远的前沿点降低评分
            if distance > 15.0:
                return 0.1
            elif distance > 10.0:
                return 0.3
            elif distance > 5.0:
                return 0.7
            else:
                return 1.0
        except Exception:
            return 0.5

    def get_robot_position(self) -> Optional[Tuple[float, float]]:
        """获取机器人位置"""
        try:
            from tf2_ros import TransformException
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (transform.transform.translation.x, transform.transform.translation.y)
        except TransformException:
            return None

    def performance_monitor_callback(self):
        """性能监控回调"""
        try:
            current_time = time.time()
            exploration_duration = current_time - self.exploration_start_time

            # 计算探索效率
            total_navigations = self.performance_stats['successful_navigations'] + self.performance_stats['failed_navigations']
            success_rate = (self.performance_stats['successful_navigations'] / total_navigations * 100) if total_navigations > 0 else 0

            # 更新探索效率
            self.performance_stats['exploration_efficiency'] = success_rate

            # 记录性能统计
            self.get_logger().info(
                f'📊 性能统计 - 运行时间: {exploration_duration:.1f}s, '
                f'检测次数: {self.performance_stats["total_detections"]}, '
                f'成功导航: {self.performance_stats["successful_navigations"]}, '
                f'失败导航: {self.performance_stats["failed_navigations"]}, '
                f'成功率: {success_rate:.1f}%'
            )

            # 地图优化统计（如果启用）
            if self.map_optimizer:
                opt_stats = self.map_optimizer.get_optimization_stats()
                self.get_logger().info(
                    f'🗺️ 地图优化统计 - 优化次数: {opt_stats["total_optimizations"]}, '
                    f'平均处理时间: {opt_stats["processing_time"]:.3f}s'
                )

        except Exception as e:
            self.get_logger().error(f'❌ 性能监控异常: {e}')

    def update_performance_stats(self, event_type: str):
        """更新性能统计"""
        try:
            if event_type == 'detection':
                self.performance_stats['total_detections'] += 1
            elif event_type == 'navigation_success':
                self.performance_stats['successful_navigations'] += 1
            elif event_type == 'navigation_failure':
                self.performance_stats['failed_navigations'] += 1
        except Exception as e:
            self.get_logger().error(f'❌ 性能统计更新异常: {e}')

    def _record_navigation_success(self):
        """记录导航成功"""
        try:
            self.total_navigation_attempts += 1
            self.successful_navigations += 1
            self.consecutive_failures = 0
            self.last_successful_navigation = time.time()

            # 更新成功率
            self.navigation_success_rate = self.successful_navigations / self.total_navigation_attempts

            self.get_logger().debug(f'📊 导航成功率: {self.navigation_success_rate:.1%} ({self.successful_navigations}/{self.total_navigation_attempts})')
        except Exception as e:
            self.get_logger().error(f'❌ 记录导航成功异常: {e}')

    def _record_navigation_failure(self):
        """记录导航失败"""
        try:
            self.total_navigation_attempts += 1
            self.consecutive_failures += 1

            # 更新成功率
            self.navigation_success_rate = self.successful_navigations / self.total_navigation_attempts

            self.get_logger().warn(f'📊 导航失败 - 连续失败: {self.consecutive_failures}, 成功率: {self.navigation_success_rate:.1%}')

            # 如果连续失败过多，调整策略
            if self.consecutive_failures >= 3:
                self.get_logger().warn('⚠️ 连续导航失败过多，可能需要调整探索策略')
        except Exception as e:
            self.get_logger().error(f'❌ 记录导航失败异常: {e}')

    def _remove_visited_frontier(self):
        """移除已访问的前沿点"""
        try:
            if not self.current_goal:
                return

            # 计算移除半径（增加到3倍以确保完全移除）
            removal_radius = self.robot_radius * 3.0

            # 移除当前目标附近的前沿点
            original_count = len(self.frontiers)
            self.frontiers = [
                frontier for frontier in self.frontiers
                if math.sqrt((frontier.x - self.current_goal.x)**2 +
                           (frontier.y - self.current_goal.y)**2) > removal_radius
            ]

            removed_count = original_count - len(self.frontiers)
            if removed_count > 0:
                self.get_logger().info(f'🗑️ 移除了 {removed_count} 个已访问的前沿点 (半径: {removal_radius:.2f}m)')

                # 重新发布前沿点标记
                self.publish_frontier_markers()

        except Exception as e:
            self.get_logger().error(f'❌ 移除已访问前沿点异常: {e}')

    def _add_to_recent_goals(self):
        """添加当前目标到最近目标列表"""
        try:
            if not self.current_goal:
                return

            # 添加到最近目标列表
            goal_pos = (self.current_goal.x, self.current_goal.y)
            self.recent_goals.append(goal_pos)

            # 保持列表长度不超过5个
            if len(self.recent_goals) > 5:
                self.recent_goals.pop(0)

            self.get_logger().debug(f'📝 添加到最近目标: {goal_pos}, 总数: {len(self.recent_goals)}')

        except Exception as e:
            self.get_logger().error(f'❌ 添加最近目标异常: {e}')

    def _check_region_completion(self) -> bool:
        """检查当前区域是否已完成探索"""
        try:
            current_time = time.time()

            # 限制检查频率
            if current_time - self.last_region_check_time < 5.0:
                return False

            self.last_region_check_time = current_time

            # 获取机器人位置
            robot_pos = self.get_robot_position()
            if not robot_pos:
                return False

            # 简化的区域ID生成（基于位置网格）
            region_x = int(robot_pos[0] / 5.0)  # 5米网格
            region_y = int(robot_pos[1] / 5.0)
            region_id = f"{region_x}_{region_y}"

            # 更新当前区域
            if self.current_region_id != region_id:
                self.current_region_id = region_id
                if region_id not in self.exploration_regions:
                    self.exploration_regions[region_id] = {
                        'first_visit': current_time,
                        'last_visit': current_time,
                        'exploration_time': 0.0,
                        'frontier_count_history': []
                    }
                    self.get_logger().info(f'🏠 进入新区域: {region_id}')

            # 更新区域信息
            region_info = self.exploration_regions[region_id]
            region_info['last_visit'] = current_time
            region_info['exploration_time'] = current_time - region_info['first_visit']
            region_info['frontier_count_history'].append(len(self.frontiers))

            # 保持历史记录在合理范围内
            if len(region_info['frontier_count_history']) > 10:
                region_info['frontier_count_history'] = region_info['frontier_count_history'][-10:]

            # 检查区域完成度
            if len(region_info['frontier_count_history']) >= 3:
                recent_frontier_counts = region_info['frontier_count_history'][-3:]
                avg_frontiers = sum(recent_frontier_counts) / len(recent_frontier_counts)

                # 如果最近的前沿点数量很少且探索时间足够长
                if avg_frontiers <= 2 and region_info['exploration_time'] > 30.0:
                    self.get_logger().info(f'🏠 区域 {region_id} 可能已完成探索 (平均前沿点: {avg_frontiers:.1f})')
                    return True

            return False

        except Exception as e:
            self.get_logger().error(f'❌ 区域完成度检查异常: {e}')
            return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = RobustExploreNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f"❌ 节点异常: {e}")
        print(f"❌ 详细错误信息: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            try:
                # 先停止状态机
                if hasattr(node, 'state_manager') and node.state_manager:
                    node.state_manager.stop()
                    print("✅ 状态机已停止")

                # 再销毁节点
                node.destroy_node()
                print("✅ 节点已销毁")
            except Exception as e:
                print(f"⚠️ 节点关闭警告: {e}")
                import traceback
                print(f"⚠️ 清理详细错误: {traceback.format_exc()}")
        try:
            rclpy.shutdown()
            print("✅ RCL已关闭")
        except Exception as e:
            print(f"⚠️ RCL关闭警告: {e}")
            # 这个警告是正常的，因为可能已经关闭过了


if __name__ == '__main__':
    main()
