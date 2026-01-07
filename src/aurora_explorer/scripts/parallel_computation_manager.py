#!/usr/bin/env python3
"""
🔄 并行计算管理器
实现预测性目标计算和并行前沿点处理

核心功能：
1. 在当前目标执行过程中并行计算下一个目标
2. 支持异步前沿点检测和评估
3. 智能缓存管理和结果预测
4. 状态同步和一致性保证

算法特点：
- 预测性计算：在目标切换前提前准备下一个目标
- 并行处理：多线程处理前沿点检测和评估
- 智能缓存：基于地图变化的缓存失效策略
- 容错机制：处理并发计算中的异常情况

作者: Aurora探索系统
日期: 2025-07-21
"""

import threading
import time
import queue
import copy
from concurrent.futures import ThreadPoolExecutor, Future
from typing import List, Optional, Dict, Tuple, Callable, Any
from dataclasses import dataclass, field
from enum import Enum
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node

class ComputationState(Enum):
    """计算状态枚举"""
    IDLE = "idle"
    COMPUTING = "computing"
    READY = "ready"
    EXPIRED = "expired"
    ERROR = "error"

@dataclass
class ComputationTask:
    """计算任务"""
    task_id: str
    task_type: str  # 'frontier_detection', 'frontier_evaluation', 'target_selection'
    priority: int = 0
    created_time: float = field(default_factory=time.time)
    map_hash: str = ""
    robot_position: Tuple[float, float] = (0.0, 0.0)
    parameters: Dict[str, Any] = field(default_factory=dict)

@dataclass
class ComputationResult:
    """计算结果"""
    task_id: str
    state: ComputationState
    result_data: Any = None
    computation_time: float = 0.0
    created_time: float = field(default_factory=time.time)
    expiry_time: float = 0.0
    confidence: float = 1.0
    error_message: str = ""

class ParallelComputationManager:
    """并行计算管理器"""
    
    def __init__(self, node: Node, max_workers: int = 4):
        """
        初始化并行计算管理器
        
        Args:
            node: ROS2节点
            max_workers: 最大工作线程数
        """
        self.node = node
        self.max_workers = max_workers
        
        # 线程池
        self.executor = ThreadPoolExecutor(max_workers=max_workers)
        
        # 任务队列
        self.task_queue = queue.PriorityQueue()
        self.active_tasks: Dict[str, Future] = {}
        
        # 结果缓存
        self.result_cache: Dict[str, ComputationResult] = {}
        self.cache_lock = threading.RLock()
        
        # 状态管理
        self.is_running = True
        self.computation_stats = {
            'total_tasks': 0,
            'completed_tasks': 0,
            'cache_hits': 0,
            'cache_misses': 0,
            'average_computation_time': 0.0
        }
        
        # 配置参数
        self.cache_expiry_time = 30.0  # 缓存过期时间(秒)
        self.max_cache_size = 100      # 最大缓存大小
        self.prediction_horizon = 10.0  # 预测时间范围(秒)

        # 🚀 增强并行处理配置
        self.batch_processing_enabled = True  # 启用批处理
        self.max_batch_size = 20             # 最大批处理大小
        self.parallel_frontier_detection = True  # 并行前沿点检测
        self.parallel_info_gain_calculation = True  # 并行信息增益计算
        self.adaptive_worker_scaling = True  # 自适应工作线程缩放
        
        # 启动后台处理线程
        self.processing_thread = threading.Thread(target=self._background_processor, daemon=True)
        self.processing_thread.start()
        
        self.node.get_logger().info('🔄 并行计算管理器初始化完成')

    def submit_batch_frontier_evaluation(self,
                                       frontiers: List,
                                       robot_position: Tuple[float, float],
                                       map_data: OccupancyGrid,
                                       evaluation_func: Callable,
                                       **kwargs) -> str:
        """
        提交批量前沿点评估任务

        Args:
            frontiers: 前沿点列表
            robot_position: 机器人位置
            map_data: 地图数据
            evaluation_func: 评估函数
            **kwargs: 额外参数

        Returns:
            str: 任务ID
        """
        if not self.batch_processing_enabled or len(frontiers) <= 1:
            # 回退到单个处理
            return self.submit_predictive_computation(
                'frontier_evaluation', robot_position, map_data,
                evaluation_func, frontiers=frontiers, **kwargs
            )

        # 🚀 批量并行处理
        task_id = f"batch_evaluation_{int(time.time() * 1000)}"

        # 将前沿点分批
        batches = self._create_frontier_batches(frontiers)

        # 提交并行评估任务
        batch_futures = []
        for i, batch in enumerate(batches):
            batch_task_id = f"{task_id}_batch_{i}"
            future = self.executor.submit(
                self._evaluate_frontier_batch,
                batch, robot_position, map_data, evaluation_func, **kwargs
            )
            batch_futures.append((batch_task_id, future))

        # 提交结果聚合任务
        aggregation_future = self.executor.submit(
            self._aggregate_batch_results, batch_futures, task_id
        )

        self.active_tasks[task_id] = aggregation_future
        self.computation_stats['total_tasks'] += 1

        self.node.get_logger().debug(f'🚀 提交批量前沿点评估: {len(frontiers)}个前沿点, {len(batches)}个批次')
        return task_id

    def submit_parallel_info_gain_calculation(self,
                                             frontiers: List,
                                             robot_position: Tuple[float, float],
                                             map_data: OccupancyGrid,
                                             info_gain_func: Callable,
                                             **kwargs) -> str:
        """
        提交并行信息增益计算任务

        Args:
            frontiers: 前沿点列表
            robot_position: 机器人位置
            map_data: 地图数据
            info_gain_func: 信息增益计算函数
            **kwargs: 额外参数

        Returns:
            str: 任务ID
        """
        if not self.parallel_info_gain_calculation:
            return self.submit_predictive_computation(
                'info_gain_calculation', robot_position, map_data,
                info_gain_func, frontiers=frontiers, **kwargs
            )

        task_id = f"parallel_info_gain_{int(time.time() * 1000)}"

        # 🔄 并行计算每个前沿点的信息增益
        info_gain_futures = []
        for i, frontier in enumerate(frontiers):
            future = self.executor.submit(
                self._calculate_single_info_gain,
                frontier, robot_position, map_data, info_gain_func, **kwargs
            )
            info_gain_futures.append((i, future))

        # 聚合结果
        aggregation_future = self.executor.submit(
            self._aggregate_info_gain_results, info_gain_futures, frontiers, task_id
        )

        self.active_tasks[task_id] = aggregation_future
        self.computation_stats['total_tasks'] += 1

        self.node.get_logger().debug(f'🔄 提交并行信息增益计算: {len(frontiers)}个前沿点')
        return task_id

    def submit_predictive_computation(self,
                                    task_type: str,
                                    robot_position: Tuple[float, float],
                                    map_data: OccupancyGrid,
                                    computation_func: Callable,
                                    priority: int = 0,
                                    **kwargs) -> str:
        """
        提交预测性计算任务
        
        Args:
            task_type: 任务类型
            robot_position: 机器人位置
            map_data: 地图数据
            computation_func: 计算函数
            priority: 任务优先级(数值越大优先级越高)
            **kwargs: 额外参数
            
        Returns:
            str: 任务ID
        """
        # 生成任务ID
        task_id = f"{task_type}_{int(time.time() * 1000)}"
        
        # 计算地图哈希
        map_hash = self._calculate_map_hash(map_data)
        
        # 检查缓存
        cache_key = f"{task_type}_{map_hash}_{robot_position[0]:.1f}_{robot_position[1]:.1f}"
        cached_result = self._get_cached_result(cache_key)
        
        if cached_result and cached_result.state == ComputationState.READY:
            self.computation_stats['cache_hits'] += 1
            self.node.get_logger().debug(f'🎯 缓存命中: {task_type}')
            # 将缓存结果添加到活动任务中，以便get_computation_result可以找到
            from concurrent.futures import Future
            future = Future()
            future.set_result(cached_result)
            self.active_tasks[task_id] = future
            return task_id
            
        # 创建计算任务
        task = ComputationTask(
            task_id=task_id,
            task_type=task_type,
            priority=priority,
            map_hash=map_hash,
            robot_position=robot_position,
            parameters={
                'map_data': copy.deepcopy(map_data),
                'computation_func': computation_func,
                'cache_key': cache_key,
                **kwargs
            }
        )
        
        # 提交任务
        future = self.executor.submit(self._execute_computation_task, task)
        self.active_tasks[task_id] = future
        
        self.computation_stats['total_tasks'] += 1
        self.computation_stats['cache_misses'] += 1
        
        self.node.get_logger().debug(f'🔄 提交预测性计算任务: {task_type}, ID: {task_id}')
        return task_id
        
    def get_computation_result(self, task_id: str, timeout: float = 0.1) -> Optional[ComputationResult]:
        """
        获取计算结果
        
        Args:
            task_id: 任务ID
            timeout: 超时时间(秒)
            
        Returns:
            Optional[ComputationResult]: 计算结果，如果未完成返回None
        """
        # 检查活动任务
        if task_id in self.active_tasks:
            future = self.active_tasks[task_id]
            try:
                if future.done():
                    result = future.result()
                    del self.active_tasks[task_id]
                    return result
                elif timeout > 0:
                    result = future.result(timeout=timeout)
                    del self.active_tasks[task_id]
                    return result
            except Exception as e:
                import traceback
                error_details = traceback.format_exc()
                self.node.get_logger().error(f'❌ 获取计算结果异常: {task_id} - {e}')
                self.node.get_logger().error(f'❌ 详细错误信息: {error_details}')
                if task_id in self.active_tasks:
                    del self.active_tasks[task_id]
                # 返回空结果而不是None，避免调用者出错
                return ComputationResult(
                    task_id=task_id,
                    state=ComputationState.ERROR,
                    result_data=None,
                    error_message=str(e)
                )
                    
        # 检查缓存
        with self.cache_lock:
            for result in self.result_cache.values():
                if result.task_id == task_id:
                    return result
                    
        return None
        
    def get_best_available_result(self, task_type: str, 
                                robot_position: Tuple[float, float],
                                max_age: float = 10.0) -> Optional[ComputationResult]:
        """
        获取最佳可用结果
        
        Args:
            task_type: 任务类型
            robot_position: 机器人位置
            max_age: 最大结果年龄(秒)
            
        Returns:
            Optional[ComputationResult]: 最佳可用结果
        """
        current_time = time.time()
        best_result = None
        best_score = -float('inf')
        
        with self.cache_lock:
            for result in self.result_cache.values():
                if (result.task_id.startswith(task_type) and 
                    result.state == ComputationState.READY and
                    current_time - result.created_time <= max_age):
                    
                    # 计算结果评分（基于时间新鲜度和位置相关性）
                    age_factor = 1.0 - (current_time - result.created_time) / max_age
                    
                    # 计算位置相关性
                    if hasattr(result, 'robot_position'):
                        distance = np.sqrt(
                            (robot_position[0] - result.robot_position[0])**2 +
                            (robot_position[1] - result.robot_position[1])**2
                        )
                        position_factor = max(0.0, 1.0 - distance / 5.0)  # 5米内相关性较高
                    else:
                        position_factor = 0.5
                        
                    score = (age_factor * 0.6 + position_factor * 0.4) * result.confidence
                    
                    if score > best_score:
                        best_score = score
                        best_result = result
                        
        return best_result
        
    def _execute_computation_task(self, task: ComputationTask) -> ComputationResult:
        """执行计算任务"""
        start_time = time.time()
        result = ComputationResult(
            task_id=task.task_id,
            state=ComputationState.COMPUTING
        )
        
        try:
            # 执行计算
            computation_func = task.parameters['computation_func']
            computation_args = {k: v for k, v in task.parameters.items() 
                              if k not in ['computation_func', 'cache_key']}
            
            result_data = computation_func(**computation_args)
            
            # 更新结果
            computation_time = time.time() - start_time
            result.state = ComputationState.READY
            result.result_data = result_data
            result.computation_time = computation_time
            result.expiry_time = time.time() + self.cache_expiry_time
            
            # 缓存结果
            cache_key = task.parameters.get('cache_key', task.task_id)
            self._cache_result(cache_key, result)
            
            # 更新统计
            self.computation_stats['completed_tasks'] += 1
            self._update_average_computation_time(computation_time)
            
            self.node.get_logger().debug(
                f'✅ 计算任务完成: {task.task_type}, 耗时: {computation_time:.3f}s'
            )
            
        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            result.state = ComputationState.ERROR
            result.error_message = f"{str(e)} | {error_details}"
            self.node.get_logger().error(f'❌ 计算任务异常: {task.task_type}, 错误: {e}')
            self.node.get_logger().error(f'❌ 详细错误信息: {error_details}')
            
        return result
        
    def _background_processor(self):
        """后台处理线程"""
        while self.is_running:
            try:
                # 清理过期缓存
                self._cleanup_expired_cache()
                
                # 清理完成的任务
                self._cleanup_completed_tasks()
                
                time.sleep(1.0)  # 每秒检查一次
                
            except Exception as e:
                self.node.get_logger().error(f'❌ 后台处理异常: {e}')
                
    def _calculate_map_hash(self, map_data: OccupancyGrid) -> str:
        """计算地图哈希值"""
        try:
            # 使用地图数据的简化哈希
            data_array = np.array(map_data.data, dtype=np.int8)
            return str(hash(data_array.tobytes()))
        except Exception:
            return str(time.time())
            
    def _get_cached_result(self, cache_key: str) -> Optional[ComputationResult]:
        """获取缓存结果"""
        with self.cache_lock:
            result = self.result_cache.get(cache_key)
            if result and time.time() < result.expiry_time:
                return result
            elif result:
                # 过期结果，删除
                del self.result_cache[cache_key]
        return None
        
    def _cache_result(self, cache_key: str, result: ComputationResult):
        """缓存结果"""
        with self.cache_lock:
            # 检查缓存大小限制
            if len(self.result_cache) >= self.max_cache_size:
                # 删除最旧的结果
                oldest_key = min(self.result_cache.keys(), 
                               key=lambda k: self.result_cache[k].created_time)
                del self.result_cache[oldest_key]
                
            self.result_cache[cache_key] = result
            
    def _cleanup_expired_cache(self):
        """清理过期缓存"""
        current_time = time.time()
        with self.cache_lock:
            expired_keys = [
                key for key, result in self.result_cache.items()
                if current_time >= result.expiry_time
            ]
            for key in expired_keys:
                del self.result_cache[key]
                
    def _cleanup_completed_tasks(self):
        """清理已完成的任务"""
        completed_tasks = [
            task_id for task_id, future in self.active_tasks.items()
            if future.done()
        ]
        for task_id in completed_tasks:
            del self.active_tasks[task_id]
            
    def _update_average_computation_time(self, computation_time: float):
        """更新平均计算时间"""
        completed = self.computation_stats['completed_tasks']
        if completed > 1:
            current_avg = self.computation_stats['average_computation_time']
            new_avg = (current_avg * (completed - 1) + computation_time) / completed
            self.computation_stats['average_computation_time'] = new_avg
        else:
            self.computation_stats['average_computation_time'] = computation_time
            
    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self.cache_lock:
            cache_size = len(self.result_cache)
            
        return {
            **self.computation_stats,
            'active_tasks': len(self.active_tasks),
            'cache_size': cache_size,
            'cache_hit_rate': (
                self.computation_stats['cache_hits'] / 
                max(1, self.computation_stats['cache_hits'] + self.computation_stats['cache_misses'])
            )
        }
        
    def shutdown(self):
        """关闭管理器"""
        try:
            self.is_running = False

            # 等待短时间让正在执行的任务完成
            import time
            time.sleep(0.1)

            # 取消所有活动任务
            for task_id, future in list(self.active_tasks.items()):
                try:
                    if not future.done():
                        future.cancel()
                except Exception:
                    pass  # 忽略取消任务时的异常

            # 清空任务队列
            while not self.task_queue.empty():
                try:
                    self.task_queue.get_nowait()
                except:
                    break

            # 清空活动任务
            self.active_tasks.clear()

            # 关闭线程池（不等待，避免阻塞）
            try:
                self.executor.shutdown(wait=False)
                print("✅ 线程池已关闭")
            except Exception as e:
                print(f"⚠️ 线程池关闭异常: {e}")

            print('✅ 并行计算管理器关闭完成')

        except Exception as e:
            print(f'❌ 并行计算管理器关闭异常: {e}')
            import traceback
            print(f'❌ 详细错误: {traceback.format_exc()}')

    def _create_frontier_batches(self, frontiers: List) -> List[List]:
        """创建前沿点批次"""
        if len(frontiers) <= self.max_batch_size:
            return [frontiers]

        batches = []
        for i in range(0, len(frontiers), self.max_batch_size):
            batch = frontiers[i:i + self.max_batch_size]
            batches.append(batch)

        return batches

    def _evaluate_frontier_batch(self, batch: List, robot_position: Tuple[float, float],
                               map_data: OccupancyGrid, evaluation_func: Callable, **kwargs) -> List:
        """评估前沿点批次"""
        try:
            batch_results = []
            for frontier in batch:
                try:
                    result = evaluation_func(frontier, robot_position, map_data, **kwargs)
                    batch_results.append((frontier, result))
                except Exception as e:
                    self.node.get_logger().error(f'❌ 前沿点评估异常: {e}')
                    batch_results.append((frontier, 0.0))  # 默认评分

            return batch_results
        except Exception as e:
            self.node.get_logger().error(f'❌ 批次评估异常: {e}')
            return [(frontier, 0.0) for frontier in batch]

    def _aggregate_batch_results(self, batch_futures: List, task_id: str) -> ComputationResult:
        """聚合批次结果"""
        try:
            all_results = []
            total_computation_time = 0.0

            for batch_task_id, future in batch_futures:
                try:
                    start_time = time.time()
                    batch_result = future.result(timeout=45.0)  # 45秒超时
                    computation_time = time.time() - start_time
                    total_computation_time += computation_time

                    all_results.extend(batch_result)
                except Exception as e:
                    self.node.get_logger().error(f'❌ 批次结果获取异常: {e}')

            # 创建聚合结果
            result = ComputationResult(
                task_id=task_id,
                state=ComputationState.READY,
                result_data=all_results,
                computation_time=total_computation_time,
                confidence=0.9 if all_results else 0.0
            )

            return result

        except Exception as e:
            self.node.get_logger().error(f'❌ 结果聚合异常: {e}')
            return ComputationResult(
                task_id=task_id,
                state=ComputationState.ERROR,
                error_message=str(e)
            )

    def _calculate_single_info_gain(self, frontier, robot_position: Tuple[float, float],
                                  map_data: OccupancyGrid, info_gain_func: Callable, **kwargs) -> float:
        """计算单个前沿点的信息增益"""
        try:
            return info_gain_func(frontier, robot_position, map_data, **kwargs)
        except Exception as e:
            self.node.get_logger().error(f'❌ 信息增益计算异常: {e}')
            return 0.0

    def _aggregate_info_gain_results(self, info_gain_futures: List, frontiers: List, task_id: str) -> ComputationResult:
        """聚合信息增益结果"""
        try:
            info_gains = [0.0] * len(frontiers)
            total_computation_time = 0.0

            for i, future in info_gain_futures:
                try:
                    start_time = time.time()
                    info_gain = future.result(timeout=20.0)  # 20秒超时
                    computation_time = time.time() - start_time
                    total_computation_time += computation_time

                    info_gains[i] = info_gain
                except Exception as e:
                    self.node.get_logger().error(f'❌ 信息增益结果获取异常: {e}')
                    info_gains[i] = 0.0

            # 创建结果字典
            result_data = {
                'frontiers': frontiers,
                'info_gains': info_gains,
                'frontier_info_pairs': list(zip(frontiers, info_gains))
            }

            result = ComputationResult(
                task_id=task_id,
                state=ComputationState.READY,
                result_data=result_data,
                computation_time=total_computation_time,
                confidence=0.95
            )

            return result

        except Exception as e:
            self.node.get_logger().error(f'❌ 信息增益聚合异常: {e}')
            return ComputationResult(
                task_id=task_id,
                state=ComputationState.ERROR,
                error_message=str(e)
            )
