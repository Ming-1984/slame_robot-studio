#!/usr/bin/env python3
"""
🛡️ 稳定状态管理器
基于2024年最新研究的容错状态机设计，提供强大的错误恢复能力
Author: Aurora Explorer Team
Date: 2025-01-20
"""

import time
import threading
from typing import Dict, List, Callable, Optional, Any
from enum import Enum
from dataclasses import dataclass
import traceback


class StateType(Enum):
    """状态类型"""
    NORMAL = "normal"
    RECOVERY = "recovery"
    CRITICAL = "critical"
    TERMINAL = "terminal"


class TransitionTrigger(Enum):
    """状态转换触发器"""
    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    ERROR = "error"
    MANUAL = "manual"


@dataclass
class StateTransition:
    """状态转换定义"""
    from_state: str
    to_state: str
    trigger: TransitionTrigger
    condition: Optional[Callable] = None
    action: Optional[Callable] = None


@dataclass
class StateDefinition:
    """状态定义"""
    name: str
    state_type: StateType
    entry_action: Optional[Callable] = None
    exit_action: Optional[Callable] = None
    update_action: Optional[Callable] = None
    timeout: Optional[float] = None
    max_retries: int = 3


class RobustStateManager:
    """稳定状态管理器"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.states: Dict[str, StateDefinition] = {}
        self.transitions: List[StateTransition] = []
        self.current_state: Optional[str] = None
        self.previous_state: Optional[str] = None
        
        # 状态历史和统计
        self.state_history: List[tuple] = []  # (state, timestamp, duration)
        self.error_count: Dict[str, int] = {}
        self.retry_count: Dict[str, int] = {}
        
        # 运行时变量
        self.state_start_time: Optional[float] = None
        self.is_running = False
        self.update_thread: Optional[threading.Thread] = None
        self.lock = threading.RLock()
        
        # 回调函数
        self.state_change_callbacks: List[Callable] = []
        self.error_callbacks: List[Callable] = []
        
        self._log_info("🛡️ 稳定状态管理器已初始化")
    
    def add_state(self, state_def: StateDefinition):
        """添加状态定义"""
        with self.lock:
            self.states[state_def.name] = state_def
            self.error_count[state_def.name] = 0
            self.retry_count[state_def.name] = 0
            self._log_info(f"📝 添加状态: {state_def.name} ({state_def.state_type.value})")
    
    def add_transition(self, transition: StateTransition):
        """添加状态转换"""
        with self.lock:
            self.transitions.append(transition)
            self._log_info(f"🔄 添加转换: {transition.from_state} -> {transition.to_state} ({transition.trigger.value})")
    
    def start(self, initial_state: str):
        """启动状态机"""
        with self.lock:
            if initial_state not in self.states:
                raise ValueError(f"初始状态 '{initial_state}' 不存在")
            
            self.current_state = initial_state
            self.state_start_time = time.time()
            self.is_running = True
            
            # 执行初始状态的进入动作
            self._execute_entry_action(initial_state)
            
            # 启动更新线程
            self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
            self.update_thread.start()
            
            self._log_info(f"🚀 状态机已启动，初始状态: {initial_state}")
    
    def stop(self):
        """停止状态机"""
        with self.lock:
            self.is_running = False
            if self.current_state:
                self._execute_exit_action(self.current_state)
            self._log_info("⏹️ 状态机已停止")
    
    def trigger_transition(self, trigger: TransitionTrigger, data: Any = None):
        """触发状态转换"""
        with self.lock:
            if not self.current_state:
                return False
            
            # 查找匹配的转换
            for transition in self.transitions:
                if (transition.from_state == self.current_state and 
                    transition.trigger == trigger):
                    
                    # 检查转换条件
                    if transition.condition and not transition.condition(data):
                        continue
                    
                    # 执行转换
                    return self._execute_transition(transition, data)
            
            self._log_warning(f"⚠️ 未找到匹配的转换: {self.current_state} -> {trigger.value}")
            return False
    
    def force_state(self, state_name: str):
        """强制切换到指定状态"""
        with self.lock:
            if state_name not in self.states:
                self._log_error(f"❌ 状态 '{state_name}' 不存在")
                return False
            
            old_state = self.current_state
            if old_state:
                self._execute_exit_action(old_state)
            
            self._change_state(state_name)
            self._log_info(f"🔧 强制切换状态: {old_state} -> {state_name}")
            return True
    
    def get_current_state(self) -> Optional[str]:
        """获取当前状态"""
        return self.current_state
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """获取状态统计信息"""
        with self.lock:
            total_time = sum(duration for _, _, duration in self.state_history if duration)
            
            stats = {
                'current_state': self.current_state,
                'total_runtime': total_time,
                'state_count': len(self.states),
                'transition_count': len(self.transitions),
                'error_count': dict(self.error_count),
                'retry_count': dict(self.retry_count),
                'state_history_length': len(self.state_history)
            }
            
            # 计算每个状态的时间占比
            if total_time > 0:
                state_time = {}
                for state_name, _, duration in self.state_history:
                    if duration:
                        state_time[state_name] = state_time.get(state_name, 0) + duration
                
                stats['state_time_percentage'] = {
                    state: (time_spent / total_time) * 100 
                    for state, time_spent in state_time.items()
                }
            
            return stats
    
    def add_state_change_callback(self, callback: Callable):
        """添加状态变化回调"""
        self.state_change_callbacks.append(callback)
    
    def add_error_callback(self, callback: Callable):
        """添加错误回调"""
        self.error_callbacks.append(callback)
    
    def _update_loop(self):
        """状态更新循环"""
        while self.is_running:
            try:
                with self.lock:
                    if self.current_state:
                        self._update_current_state()
                        self._check_timeout()
                
                time.sleep(0.1)  # 100ms更新间隔
                
            except Exception as e:
                self._log_error(f"❌ 状态更新循环异常: {e}")
                self._handle_error(e)
    
    def _update_current_state(self):
        """更新当前状态"""
        if not self.current_state:
            return
        
        state_def = self.states[self.current_state]
        if state_def.update_action:
            try:
                result = state_def.update_action()
                
                # 根据更新结果触发转换
                if result == "success":
                    self.trigger_transition(TransitionTrigger.SUCCESS)
                elif result == "failure":
                    self.trigger_transition(TransitionTrigger.FAILURE)
                elif result == "error":
                    self.trigger_transition(TransitionTrigger.ERROR)
                    
            except Exception as e:
                self._log_error(f"❌ 状态更新动作异常: {e}")
                self._handle_error(e)
    
    def _check_timeout(self):
        """检查状态超时"""
        if not self.current_state or not self.state_start_time:
            return
        
        state_def = self.states[self.current_state]
        if state_def.timeout:
            elapsed = time.time() - self.state_start_time
            if elapsed > state_def.timeout:
                self._log_warning(f"⏰ 状态超时: {self.current_state} ({elapsed:.1f}s)")
                self.trigger_transition(TransitionTrigger.TIMEOUT)
    
    def _execute_transition(self, transition: StateTransition, data: Any = None) -> bool:
        """执行状态转换"""
        try:
            # 执行转换动作
            if transition.action:
                transition.action(data)
            
            # 执行退出动作
            self._execute_exit_action(transition.from_state)
            
            # 切换状态
            self._change_state(transition.to_state)
            
            self._log_info(f"🔄 状态转换: {transition.from_state} -> {transition.to_state}")
            return True
            
        except Exception as e:
            self._log_error(f"❌ 状态转换异常: {e}")
            self._handle_error(e)
            return False
    
    def _change_state(self, new_state: str):
        """切换状态"""
        old_state = self.current_state
        old_start_time = self.state_start_time
        
        # 记录状态历史
        if old_state and old_start_time:
            duration = time.time() - old_start_time
            self.state_history.append((old_state, old_start_time, duration))
        
        # 切换到新状态
        self.previous_state = old_state
        self.current_state = new_state
        self.state_start_time = time.time()
        
        # 执行进入动作
        self._execute_entry_action(new_state)
        
        # 调用回调函数
        for callback in self.state_change_callbacks:
            try:
                callback(old_state, new_state)
            except Exception as e:
                self._log_error(f"❌ 状态变化回调异常: {e}")
    
    def _execute_entry_action(self, state_name: str):
        """执行状态进入动作"""
        state_def = self.states[state_name]
        if state_def.entry_action:
            try:
                state_def.entry_action()
            except Exception as e:
                self._log_error(f"❌ 状态进入动作异常: {e}")
                self._handle_error(e)
    
    def _execute_exit_action(self, state_name: str):
        """执行状态退出动作"""
        state_def = self.states[state_name]
        if state_def.exit_action:
            try:
                state_def.exit_action()
            except Exception as e:
                self._log_error(f"❌ 状态退出动作异常: {e}")
                self._handle_error(e)
    
    def _handle_error(self, error: Exception):
        """处理错误"""
        if self.current_state:
            self.error_count[self.current_state] += 1
            
            # 检查是否需要重试
            state_def = self.states[self.current_state]
            if self.retry_count[self.current_state] < state_def.max_retries:
                self.retry_count[self.current_state] += 1
                self._log_info(f"🔄 重试状态: {self.current_state} ({self.retry_count[self.current_state]}/{state_def.max_retries})")
                return
            
            # 超过重试次数，触发错误转换
            self.trigger_transition(TransitionTrigger.ERROR, error)
        
        # 调用错误回调
        for callback in self.error_callbacks:
            try:
                callback(error)
            except Exception as e:
                self._log_error(f"❌ 错误回调异常: {e}")
    
    def _log_info(self, message: str):
        """记录信息日志"""
        if self.logger:
            self.logger.info(message)
        else:
            print(f"[INFO] {message}")
    
    def _log_warning(self, message: str):
        """记录警告日志"""
        if self.logger:
            self.logger.warning(message)
        else:
            print(f"[WARNING] {message}")
    
    def _log_error(self, message: str):
        """记录错误日志"""
        if self.logger:
            self.logger.error(message)
        else:
            print(f"[ERROR] {message}")
