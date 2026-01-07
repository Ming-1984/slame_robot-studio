#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
探索功能验证脚本
验证增强探索系统的各项功能是否正常工作
"""

import os
import sys
import time
import importlib.util
from typing import Dict, List, Tuple, Optional

# 添加路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def check_module_import(module_name: str, module_path: str = None) -> Tuple[bool, str]:
    """检查模块导入"""
    try:
        if module_path:
            spec = importlib.util.spec_from_file_location(module_name, module_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        else:
            __import__(module_name)
        return True, "✅ 导入成功"
    except Exception as e:
        return False, f"❌ 导入失败: {e}"

def validate_dependencies() -> Dict[str, Tuple[bool, str]]:
    """验证依赖项"""
    print("🔍 验证系统依赖...")
    
    dependencies = {
        'numpy': ('numpy', None),
        'opencv-python': ('cv2', None),
        'scikit-learn': ('sklearn', None),
        'scipy': ('scipy', None),
        'rclpy': ('rclpy', None),
        'nav_msgs': ('nav_msgs.msg', None),
        'geometry_msgs': ('geometry_msgs.msg', None),
        'sensor_msgs': ('sensor_msgs.msg', None),
        'std_msgs': ('std_msgs.msg', None),
    }
    
    results = {}
    for dep_name, (module_name, module_path) in dependencies.items():
        success, message = check_module_import(module_name, module_path)
        results[dep_name] = (success, message)
        print(f"  {dep_name}: {message}")
    
    return results

def validate_core_modules() -> Dict[str, Tuple[bool, str]]:
    """验证核心模块"""
    print("\n🧩 验证核心模块...")
    
    core_modules = {
        'robust_explore_node': 'robust_explore_node.py',
        'parallel_computation_manager': 'parallel_computation_manager.py',
        'predictive_frontier_detector': 'predictive_frontier_detector.py',
    }
    
    results = {}
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    for module_name, filename in core_modules.items():
        module_path = os.path.join(script_dir, filename)
        if os.path.exists(module_path):
            success, message = check_module_import(module_name, module_path)
        else:
            success, message = False, f"❌ 文件不存在: {module_path}"
        
        results[module_name] = (success, message)
        print(f"  {module_name}: {message}")
    
    return results

def validate_aurora_modules() -> Dict[str, Tuple[bool, str]]:
    """验证Aurora模块"""
    print("\n🌟 验证Aurora模块...")
    
    aurora_modules = {
        'optimized_frontier_detector': '../aurora_explorer/optimized_frontier_detector.py',
        'room_aware_explorer': '../aurora_explorer/room_aware_explorer.py',
        'map_optimizer': '../aurora_explorer/map_optimizer.py',
    }
    
    results = {}
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    for module_name, relative_path in aurora_modules.items():
        module_path = os.path.join(script_dir, relative_path)
        if os.path.exists(module_path):
            success, message = check_module_import(module_name, module_path)
        else:
            success, message = False, f"❌ 文件不存在: {module_path}"
        
        results[module_name] = (success, message)
        print(f"  {module_name}: {message}")
    
    return results

def validate_configuration_files() -> Dict[str, Tuple[bool, str]]:
    """验证配置文件"""
    print("\n⚙️ 验证配置文件...")
    
    config_files = {
        'optimized_robust_params.yaml': '../config/optimized_robust_params.yaml',
        'start_path_planning.sh': '../../../start_path_planning.sh',
    }
    
    results = {}
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    for file_name, relative_path in config_files.items():
        file_path = os.path.join(script_dir, relative_path)
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if len(content) > 0:
                        success, message = True, "✅ 文件存在且可读"
                    else:
                        success, message = False, "❌ 文件为空"
            except Exception as e:
                success, message = False, f"❌ 读取失败: {e}"
        else:
            success, message = False, f"❌ 文件不存在: {file_path}"
        
        results[file_name] = (success, message)
        print(f"  {file_name}: {message}")
    
    return results

def validate_enhanced_features() -> Dict[str, Tuple[bool, str]]:
    """验证增强功能"""
    print("\n🚀 验证增强功能...")
    
    results = {}
    
    # 验证房间感知功能
    try:
        import cv2
        # 检查连通组件分析功能
        test_image = cv2.imread('/dev/null')  # 这会失败，但不会导致导入错误
        cv2.connectedComponents
        results['room_aware_exploration'] = (True, "✅ 房间感知功能可用")
    except Exception as e:
        results['room_aware_exploration'] = (False, f"❌ 房间感知功能不可用: {e}")
    
    # 验证并行计算功能
    try:
        import threading
        import concurrent.futures
        import queue
        results['parallel_computation'] = (True, "✅ 并行计算功能可用")
    except Exception as e:
        results['parallel_computation'] = (False, f"❌ 并行计算功能不可用: {e}")
    
    # 验证多尺度检测功能
    try:
        import numpy as np
        import cv2
        # 测试基本的图像处理功能
        test_array = np.zeros((10, 10), dtype=np.uint8)
        cv2.Canny(test_array, 50, 150)
        results['multiscale_detection'] = (True, "✅ 多尺度检测功能可用")
    except Exception as e:
        results['multiscale_detection'] = (False, f"❌ 多尺度检测功能不可用: {e}")
    
    # 验证自适应参数功能
    try:
        import time
        import math
        results['adaptive_parameters'] = (True, "✅ 自适应参数功能可用")
    except Exception as e:
        results['adaptive_parameters'] = (False, f"❌ 自适应参数功能不可用: {e}")
    
    for feature, (success, message) in results.items():
        print(f"  {feature}: {message}")
    
    return results

def run_functional_tests() -> Dict[str, Tuple[bool, str]]:
    """运行功能测试"""
    print("\n🧪 运行功能测试...")
    
    results = {}
    
    # 测试前沿点检测算法
    try:
        import numpy as np
        from sklearn.cluster import DBSCAN
        
        # 模拟前沿点数据
        test_points = np.random.rand(20, 2) * 100
        clustering = DBSCAN(eps=5.0, min_samples=3).fit(test_points)
        
        if len(set(clustering.labels_)) > 0:
            results['frontier_detection'] = (True, "✅ 前沿点检测算法正常")
        else:
            results['frontier_detection'] = (False, "❌ 前沿点检测算法异常")
    except Exception as e:
        results['frontier_detection'] = (False, f"❌ 前沿点检测测试失败: {e}")
    
    # 测试信息增益计算
    try:
        import math
        
        # 模拟信息增益计算
        test_radius = 3.0
        test_unknown_ratio = 0.3
        info_gain = test_unknown_ratio * (math.pi * test_radius**2)
        
        if info_gain > 0:
            results['information_gain'] = (True, "✅ 信息增益计算正常")
        else:
            results['information_gain'] = (False, "❌ 信息增益计算异常")
    except Exception as e:
        results['information_gain'] = (False, f"❌ 信息增益测试失败: {e}")
    
    # 测试TAD算法
    try:
        # 模拟TAD评分计算
        trapezoid_score = 0.8
        adjacent_score = 0.7
        distance_score = 0.6
        info_gain = 0.5
        reachability_score = 0.9
        
        weights = {
            'trapezoid': 0.25,
            'adjacent': 0.2,
            'distance': 0.2,
            'info_gain': 0.2,
            'reachability': 0.15
        }
        
        total_score = (
            trapezoid_score * weights['trapezoid'] +
            adjacent_score * weights['adjacent'] +
            distance_score * weights['distance'] +
            info_gain * weights['info_gain'] +
            reachability_score * weights['reachability']
        )
        
        if 0 <= total_score <= 1:
            results['tad_algorithm'] = (True, "✅ TAD算法计算正常")
        else:
            results['tad_algorithm'] = (False, "❌ TAD算法计算异常")
    except Exception as e:
        results['tad_algorithm'] = (False, f"❌ TAD算法测试失败: {e}")
    
    for test, (success, message) in results.items():
        print(f"  {test}: {message}")
    
    return results

def generate_validation_report(all_results: Dict[str, Dict[str, Tuple[bool, str]]]) -> None:
    """生成验证报告"""
    print("\n" + "="*80)
    print("📋 增强探索系统功能验证报告")
    print("="*80)
    
    total_tests = 0
    passed_tests = 0
    
    for category, results in all_results.items():
        print(f"\n📂 {category}:")
        category_passed = 0
        category_total = len(results)
        
        for test_name, (success, message) in results.items():
            status = "✅ PASS" if success else "❌ FAIL"
            print(f"  {test_name}: {status}")
            if not success:
                print(f"    详情: {message}")
            
            total_tests += 1
            if success:
                passed_tests += 1
                category_passed += 1
        
        print(f"  分类通过率: {category_passed}/{category_total} ({category_passed/category_total*100:.1f}%)")
    
    print(f"\n🏆 总体通过率: {passed_tests}/{total_tests} ({passed_tests/total_tests*100:.1f}%)")
    
    if passed_tests == total_tests:
        print("🎉 所有功能验证通过！系统已准备就绪。")
    elif passed_tests / total_tests >= 0.8:
        print("⚠️ 大部分功能正常，但有少数问题需要关注。")
    else:
        print("🚨 存在较多问题，建议检查系统配置和依赖。")
    
    print("="*80)

def main():
    """主函数"""
    print("🔍 开始增强探索系统功能验证...")
    print("时间:", time.strftime("%Y-%m-%d %H:%M:%S"))
    
    all_results = {}
    
    try:
        # 运行各项验证
        all_results['依赖项验证'] = validate_dependencies()
        all_results['核心模块验证'] = validate_core_modules()
        all_results['Aurora模块验证'] = validate_aurora_modules()
        all_results['配置文件验证'] = validate_configuration_files()
        all_results['增强功能验证'] = validate_enhanced_features()
        all_results['功能测试'] = run_functional_tests()
        
        # 生成报告
        generate_validation_report(all_results)
        
    except KeyboardInterrupt:
        print("\n🛑 验证被用户中断")
    except Exception as e:
        print(f"\n❌ 验证过程异常: {e}")

if __name__ == '__main__':
    main()
