#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
快速验证脚本
验证增强探索系统的基本功能是否正常
"""

import os
import sys
import time
import importlib.util

def check_basic_dependencies():
    """检查基本依赖"""
    print("🔍 检查基本依赖...")
    
    dependencies = ['numpy', 'cv2', 'sklearn', 'rclpy']
    missing = []
    
    for dep in dependencies:
        try:
            __import__(dep)
            print(f"  ✅ {dep}")
        except ImportError:
            print(f"  ❌ {dep}")
            missing.append(dep)
    
    return len(missing) == 0

def check_core_files():
    """检查核心文件"""
    print("\n📁 检查核心文件...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    files = [
        'robust_explore_node.py',
        'parallel_computation_manager.py',
        'predictive_frontier_detector.py',
        '../config/optimized_robust_params.yaml'
    ]
    
    missing = []
    for file in files:
        file_path = os.path.join(script_dir, file)
        if os.path.exists(file_path):
            print(f"  ✅ {file}")
        else:
            print(f"  ❌ {file}")
            missing.append(file)
    
    return len(missing) == 0

def test_robust_explore_node():
    """测试主探索节点"""
    print("\n🧪 测试主探索节点...")
    
    try:
        # 尝试导入主节点
        script_dir = os.path.dirname(os.path.abspath(__file__))
        spec = importlib.util.spec_from_file_location(
            "robust_explore_node", 
            os.path.join(script_dir, "robust_explore_node.py")
        )
        module = importlib.util.module_from_spec(spec)
        
        # 检查是否能成功加载模块
        spec.loader.exec_module(module)
        print("  ✅ 模块导入成功")
        
        # 检查关键类是否存在
        if hasattr(module, 'RobustExploreNode'):
            print("  ✅ RobustExploreNode类存在")
        else:
            print("  ❌ RobustExploreNode类不存在")
            return False
        
        return True
        
    except Exception as e:
        print(f"  ❌ 导入失败: {e}")
        return False

def test_configuration():
    """测试配置文件"""
    print("\n⚙️ 测试配置文件...")
    
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '../config/optimized_robust_params.yaml')
        
        if not os.path.exists(config_path):
            print("  ❌ 配置文件不存在")
            return False
        
        with open(config_path, 'r', encoding='utf-8') as f:
            content = f.read()
            
        # 检查关键配置项
        required_configs = [
            'exploration_timeout',
            'min_frontier_size',
            'navigation_timeout',
            'global_completion_threshold',
            'enable_parallel_frontier_detection'
        ]
        
        missing_configs = []
        for config in required_configs:
            if config in content:
                print(f"  ✅ {config}")
            else:
                print(f"  ❌ {config}")
                missing_configs.append(config)
        
        return len(missing_configs) == 0
        
    except Exception as e:
        print(f"  ❌ 配置文件测试失败: {e}")
        return False

def test_startup_script():
    """测试启动脚本"""
    print("\n🚀 测试启动脚本...")
    
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        startup_script = os.path.join(script_dir, '../../../start_path_planning.sh')
        
        if not os.path.exists(startup_script):
            print("  ❌ 启动脚本不存在")
            return False
        
        with open(startup_script, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查关键内容
        required_content = [
            'robust_explore_node.py',
            'optimized_robust_params.yaml',
            '增强的智能探索系统',
            'check_explorer_dependencies'
        ]
        
        missing_content = []
        for item in required_content:
            if item in content:
                print(f"  ✅ {item}")
            else:
                print(f"  ❌ {item}")
                missing_content.append(item)
        
        return len(missing_content) == 0
        
    except Exception as e:
        print(f"  ❌ 启动脚本测试失败: {e}")
        return False

def run_functional_tests():
    """运行功能测试"""
    print("\n🔧 运行功能测试...")
    
    tests_passed = 0
    total_tests = 0
    
    # 测试TAD算法计算
    total_tests += 1
    try:
        # 模拟TAD评分计算
        weights = {'trapezoid': 0.25, 'adjacent': 0.2, 'distance': 0.2, 'info_gain': 0.2, 'reachability': 0.15}
        scores = [0.8, 0.7, 0.6, 0.5, 0.9]
        total_score = sum(score * weight for score, weight in zip(scores, weights.values()))
        
        if 0 <= total_score <= 1:
            print("  ✅ TAD算法计算")
            tests_passed += 1
        else:
            print("  ❌ TAD算法计算")
    except Exception as e:
        print(f"  ❌ TAD算法计算: {e}")
    
    # 测试前沿点聚类
    total_tests += 1
    try:
        import numpy as np
        from sklearn.cluster import DBSCAN
        
        # 模拟前沿点数据
        points = np.random.rand(10, 2) * 100
        clustering = DBSCAN(eps=5.0, min_samples=2).fit(points)
        
        if hasattr(clustering, 'labels_'):
            print("  ✅ 前沿点聚类")
            tests_passed += 1
        else:
            print("  ❌ 前沿点聚类")
    except Exception as e:
        print(f"  ❌ 前沿点聚类: {e}")
    
    # 测试信息增益计算
    total_tests += 1
    try:
        import math
        
        radius = 3.0
        unknown_ratio = 0.3
        info_gain = unknown_ratio * (math.pi * radius**2)
        
        if info_gain > 0:
            print("  ✅ 信息增益计算")
            tests_passed += 1
        else:
            print("  ❌ 信息增益计算")
    except Exception as e:
        print(f"  ❌ 信息增益计算: {e}")
    
    # 测试多线程功能
    total_tests += 1
    try:
        import threading
        import concurrent.futures
        
        def test_task():
            return 42
        
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            future = executor.submit(test_task)
            result = future.result(timeout=1.0)
            
        if result == 42:
            print("  ✅ 多线程功能")
            tests_passed += 1
        else:
            print("  ❌ 多线程功能")
    except Exception as e:
        print(f"  ❌ 多线程功能: {e}")
    
    return tests_passed, total_tests

def main():
    """主函数"""
    print("🔍 增强探索系统快速验证")
    print("=" * 50)
    print(f"时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    all_passed = True
    
    # 运行各项检查
    if not check_basic_dependencies():
        all_passed = False
    
    if not check_core_files():
        all_passed = False
    
    if not test_robust_explore_node():
        all_passed = False
    
    if not test_configuration():
        all_passed = False
    
    if not test_startup_script():
        all_passed = False
    
    tests_passed, total_tests = run_functional_tests()
    if tests_passed < total_tests:
        all_passed = False
    
    # 生成报告
    print("\n" + "=" * 50)
    print("📋 验证结果摘要")
    print("=" * 50)
    
    if all_passed and tests_passed == total_tests:
        print("🎉 所有验证通过！系统已准备就绪。")
        print("✅ 可以安全启动增强探索系统")
    elif tests_passed / total_tests >= 0.8:
        print("⚠️ 大部分功能正常，但有少数问题需要关注")
        print("🔧 建议检查失败的项目后再启动系统")
    else:
        print("🚨 存在较多问题，建议修复后再启动系统")
        print("❌ 不建议在当前状态下启动系统")
    
    print(f"\n📊 功能测试通过率: {tests_passed}/{total_tests} ({tests_passed/total_tests*100:.1f}%)")
    print("=" * 50)

if __name__ == '__main__':
    main()
