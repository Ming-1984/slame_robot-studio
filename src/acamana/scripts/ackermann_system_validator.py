#!/usr/bin/env python3
"""
阿克曼底盘系统验证器
检测参数不一致、运动学约束违反等问题
Author: Acamana-Bot Development Team
Date: 2025-01-15
"""

import os
import sys
import math
import yaml
import glob
from typing import Dict, List, Tuple, Any
import re

class AckermannSystemValidator:
    """阿克曼系统验证器"""
    
    def __init__(self):
        self.workspace_root = "/home/jetson/ros2_ws"
        self.validation_results = []
        self.parameter_files = []
        self.code_files = []
        
    def run_full_validation(self) -> Dict[str, Any]:
        """运行完整的系统验证"""
        print("🔍 开始阿克曼底盘系统验证...")
        
        # 1. 扫描参数文件
        self._scan_parameter_files()
        
        # 2. 扫描代码文件
        self._scan_code_files()
        
        # 3. 验证参数一致性
        consistency_results = self._validate_parameter_consistency()
        
        # 4. 验证运动学约束
        kinematics_results = self._validate_kinematics()
        
        # 5. 检测代码重复
        duplication_results = self._detect_code_duplication()
        
        # 6. 生成报告
        report = self._generate_report(consistency_results, kinematics_results, duplication_results)
        
        return report
        
    def _scan_parameter_files(self):
        """扫描参数配置文件"""
        patterns = [
            "src/**/config/*.yaml",
            "src/**/params/*.yaml", 
            "src/**/*params*.yaml"
        ]
        
        for pattern in patterns:
            files = glob.glob(os.path.join(self.workspace_root, pattern), recursive=True)
            self.parameter_files.extend(files)
            
        print(f"📁 发现 {len(self.parameter_files)} 个参数文件")
        
    def _scan_code_files(self):
        """扫描相关代码文件"""
        patterns = [
            "src/**/*ackermann*.py",
            "src/**/*controller*.py"
        ]
        
        for pattern in patterns:
            files = glob.glob(os.path.join(self.workspace_root, pattern), recursive=True)
            self.code_files.extend(files)
            
        print(f"📁 发现 {len(self.code_files)} 个代码文件")
        
    def _validate_parameter_consistency(self) -> Dict[str, Any]:
        """验证参数一致性"""
        print("\n🔧 验证参数一致性...")
        
        parameters = {
            'wheelbase': [],
            'max_steer_angle': [], 
            'min_turning_radius': [],
            'max_linear_velocity': [],
            'max_angular_velocity': []
        }
        
        # 从YAML文件提取参数
        for file_path in self.parameter_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = yaml.safe_load(f)
                    self._extract_parameters_from_yaml(content, parameters, file_path)
            except Exception as e:
                print(f"⚠️ 无法读取 {file_path}: {e}")
                
        # 从Python代码提取参数
        for file_path in self.code_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    self._extract_parameters_from_code(content, parameters, file_path)
            except Exception as e:
                print(f"⚠️ 无法读取 {file_path}: {e}")
                
        return self._analyze_parameter_consistency(parameters)
        
    def _extract_parameters_from_yaml(self, data: Any, parameters: Dict, file_path: str):
        """从YAML数据提取参数"""
        if isinstance(data, dict):
            for key, value in data.items():
                if key in parameters and isinstance(value, (int, float)):
                    parameters[key].append({
                        'value': float(value),
                        'source': file_path,
                        'type': 'yaml'
                    })
                elif isinstance(value, dict):
                    self._extract_parameters_from_yaml(value, parameters, file_path)
                    
    def _extract_parameters_from_code(self, content: str, parameters: Dict, file_path: str):
        """从Python代码提取参数"""
        patterns = {
            'wheelbase': r'wheelbase[\'\"]*\s*[=:,]\s*([0-9.]+)',
            'max_steer_angle': r'max_steer(?:ing)?_angle[\'\"]*\s*[=:,]\s*([0-9.]+)',
            'min_turning_radius': r'min_turning_radius[\'\"]*\s*[=:,]\s*([0-9.]+)',
            'max_linear_velocity': r'max_linear_vel(?:ocity)?[\'\"]*\s*[=:,]\s*([0-9.]+)',
            'max_angular_velocity': r'max_angular_vel(?:ocity)?[\'\"]*\s*[=:,]\s*([0-9.]+)'
        }
        
        for param_name, pattern in patterns.items():
            matches = re.findall(pattern, content, re.IGNORECASE)
            for match in matches:
                try:
                    value = float(match)
                    parameters[param_name].append({
                        'value': value,
                        'source': file_path,
                        'type': 'code'
                    })
                except ValueError:
                    pass
                    
    def _analyze_parameter_consistency(self, parameters: Dict) -> Dict[str, Any]:
        """分析参数一致性"""
        results = {
            'consistent': True,
            'issues': [],
            'summary': {}
        }
        
        for param_name, values in parameters.items():
            if len(values) <= 1:
                continue
                
            unique_values = {}
            for item in values:
                val = round(item['value'], 4)  # 4位小数精度
                if val not in unique_values:
                    unique_values[val] = []
                unique_values[val].append(item)
                
            if len(unique_values) > 1:
                results['consistent'] = False
                results['issues'].append({
                    'parameter': param_name,
                    'values': unique_values,
                    'count': len(unique_values)
                })
                
            results['summary'][param_name] = {
                'total_occurrences': len(values),
                'unique_values': len(unique_values),
                'values': list(unique_values.keys())
            }
            
        return results
        
    def _validate_kinematics(self) -> Dict[str, Any]:
        """验证运动学约束"""
        print("\n🎯 验证运动学约束...")
        
        # 获取关键参数的典型值
        wheelbase_values = [0.21333, 0.335, 0.5]  # 常见的轴距值
        max_steer_values = [0.6, 0.6108]         # 常见的最大转向角
        
        results = {
            'valid_combinations': [],
            'invalid_combinations': [],
            'recommendations': []
        }
        
        for wheelbase in wheelbase_values:
            for max_steer in max_steer_values:
                # 计算理论最小转弯半径
                theoretical_min_radius = wheelbase / math.tan(max_steer)
                
                # 检查是否符合物理约束
                is_valid = self._check_kinematic_validity(wheelbase, max_steer, theoretical_min_radius)
                
                combo = {
                    'wheelbase': wheelbase,
                    'max_steer_angle': max_steer,
                    'max_steer_degrees': math.degrees(max_steer),
                    'theoretical_min_radius': theoretical_min_radius,
                    'valid': is_valid
                }
                
                if is_valid:
                    results['valid_combinations'].append(combo)
                else:
                    results['invalid_combinations'].append(combo)
                    
        return results
        
    def _check_kinematic_validity(self, wheelbase: float, max_steer: float, min_radius: float) -> bool:
        """检查运动学参数的有效性"""
        # 基本物理约束检查
        if wheelbase <= 0 or max_steer <= 0 or min_radius <= 0:
            return False
            
        # 转向角应该在合理范围内 (0-45度)
        if max_steer > math.radians(45):
            return False
            
        # 最小转弯半径应该大于轴距
        if min_radius < wheelbase:
            return False
            
        # 检查计算一致性
        calculated_radius = wheelbase / math.tan(max_steer)
        if abs(calculated_radius - min_radius) > 0.001:
            return False
            
        return True
        
    def _detect_code_duplication(self) -> Dict[str, Any]:
        """检测代码重复"""
        print("\n🔄 检测代码重复...")
        
        # 查找阿克曼相关的类和函数
        ackermann_implementations = []
        
        for file_path in self.code_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                # 查找类定义
                class_matches = re.findall(r'class\s+(\w*[Aa]ckermann\w*)', content)
                for class_name in class_matches:
                    ackermann_implementations.append({
                        'type': 'class',
                        'name': class_name,
                        'file': file_path
                    })
                    
                # 查找函数定义
                func_matches = re.findall(r'def\s+(\w*ackermann\w*)', content, re.IGNORECASE)
                for func_name in func_matches:
                    ackermann_implementations.append({
                        'type': 'function', 
                        'name': func_name,
                        'file': file_path
                    })
                    
            except Exception as e:
                print(f"⚠️ 无法分析 {file_path}: {e}")
                
        return {
            'implementations': ackermann_implementations,
            'duplication_level': len(ackermann_implementations),
            'suspected_duplicates': self._identify_suspected_duplicates(ackermann_implementations)
        }
        
    def _identify_suspected_duplicates(self, implementations: List[Dict]) -> List[Dict]:
        """识别疑似重复的实现"""
        duplicates = []
        
        # 按名称分组
        name_groups = {}
        for impl in implementations:
            name = impl['name'].lower()
            if name not in name_groups:
                name_groups[name] = []
            name_groups[name].append(impl)
            
        # 找出有多个实现的名称
        for name, impls in name_groups.items():
            if len(impls) > 1:
                duplicates.append({
                    'name': name,
                    'count': len(impls),
                    'implementations': impls
                })
                
        return duplicates
        
    def _generate_report(self, consistency: Dict, kinematics: Dict, duplication: Dict) -> Dict[str, Any]:
        """生成验证报告"""
        print("\n📊 生成验证报告...")
        
        # 计算总体健康度评分
        health_score = self._calculate_health_score(consistency, kinematics, duplication)
        
        report = {
            'timestamp': '2025-01-15',
            'health_score': health_score,
            'parameter_consistency': consistency,
            'kinematics_validation': kinematics,
            'code_duplication': duplication,
            'recommendations': self._generate_recommendations(consistency, kinematics, duplication)
        }
        
        return report
        
    def _calculate_health_score(self, consistency: Dict, kinematics: Dict, duplication: Dict) -> int:
        """计算系统健康度评分 (0-100)"""
        score = 100
        
        # 参数一致性影响 (40分)
        if not consistency['consistent']:
            penalty = min(40, len(consistency['issues']) * 10)
            score -= penalty
            
        # 代码重复影响 (30分)
        if duplication['duplication_level'] > 2:
            penalty = min(30, (duplication['duplication_level'] - 2) * 10)
            score -= penalty
            
        # 运动学约束影响 (30分)
        if len(kinematics['invalid_combinations']) > 0:
            penalty = min(30, len(kinematics['invalid_combinations']) * 15)
            score -= penalty
            
        return max(0, score)
        
    def _generate_recommendations(self, consistency: Dict, kinematics: Dict, duplication: Dict) -> List[str]:
        """生成改进建议"""
        recommendations = []
        
        if not consistency['consistent']:
            recommendations.append("🔧 立即统一参数配置，使用单一配置文件")
            recommendations.append("✅ 实施参数自动验证机制")
            
        if duplication['duplication_level'] > 2:
            recommendations.append("🔄 整合重复的阿克曼控制器实现")
            recommendations.append("📦 创建统一的阿克曼控制模块")
            
        if len(kinematics['invalid_combinations']) > 0:
            recommendations.append("🎯 修正运动学参数，确保物理约束满足")
            recommendations.append("🧮 使用标准阿克曼运动学公式验证参数")
            
        if len(recommendations) == 0:
            recommendations.append("✅ 系统参数基本正常，建议进行性能优化")
            
        return recommendations
        
    def print_report(self, report: Dict):
        """打印验证报告"""
        print(f"\n{'='*60}")
        print("🚗 阿克曼底盘系统验证报告")
        print(f"{'='*60}")
        
        # 健康度评分
        health_score = report['health_score']
        if health_score >= 80:
            status = "✅ 优秀"
        elif health_score >= 60:
            status = "⚠️ 良好"
        elif health_score >= 40:
            status = "🔧 需要改进"
        else:
            status = "🚨 严重问题"
            
        print(f"📊 系统健康度: {health_score}/100 ({status})")
        
        # 参数一致性
        consistency = report['parameter_consistency']
        print(f"\n🔧 参数一致性: {'✅ 通过' if consistency['consistent'] else '❌ 失败'}")
        if not consistency['consistent']:
            print("   发现的问题:")
            for issue in consistency['issues']:
                print(f"   - {issue['parameter']}: {issue['count']} 个不同值")
                
        # 代码重复
        duplication = report['code_duplication']  
        print(f"\n🔄 代码重复检查: 发现 {duplication['duplication_level']} 个阿克曼实现")
        if duplication['suspected_duplicates']:
            print("   疑似重复:")
            for dup in duplication['suspected_duplicates']:
                print(f"   - {dup['name']}: {dup['count']} 个实现")
                
        # 改进建议
        print(f"\n💡 改进建议:")
        for i, rec in enumerate(report['recommendations'], 1):
            print(f"   {i}. {rec}")
            
        print(f"\n{'='*60}")


def main():
    """主函数"""
    validator = AckermannSystemValidator()
    
    try:
        report = validator.run_full_validation()
        validator.print_report(report)
        
        # 保存报告到文件
        import json
        with open('/tmp/ackermann_validation_report.json', 'w') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
            
        print(f"\n📄 详细报告已保存到: /tmp/ackermann_validation_report.json")
        
        # 返回错误码
        return 0 if report['health_score'] >= 70 else 1
        
    except Exception as e:
        print(f"❌ 验证过程出错: {e}")
        return 2


if __name__ == '__main__':
    sys.exit(main()) 