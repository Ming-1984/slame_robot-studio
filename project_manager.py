#!/usr/bin/env python3
"""
项目管理器
自动创建和管理项目文件夹，确保文件正确保存和组织
"""

import os
import shutil
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import logging

class ProjectManager:
    """项目管理器类"""
    
    def __init__(self, base_dir: str = "/home/jetson/ros2_ws/projects"):
        """
        初始化项目管理器
        
        Args:
            base_dir: 项目根目录
        """
        self.base_dir = Path(base_dir)
        self.max_projects = 20  # 最大项目数量（历史记录限制）
        self.current_project = None
        self.config_file = self.base_dir / "project_config.json"
        
        # 确保目录存在
        self.base_dir.mkdir(parents=True, exist_ok=True)
        
        # 子目录结构
        self.subdirs = [
            'maps',           # 地图文件
            'point_clouds',   # 点云文件
            'videos',         # 视频文件
            'logs',           # 日志文件
            'config',         # 配置文件
            'exports'         # 导出文件
        ]
        
        self.logger = logging.getLogger(__name__)
        
        # 加载或创建配置
        self.load_config()
    
    def create_new_project(self) -> str:
        """
        创建新项目

        Returns:
            项目ID
        """
        # 生成项目ID (格式: YYYY.MM.DD.HH.MM.SS)，确保唯一性
        now = datetime.now()
        project_id = now.strftime("%Y.%m.%d.%H.%M.%S")

        # 如果项目ID已存在，添加微秒数确保唯一性
        project_dir = self.base_dir / project_id
        counter = 0
        while project_dir.exists():
            counter += 1
            project_id = now.strftime(f"%Y.%m.%d.%H.%M.%S.{counter:03d}")
            project_dir = self.base_dir / project_id
        
        # 创建项目目录结构
        project_dir.mkdir(exist_ok=True)
        
        for subdir in self.subdirs:
            (project_dir / subdir).mkdir(exist_ok=True)
        
        # 创建项目元数据
        metadata = {
            'project_id': project_id,
            'created_at': now.isoformat(),
            'description': f'项目 {project_id}',
            'files_count': 0,
            'last_activity': now.isoformat()
        }
        
        # 保存项目元数据
        metadata_file = project_dir / 'project.json'
        with open(metadata_file, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, ensure_ascii=False, indent=2)
        
        # 更新当前项目
        self.current_project = project_id
        self.save_config()
        
        # 清理旧项目
        self.cleanup_old_projects()
        
        self.logger.info(f"创建新项目: {project_id}")
        return project_id
    
    def get_current_project(self) -> Optional[str]:
        """
        获取当前项目ID
        
        Returns:
            当前项目ID，如果没有则返回None
        """
        return self.current_project
    
    def get_project_dir(self, project_id: str = None) -> Path:
        """
        获取项目目录路径
        
        Args:
            project_id: 项目ID，如果为None则使用当前项目
            
        Returns:
            项目目录路径
        """
        if project_id is None:
            project_id = self.current_project
        
        if not project_id:
            raise ValueError("没有指定项目ID且当前项目为空")
        
        return self.base_dir / project_id
    
    def get_subdir_path(self, subdir: str, project_id: str = None) -> Path:
        """
        获取项目子目录路径
        
        Args:
            subdir: 子目录名称
            project_id: 项目ID，如果为None则使用当前项目
            
        Returns:
            子目录路径
        """
        project_dir = self.get_project_dir(project_id)
        subdir_path = project_dir / subdir
        
        # 确保子目录存在
        subdir_path.mkdir(exist_ok=True)
        
        return subdir_path
    
    def save_file(self, source_path: str, subdir: str, filename: str = None, project_id: str = None) -> str:
        """
        保存文件到项目目录
        
        Args:
            source_path: 源文件路径
            subdir: 目标子目录
            filename: 目标文件名，如果为None则使用原文件名
            project_id: 项目ID，如果为None则使用当前项目
            
        Returns:
            保存后的文件路径
        """
        if not self.current_project:
            self.create_new_project()
        
        source_path = Path(source_path)
        if not source_path.exists():
            raise FileNotFoundError(f"源文件不存在: {source_path}")
        
        # 获取目标目录
        target_dir = self.get_subdir_path(subdir, project_id)
        
        # 确定目标文件名
        if filename is None:
            filename = source_path.name
        
        target_path = target_dir / filename
        
        # 如果目标文件已存在，添加时间戳
        if target_path.exists():
            stem = target_path.stem
            suffix = target_path.suffix
            timestamp = datetime.now().strftime("_%H%M%S")
            target_path = target_dir / f"{stem}{timestamp}{suffix}"
        
        # 复制文件
        if source_path.is_dir():
            shutil.copytree(source_path, target_path)
        else:
            shutil.copy2(source_path, target_path)
        
        # 更新项目元数据
        self.update_project_metadata(project_id)
        
        self.logger.info(f"文件已保存: {source_path} -> {target_path}")
        return str(target_path)
    
    def list_projects(self) -> List[Dict]:
        """
        列出所有项目
        
        Returns:
            项目列表
        """
        projects = []
        
        for project_dir in sorted(self.base_dir.iterdir(), key=lambda x: x.name, reverse=True):
            if project_dir.is_dir() and not project_dir.name.startswith('.'):
                metadata_file = project_dir / 'project.json'
                
                if metadata_file.exists():
                    try:
                        with open(metadata_file, 'r', encoding='utf-8') as f:
                            metadata = json.load(f)
                        projects.append(metadata)
                    except Exception as e:
                        self.logger.error(f"读取项目元数据失败 {project_dir}: {e}")
                        # 创建默认元数据
                        metadata = {
                            'project_id': project_dir.name,
                            'created_at': datetime.fromtimestamp(project_dir.stat().st_ctime).isoformat(),
                            'description': f'项目 {project_dir.name}',
                            'files_count': self.count_project_files(project_dir.name),
                            'last_activity': datetime.fromtimestamp(project_dir.stat().st_mtime).isoformat()
                        }
                        projects.append(metadata)
        
        return projects
    
    def list_project_files(self, project_id: str = None, subdir: str = None) -> List[Dict]:
        """
        列出项目中的文件
        
        Args:
            project_id: 项目ID，如果为None则使用当前项目
            subdir: 子目录，如果为None则列出所有文件
            
        Returns:
            文件列表
        """
        project_dir = self.get_project_dir(project_id)
        files = []
        
        if subdir:
            search_dirs = [project_dir / subdir]
        else:
            search_dirs = [project_dir / d for d in self.subdirs if (project_dir / d).exists()]
        
        for search_dir in search_dirs:
            if not search_dir.exists():
                continue
                
            for file_path in search_dir.rglob('*'):
                if file_path.is_file() and not file_path.name.startswith('.'):
                    # 计算相对路径
                    rel_path = file_path.relative_to(project_dir)
                    
                    files.append({
                        'name': file_path.name,
                        'path': str(rel_path),
                        'full_path': str(file_path),
                        'size': file_path.stat().st_size,
                        'created_at': datetime.fromtimestamp(file_path.stat().st_ctime).isoformat(),
                        'modified_at': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat(),
                        'category': rel_path.parts[0] if rel_path.parts else 'unknown'
                    })
        
        # 按修改时间排序
        files.sort(key=lambda x: x['modified_at'], reverse=True)
        return files
    
    def count_project_files(self, project_id: str = None) -> int:
        """
        计算项目中的文件数量
        
        Args:
            project_id: 项目ID
            
        Returns:
            文件数量
        """
        try:
            return len(self.list_project_files(project_id))
        except:
            return 0
    
    def update_project_metadata(self, project_id: str = None):
        """
        更新项目元数据
        
        Args:
            project_id: 项目ID，如果为None则使用当前项目
        """
        if project_id is None:
            project_id = self.current_project
        
        if not project_id:
            return
        
        project_dir = self.get_project_dir(project_id)
        metadata_file = project_dir / 'project.json'
        
        # 读取现有元数据
        if metadata_file.exists():
            try:
                with open(metadata_file, 'r', encoding='utf-8') as f:
                    metadata = json.load(f)
            except:
                metadata = {}
        else:
            metadata = {}
        
        # 更新元数据
        metadata.update({
            'project_id': project_id,
            'files_count': self.count_project_files(project_id),
            'last_activity': datetime.now().isoformat()
        })
        
        # 保存元数据
        with open(metadata_file, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, ensure_ascii=False, indent=2)
    
    def cleanup_old_projects(self):
        """清理超过最大数量的旧项目"""
        projects = self.list_projects()
        
        if len(projects) > self.max_projects:
            # 按创建时间排序，删除最旧的项目
            projects.sort(key=lambda x: x['created_at'])
            projects_to_delete = projects[:-self.max_projects]
            
            for project in projects_to_delete:
                project_dir = self.base_dir / project['project_id']
                try:
                    shutil.rmtree(project_dir)
                    self.logger.info(f"删除旧项目: {project['project_id']}")
                except Exception as e:
                    self.logger.error(f"删除项目失败 {project['project_id']}: {e}")
    
    def switch_project(self, project_id: str) -> bool:
        """
        切换到指定项目
        
        Args:
            project_id: 项目ID
            
        Returns:
            是否成功切换
        """
        project_dir = self.base_dir / project_id
        
        if not project_dir.exists():
            self.logger.error(f"项目不存在: {project_id}")
            return False
        
        self.current_project = project_id
        self.save_config()
        self.logger.info(f"切换到项目: {project_id}")
        return True
    
    def load_config(self):
        """加载配置文件"""
        if self.config_file.exists():
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                
                self.current_project = config.get('current_project')
                self.max_projects = config.get('max_projects', 10)
                
                # 验证当前项目是否存在
                if self.current_project:
                    project_dir = self.base_dir / self.current_project
                    if not project_dir.exists():
                        self.current_project = None
                        
            except Exception as e:
                self.logger.error(f"加载配置失败: {e}")
                self.current_project = None
    
    def save_config(self):
        """保存配置文件"""
        config = {
            'current_project': self.current_project,
            'max_projects': self.max_projects,
            'last_updated': datetime.now().isoformat()
        }
        
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.logger.error(f"保存配置失败: {e}")
    
    def get_project_history(self, limit: int = 20) -> List[Dict]:
        """
        获取项目历史记录

        Args:
            limit: 返回的项目数量限制

        Returns:
            项目历史列表，按最后活动时间排序
        """
        projects = self.list_projects()

        # 按最后活动时间排序
        projects.sort(key=lambda x: x.get('last_activity', x.get('created_at', '')), reverse=True)

        # 限制数量
        return projects[:limit]

    def get_project_summary(self, project_id: str) -> Dict:
        """
        获取项目摘要信息

        Args:
            project_id: 项目ID

        Returns:
            项目摘要信息
        """
        try:
            project_dir = self.get_project_dir(project_id)
            metadata_file = project_dir / 'project.json'

            # 读取基本元数据
            if metadata_file.exists():
                with open(metadata_file, 'r', encoding='utf-8') as f:
                    metadata = json.load(f)
            else:
                metadata = {
                    'project_id': project_id,
                    'created_at': datetime.fromtimestamp(project_dir.stat().st_ctime).isoformat(),
                    'description': f'项目 {project_id}'
                }

            # 统计各类文件数量
            file_stats = {}
            total_size = 0

            for subdir in self.subdirs:
                subdir_path = project_dir / subdir
                if subdir_path.exists():
                    files = list(subdir_path.rglob('*'))
                    file_count = len([f for f in files if f.is_file()])
                    subdir_size = sum(f.stat().st_size for f in files if f.is_file())

                    file_stats[subdir] = {
                        'count': file_count,
                        'size': subdir_size,
                        'size_mb': round(subdir_size / (1024 * 1024), 2)
                    }
                    total_size += subdir_size
                else:
                    file_stats[subdir] = {'count': 0, 'size': 0, 'size_mb': 0}

            # 添加统计信息
            metadata['file_stats'] = file_stats
            metadata['total_size'] = total_size
            metadata['total_size_mb'] = round(total_size / (1024 * 1024), 2)
            metadata['total_files'] = sum(stats['count'] for stats in file_stats.values())

            return metadata

        except Exception as e:
            self.logger.error(f"获取项目摘要失败 {project_id}: {e}")
            return {
                'project_id': project_id,
                'error': str(e),
                'file_stats': {},
                'total_files': 0,
                'total_size': 0
            }

    def get_legacy_files_info(self) -> Dict:
        """获取旧文件系统中的文件信息（兼容性）"""
        legacy_dirs = {
            'maps': '/home/jetson/maps',
            'point_clouds': '/home/jetson/ros2_ws/data/point_clouds',
            'videos': '/home/jetson/ros2_ws/videos'
        }

        legacy_files = {}

        for category, path in legacy_dirs.items():
            path_obj = Path(path)
            if path_obj.exists():
                files = []
                for file_path in path_obj.rglob('*'):
                    if file_path.is_file():
                        files.append({
                            'name': file_path.name,
                            'full_path': str(file_path),
                            'size': file_path.stat().st_size,
                            'modified_at': datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
                        })
                legacy_files[category] = files

        return legacy_files
    
    def migrate_legacy_files(self) -> bool:
        """将旧文件迁移到项目系统"""
        try:
            # 创建迁移项目
            migration_project = "legacy_migration"
            migration_dir = self.base_dir / migration_project
            
            if not migration_dir.exists():
                migration_dir.mkdir()
                for subdir in self.subdirs:
                    (migration_dir / subdir).mkdir(exist_ok=True)
                
                # 迁移文件
                legacy_files = self.get_legacy_files_info()
                migrated_count = 0
                
                for category, files in legacy_files.items():
                    target_dir = migration_dir / category
                    
                    for file_info in files:
                        source_path = Path(file_info['full_path'])
                        target_path = target_dir / source_path.name
                        
                        try:
                            if source_path.is_dir():
                                shutil.copytree(source_path, target_path)
                            else:
                                shutil.copy2(source_path, target_path)
                            migrated_count += 1
                        except Exception as e:
                            self.logger.error(f"迁移文件失败 {source_path}: {e}")
                
                # 创建迁移项目元数据
                metadata = {
                    'project_id': migration_project,
                    'created_at': datetime.now().isoformat(),
                    'description': '旧文件系统迁移项目',
                    'files_count': migrated_count,
                    'last_activity': datetime.now().isoformat(),
                    'is_migration': True
                }
                
                with open(migration_dir / 'project.json', 'w', encoding='utf-8') as f:
                    json.dump(metadata, f, ensure_ascii=False, indent=2)
                
                self.logger.info(f"迁移完成，共迁移 {migrated_count} 个文件")
                return True
                
        except Exception as e:
            self.logger.error(f"文件迁移失败: {e}")
            
        return False


# 全局项目管理器实例
project_manager = ProjectManager()


def get_project_manager() -> ProjectManager:
    """获取项目管理器实例"""
    return project_manager


if __name__ == "__main__":
    # 测试代码
    pm = ProjectManager()
    
    # 创建新项目
    project_id = pm.create_new_project()
    print(f"创建项目: {project_id}")
    
    # 列出项目
    projects = pm.list_projects()
    print(f"项目列表: {[p['project_id'] for p in projects]}")
    
    # 获取当前项目路径
    current_dir = pm.get_project_dir()
    print(f"当前项目目录: {current_dir}")