#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aurora GUI Build Script
This script builds the Aurora GUI application into a standalone executable
"""

import os
import sys
import platform
import subprocess
import shutil
from pathlib import Path

def check_requirements():
    """Check if required tools are available"""
    print("Checking requirements...")
    
    # Check Python
    python_version = sys.version_info
    if python_version.major < 3 or (python_version.major == 3 and python_version.minor < 7):
        print("Error: Python 3.7 or higher is required")
        return False
    print(f"✓ Python {python_version.major}.{python_version.minor}.{python_version.micro}")
    
    # Check PyInstaller
    try:
        import PyInstaller
        print(f"✓ PyInstaller {PyInstaller.__version__}")
    except ImportError:
        print("Installing PyInstaller...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])
            print("✓ PyInstaller installed successfully")
        except subprocess.CalledProcessError:
            print("Error: Failed to install PyInstaller")
            return False
    
    return True

def build_executable():
    """Build the executable using PyInstaller"""
    print("\nBuilding executable...")
    
    project_dir = Path(__file__).parent.absolute()
    gui_script = project_dir / "aurora_gui.py"
    dist_dir = project_dir / "dist"
    build_dir = project_dir / "build_temp"
    
    if not gui_script.exists():
        print(f"Error: GUI script not found: {gui_script}")
        return False
    
    # Clean previous builds
    if dist_dir.exists():
        shutil.rmtree(dist_dir)
    if build_dir.exists():
        shutil.rmtree(build_dir)
    
    # PyInstaller command
    add_data_sep = ";" if platform.system() == "Windows" else ":"
    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--onefile",                    # Create a single executable file
        "--windowed",                   # Hide console window
        "--name", "AuroraGUI",         # Executable name
        "--distpath", str(dist_dir),   # Output directory
        "--workpath", str(build_dir),  # Temporary build directory
        "--specpath", str(project_dir), # Spec file location
        "--add-data", f"{project_dir / 'scripts'}{add_data_sep}scripts",  # Include scripts folder
        "--icon", "NONE",              # No icon for now
        str(gui_script)
    ]
    
    try:
        print("Running PyInstaller...")
        print(f"Command: {' '.join(cmd)}")
        subprocess.check_call(cmd, cwd=str(project_dir))
        print("✓ Executable built successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error: PyInstaller failed with exit code {e.returncode}")
        return False

def create_distribution():
    """Create a complete distribution package"""
    print("\nCreating distribution package...")
    
    project_dir = Path(__file__).parent.absolute()
    dist_dir = project_dir / "dist"
    exe_name = "AuroraGUI.exe" if platform.system() == "Windows" else "AuroraGUI"
    exe_file = dist_dir / exe_name
    
    if not exe_file.exists():
        print(f"Error: Executable not found: {exe_file}")
        return False
    
    # Create distribution folder
    package_dir = dist_dir / "AuroraGUI_Package"
    if package_dir.exists():
        shutil.rmtree(package_dir)
    package_dir.mkdir(parents=True)
    
    # Copy executable
    shutil.copy2(exe_file, package_dir / exe_name)
    print(f"✓ Copied executable to {package_dir}")
    
    # Copy scripts folder
    scripts_src = project_dir / "scripts"
    scripts_dst = package_dir / "scripts"
    if scripts_src.exists():
        shutil.copytree(scripts_src, scripts_dst)
        print(f"✓ Copied scripts folder")
    
    # Copy necessary project files
    files_to_copy = [
        "CMakeLists.txt",
        "README.md"
    ]
    
    for file_name in files_to_copy:
        src_file = project_dir / file_name
        if src_file.exists():
            shutil.copy2(src_file, package_dir / file_name)
            print(f"✓ Copied {file_name}")
    
    # Copy source and include directories
    for dir_name in ["src", "include", "lib"]:
        src_dir = project_dir / dir_name
        if src_dir.exists():
            dst_dir = package_dir / dir_name
            shutil.copytree(src_dir, dst_dir)
            print(f"✓ Copied {dir_name} directory")
    
    # Create data directory
    data_dir = package_dir / "data"
    data_dir.mkdir(exist_ok=True)
    
    # Copy existing data files if any
    src_data_dir = project_dir / "data"
    if src_data_dir.exists():
        for file in src_data_dir.glob("*.stcm"):
            shutil.copy2(file, data_dir)
            print(f"✓ Copied data file: {file.name}")
    
    # Create README for the package
    launcher_name = "Start_Aurora.bat" if platform.system() == "Windows" else "Start_Aurora.sh"

    readme_content = f"""# Aurora Point Cloud Processing Tool

## Overview
This package contains the Aurora Point Cloud Processing Tool, a GUI application for processing point cloud data from radar systems.

## Contents
- {exe_name}: The main GUI application
- scripts/: Bash scripts for processing
- src/: Source code files
- include/: Header files
- lib/: Library files
- data/: Data directory (for input/output files)

## Requirements
- Ubuntu 22.04 (Jetson/Linux desktop recommended)
- CMake 3.15 or later
- Python 3.7+ (if running from source)

## Usage
1. Run {launcher_name} (or execute {exe_name} directly)
2. Select an output directory for processed files
3. Use the buttons to run processing steps:
   - Extract Point Cloud: Extract point cloud data from STCM files
   - Process Point Cloud: Enhance wall details in point cloud
4. Use "Run All Steps" for automated batch processing

## Output Files
The application will generate the following files in your selected output directory:
- colored_point_cloud.ply: Original extracted point cloud
- colored_point_cloud.xyz: Point cloud in XYZ format
- walls_enhanced_cloud.ply: Processed point cloud with enhanced walls

## Support
For issues or questions, please refer to the project documentation.
"""
    
    readme_file = package_dir / "README.txt"
    readme_file.write_text(readme_content, encoding='utf-8')
    print(f"✓ Created README.txt")
    
    # Create launcher script for easy launching
    launcher_file = package_dir / launcher_name
    if platform.system() == "Windows":
        launcher_content = f"""@echo off
echo Starting Aurora Point Cloud Processing Tool...
echo.
{exe_name}
pause
"""
        launcher_file.write_text(launcher_content, encoding='utf-8')
    else:
        launcher_content = f"""#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")" && pwd)"
exec "$SCRIPT_DIR/{exe_name}" "$@"
"""
        launcher_file.write_text(launcher_content, encoding='utf-8')
        os.chmod(launcher_file, 0o755)

    print(f"✓ Created {launcher_name}")
    
    print(f"\n✓ Distribution package created: {package_dir}")
    print(f"Package size: {get_folder_size(package_dir):.1f} MB")
    
    return True

def get_folder_size(folder_path):
    """Calculate folder size in MB"""
    total_size = 0
    for dirpath, dirnames, filenames in os.walk(folder_path):
        for filename in filenames:
            filepath = os.path.join(dirpath, filename)
            if os.path.exists(filepath):
                total_size += os.path.getsize(filepath)
    return total_size / (1024 * 1024)  # Convert to MB

def cleanup():
    """Clean up temporary build files"""
    print("\nCleaning up...")
    
    project_dir = Path(__file__).parent.absolute()
    
    # Remove temporary directories
    temp_dirs = [
        project_dir / "build_temp",
        project_dir / "__pycache__"
    ]
    
    for temp_dir in temp_dirs:
        if temp_dir.exists():
            shutil.rmtree(temp_dir)
            print(f"✓ Removed {temp_dir.name}")
    
    # Remove spec file
    spec_file = project_dir / "AuroraGUI.spec"
    if spec_file.exists():
        spec_file.unlink()
        print(f"✓ Removed {spec_file.name}")

def main():
    """Main build function"""
    print("=" * 60)
    print("Aurora GUI Build Script")
    print("=" * 60)
    
    try:
        # Check requirements
        if not check_requirements():
            print("\nBuild failed: Requirements not met")
            return 1
        
        # Build executable
        if not build_executable():
            print("\nBuild failed: Executable creation failed")
            return 1
        
        # Create distribution package
        if not create_distribution():
            print("\nBuild failed: Distribution package creation failed")
            return 1
        
        # Cleanup
        cleanup()
        
        print("\n" + "=" * 60)
        print("BUILD COMPLETED SUCCESSFULLY!")
        print("=" * 60)
        print(f"Distribution package: {Path(__file__).parent / 'dist' / 'AuroraGUI_Package'}")
        print("You can now distribute the AuroraGUI_Package folder.")
        
        return 0
        
    except KeyboardInterrupt:
        print("\nBuild cancelled by user")
        return 1
    except Exception as e:
        print(f"\nBuild failed with error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
