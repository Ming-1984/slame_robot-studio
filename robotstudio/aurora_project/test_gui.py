#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Aurora GUI Test Script
This script performs basic tests on the Aurora GUI components
"""

import sys
import os
from pathlib import Path

def test_imports():
    """Test if all required modules can be imported"""
    print("Testing imports...")
    
    try:
        import tkinter as tk
        print("✓ tkinter imported successfully")
    except ImportError as e:
        print(f"✗ tkinter import failed: {e}")
        return False
    
    try:
        from tkinter import ttk, filedialog, messagebox, scrolledtext
        print("✓ tkinter submodules imported successfully")
    except ImportError as e:
        print(f"✗ tkinter submodules import failed: {e}")
        return False
    
    try:
        import threading
        import queue
        import subprocess
        print("✓ Standard library modules imported successfully")
    except ImportError as e:
        print(f"✗ Standard library import failed: {e}")
        return False
    
    return True

def test_gui_creation():
    """Test if GUI can be created without errors"""
    print("\nTesting GUI creation...")
    
    try:
        import tkinter as tk
        from aurora_gui import AuroraGUI
        
        # Create root window
        root = tk.Tk()
        root.withdraw()  # Hide the window
        
        # Create GUI instance
        app = AuroraGUI(root)
        print("✓ GUI instance created successfully")
        
        # Test basic properties
        if hasattr(app, 'project_dir'):
            print(f"✓ Project directory: {app.project_dir}")

        if hasattr(app, 'scripts_dir'):
            print(f"✓ Scripts directory: {app.scripts_dir}")

        if hasattr(app, 'data_dir'):
            print(f"✓ Data directory: {app.data_dir}")

        if hasattr(app, 'device_ip'):
            print(f"✓ Aurora device IP variable: {app.device_ip.get()}")

        if hasattr(app, 'output_dir'):
            print(f"✓ Output directory variable: {app.output_dir.get()}")
        
        # Destroy the window
        root.destroy()
        
        return True
        
    except Exception as e:
        print(f"✗ GUI creation failed: {e}")
        return False

def test_project_structure():
    """Test if project structure is correct"""
    print("\nTesting project structure...")
    
    project_dir = Path(__file__).parent.absolute()
    
    # Check required files
    required_files = [
        "aurora_gui.py",
        "scripts/extract_point_cloud.sh",
        "scripts/process_point_cloud_walls.sh",
        "scripts/advanced_building_detection.sh",
        "CMakeLists.txt"
    ]
    
    for file_path in required_files:
        full_path = project_dir / file_path
        if full_path.exists():
            print(f"✓ Found: {file_path}")
        else:
            print(f"✗ Missing: {file_path}")
            return False
    
    # Check required directories
    required_dirs = [
        "scripts",
        "src",
        "include"
    ]
    
    for dir_path in required_dirs:
        full_path = project_dir / dir_path
        if full_path.exists() and full_path.is_dir():
            print(f"✓ Found directory: {dir_path}")
        else:
            print(f"✗ Missing directory: {dir_path}")
            return False
    
    return True

def test_script_modifications():
    """Test if scripts accept output directory arguments"""
    print("\nTesting script modifications...")
    
    project_dir = Path(__file__).parent.absolute()
    scripts_dir = project_dir / "scripts"
    
    scripts_to_check = [
        "extract_point_cloud.sh",
        "process_point_cloud_walls.sh",
        "advanced_building_detection.sh",
    ]
    
    for script_name in scripts_to_check:
        script_path = scripts_dir / script_name
        if not script_path.exists():
            print(f"✗ Script not found: {script_name}")
            return False
        
        try:
            content = script_path.read_text(encoding='utf-8')
            if "-OutputDir" in content and "-InputDir" in content:
                print(f"✓ {script_name} supports input and output directory parameters")
            elif "-OutputDir" in content:
                print(f"✓ {script_name} supports output directory parameter")
            else:
                print(f"✗ {script_name} missing directory parameter support")
                return False
        except Exception as e:
            print(f"✗ Error reading {script_name}: {e}")
            return False
    
    return True

def test_build_script():
    """Test if build script exists and is valid"""
    print("\nTesting build script...")
    
    project_dir = Path(__file__).parent.absolute()
    build_script = project_dir / "build_gui.py"
    
    if not build_script.exists():
        print("✗ Build script not found")
        return False
    
    try:
        # Try to compile the build script
        import py_compile
        py_compile.compile(str(build_script), doraise=True)
        print("✓ Build script syntax is valid")
        return True
    except Exception as e:
        print(f"✗ Build script has syntax errors: {e}")
        return False

def main():
    """Run all tests"""
    print("=" * 60)
    print("Aurora GUI Test Suite")
    print("=" * 60)
    
    tests = [
        ("Import Test", test_imports),
        ("GUI Creation Test", test_gui_creation),
        ("Project Structure Test", test_project_structure),
        ("Script Modifications Test", test_script_modifications),
        ("Build Script Test", test_build_script)
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * 40)
        
        try:
            if test_func():
                print(f"✓ {test_name} PASSED")
                passed += 1
            else:
                print(f"✗ {test_name} FAILED")
        except Exception as e:
            print(f"✗ {test_name} FAILED with exception: {e}")
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} tests passed")
    print("=" * 60)
    
    if passed == total:
        print("🎉 All tests passed! The GUI is ready to use.")
        return 0
    else:
        print("❌ Some tests failed. Please check the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
