#!/bin/bash
# Aurora Project System Test Script
# This script performs comprehensive testing of the Aurora project on Linux

set -e  # Exit on any error

echo "🧪 Aurora Project System Test"
echo "============================="
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_DIR/build"
BIN_DIR="$PROJECT_DIR/bin"

# Test results
TESTS_PASSED=0
TESTS_FAILED=0
FAILED_TESTS=()

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo "🔍 Testing: $test_name"
    
    if eval "$test_command" >/dev/null 2>&1; then
        echo "✅ PASSED: $test_name"
        ((++TESTS_PASSED))
    else
        echo "❌ FAILED: $test_name"
        ((++TESTS_FAILED))
        FAILED_TESTS+=("$test_name")
    fi
}

# Function to run a test with output
run_test_with_output() {
    local test_name="$1"
    local test_command="$2"
    
    echo "🔍 Testing: $test_name"
    
    if eval "$test_command"; then
        echo "✅ PASSED: $test_name"
        ((++TESTS_PASSED))
    else
        echo "❌ FAILED: $test_name"
        ((++TESTS_FAILED))
        FAILED_TESTS+=("$test_name")
    fi
    echo ""
}

echo "📋 System Information:"
echo "   OS: $(uname -s)"
echo "   Architecture: $(uname -m)"
echo "   Kernel: $(uname -r)"
if [ -f /etc/os-release ]; then
    echo "   Distribution: $(grep PRETTY_NAME /etc/os-release | cut -d'"' -f2)"
fi
echo ""

# Test 1: Basic system dependencies
echo "🔧 Testing System Dependencies"
echo "------------------------------"

run_test "CMake availability" "command -v cmake"
run_test "Python3 availability" "command -v python3"
run_test "GCC availability" "command -v gcc"
run_test "Make availability" "command -v make"
run_test "Git availability" "command -v git"

echo ""

# Test 2: Python dependencies
echo "🐍 Testing Python Dependencies"
echo "------------------------------"

run_test "Python tkinter" "python3 -c 'import tkinter'"
run_test "Python pathlib" "python3 -c 'import pathlib'"
run_test "Python threading" "python3 -c 'import threading'"
run_test "Python subprocess" "python3 -c 'import subprocess'"
run_test "Python queue" "python3 -c 'import queue'"

echo ""

# Test 3: PCL availability
echo "☁️ Testing Point Cloud Library (PCL)"
echo "------------------------------------"

PCL_PKG=$(pkg-config --list-all 2>/dev/null | awk '{print $1}' | grep -E '^pcl_common' | head -n1)
run_test "PCL pkg-config" "[ -n \"$PCL_PKG\" ] && pkg-config --exists \"$PCL_PKG\""
if [ -n "$PCL_PKG" ] && pkg-config --exists "$PCL_PKG"; then
    PCL_VERSION=$(pkg-config --modversion "$PCL_PKG")
    echo "   PCL Version: $PCL_VERSION"
fi

echo ""

# Test 4: Aurora SDK
echo "🤖 Testing Aurora SDK"
echo "--------------------"

AURORA_SDK_DIR="$PROJECT_DIR/../aurora_sdk/aurora_remote_public"
run_test "Aurora SDK directory" "[ -d '$AURORA_SDK_DIR' ]"
run_test "Aurora SDK headers" "[ -f '$AURORA_SDK_DIR/include/aurora_pubsdk_inc.h' ]"

# Check platform-specific libraries
ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    PLATFORM_DIR="linux_aarch64"
else
    PLATFORM_DIR="linux_x86_64"
fi

AURORA_SDK_LIB_DIR="$AURORA_SDK_DIR/lib/$PLATFORM_DIR"
run_test "Aurora SDK libraries ($PLATFORM_DIR)" "[ -f '$AURORA_SDK_LIB_DIR/libslamtec_aurora_remote_sdk.so' ]"

echo ""

# Test 5: Project structure
echo "📁 Testing Project Structure"
echo "---------------------------"

run_test "CMakeLists.txt exists" "[ -f '$PROJECT_DIR/CMakeLists.txt' ]"
run_test "Source directory exists" "[ -d '$PROJECT_DIR/src' ]"
run_test "Scripts directory exists" "[ -d '$PROJECT_DIR/scripts' ]"
run_test "GUI script exists" "[ -f '$PROJECT_DIR/aurora_gui.py' ]"

# Check for key source files
run_test "colored_point_extractor.cpp" "[ -f '$PROJECT_DIR/src/colored_point_extractor.cpp' ]"
run_test "cloud_processor_tool.cpp" "[ -f '$PROJECT_DIR/src/cloud_processor_tool.cpp' ]"
run_test "advanced_building_detector.cpp" "[ -f '$PROJECT_DIR/src/advanced_building_detector.cpp' ]"

# Check for bash scripts
run_test "extract_point_cloud.sh" "[ -f '$PROJECT_DIR/scripts/extract_point_cloud.sh' ]"
run_test "process_point_cloud_walls.sh" "[ -f '$PROJECT_DIR/scripts/process_point_cloud_walls.sh' ]"
run_test "advanced_building_detection.sh" "[ -f '$PROJECT_DIR/scripts/advanced_building_detection.sh' ]"

echo ""

# Test 6: Build system
echo "🏗️ Testing Build System"
echo "----------------------"

if [ -d "$BUILD_DIR" ]; then
    run_test "Build directory exists" "true"
    run_test "CMake cache exists" "[ -f '$BUILD_DIR/CMakeCache.txt' ]"
else
    echo "⚠️  Build directory not found. Project needs to be built."
    echo "   Run: ./build_project.sh"
fi

if [ -d "$BIN_DIR" ]; then
    run_test "Binary directory exists" "true"
    
    # Check for built executables
    if [ -f "$BIN_DIR/colored_extractor" ]; then
        run_test "colored_extractor executable" "[ -x '$BIN_DIR/colored_extractor' ]"
    fi
    
    if [ -f "$BIN_DIR/cloud_processor_tool" ]; then
        run_test "cloud_processor_tool executable" "[ -x '$BIN_DIR/cloud_processor_tool' ]"
    fi
    
    if [ -f "$BIN_DIR/advanced_building_detector" ]; then
        run_test "advanced_building_detector executable" "[ -x '$BIN_DIR/advanced_building_detector' ]"
    fi
    
    if [ -f "$BIN_DIR/aurora_sdk_test" ]; then
        echo "🧪 Running Aurora SDK compatibility test..."
        run_test_with_output "Aurora SDK compatibility" "'$BIN_DIR/aurora_sdk_test'"
    fi
else
    echo "⚠️  Binary directory not found. Project needs to be built."
fi

echo ""

# Test 7: GUI functionality
echo "🖥️ Testing GUI Functionality"
echo "---------------------------"

# Test GUI import without starting it
run_test "GUI script syntax" "python3 -m py_compile '$PROJECT_DIR/aurora_gui.py'"

# Test if GUI can be imported
run_test "GUI imports" "PROJECT_DIR='$PROJECT_DIR' python3 -c 'import os,sys; project=os.environ[\"PROJECT_DIR\"]; sys.path.insert(0, project); p=os.path.join(project, \"aurora_gui.py\"); exec(open(p, encoding=\"utf-8\").read().split(\"if __name__\")[0])'"

echo ""

# Test 8: File permissions
echo "🔐 Testing File Permissions"
echo "--------------------------"

run_test "install_dependencies_ubuntu.sh executable" "[ -x '$PROJECT_DIR/install_dependencies_ubuntu.sh' ]"
run_test "build_project.sh executable" "[ -x '$PROJECT_DIR/build_project.sh' ]"
run_test "start_aurora_gui.sh executable" "[ -x '$PROJECT_DIR/start_aurora_gui.sh' ]"

# Check script permissions
for script in "$PROJECT_DIR/scripts"/*.sh; do
    if [ -f "$script" ]; then
        script_name=$(basename "$script")
        run_test "$script_name executable" "[ -x '$script' ]"
    fi
done

echo ""

# Summary
echo "📊 Test Summary"
echo "==============="
echo "✅ Tests Passed: $TESTS_PASSED"
echo "❌ Tests Failed: $TESTS_FAILED"
echo "📈 Success Rate: $(( TESTS_PASSED * 100 / (TESTS_PASSED + TESTS_FAILED) ))%"

if [ $TESTS_FAILED -gt 0 ]; then
    echo ""
    echo "❌ Failed Tests:"
    for test in "${FAILED_TESTS[@]}"; do
        echo "   - $test"
    done
    echo ""
    echo "🔧 Recommended Actions:"
    echo "1. Install missing dependencies: ./install_dependencies_ubuntu.sh"
    echo "2. Build the project: ./build_project.sh"
    echo "3. Set file permissions: chmod +x *.sh scripts/*.sh"
    echo "4. Check Aurora SDK installation"
fi

echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo "🎉 All tests passed! The system is ready to use."
    echo "   Start the GUI with: ./start_aurora_gui.sh"
    exit 0
else
    echo "⚠️  Some tests failed. Please address the issues above."
    exit 1
fi
