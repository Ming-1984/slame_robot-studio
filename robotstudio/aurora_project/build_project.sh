#!/bin/bash
# Aurora Project Build Script for Linux
# This script builds the Aurora Point Cloud Processing project on Linux systems

set -e  # Exit on any error

echo "🏗️ Aurora Project Build Script for Linux"
echo "========================================"
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_DIR/build"
BIN_DIR="$PROJECT_DIR/bin"

# Parse command line arguments
CLEAN_BUILD=false
VERBOSE=false
BUILD_TYPE="Release"
RUN_TESTS=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --test)
            RUN_TESTS=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --clean     Clean build directory before building"
            echo "  --verbose   Enable verbose build output"
            echo "  --debug     Build in Debug mode (default: Release)"
            echo "  --test      Run tests after building"
            echo "  --help      Show this help message"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "📋 Build Configuration:"
echo "   Project Directory: $PROJECT_DIR"
echo "   Build Directory: $BUILD_DIR"
echo "   Build Type: $BUILD_TYPE"
echo "   Clean Build: $CLEAN_BUILD"
echo "   Verbose: $VERBOSE"
echo "   Run Tests: $RUN_TESTS"
echo ""

# Check if we're in the right directory
if [ ! -f "$PROJECT_DIR/CMakeLists.txt" ]; then
    echo "❌ Error: CMakeLists.txt not found in $PROJECT_DIR"
    echo "Please run this script from the project root directory."
    exit 1
fi

# Check dependencies
echo "🔍 Checking dependencies..."

# Check CMake
if ! command -v cmake &> /dev/null; then
    echo "❌ CMake not found. Please install CMake first."
    echo "Run: sudo apt install cmake"
    exit 1
fi

# Check PCL (try different version patterns)
if ! pkg-config --exists pcl_common-1.12 && ! pkg-config --exists pcl_common; then
    echo "❌ PCL (Point Cloud Library) not found."
    echo "Please install PCL first:"
    echo "Run: sudo apt install libpcl-dev"
    exit 1
fi

# Check Aurora SDK
AURORA_SDK_DIR="$PROJECT_DIR/../aurora_sdk/aurora_remote_public"
if [ ! -d "$AURORA_SDK_DIR" ]; then
    echo "❌ Aurora SDK not found at: $AURORA_SDK_DIR"
    echo "Please ensure the Aurora SDK is properly installed."
    exit 1
fi

# Detect architecture
ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    PLATFORM_DIR="linux_aarch64"
    echo "🤖 Detected ARM64 architecture (Jetson Nano compatible)"
else
    PLATFORM_DIR="linux_x86_64"
    echo "💻 Detected x86_64 architecture"
fi

AURORA_SDK_LIB_DIR="$AURORA_SDK_DIR/lib/$PLATFORM_DIR"
if [ ! -d "$AURORA_SDK_LIB_DIR" ]; then
    echo "❌ Aurora SDK libraries not found for platform: $PLATFORM_DIR"
    echo "Expected directory: $AURORA_SDK_LIB_DIR"
    exit 1
fi

echo "✅ All dependencies found"
echo ""

# Clean build directory if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo "🧹 Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    rm -rf "$BIN_DIR"
fi

# Create build directory
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    echo "📁 Created build directory: $BUILD_DIR"
fi

# Create bin directory
if [ ! -d "$BIN_DIR" ]; then
    mkdir -p "$BIN_DIR"
    echo "📁 Created bin directory: $BIN_DIR"
fi

# Configure with CMake
echo "⚙️ Configuring project with CMake..."
cd "$BUILD_DIR"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
if [ "$VERBOSE" = true ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_VERBOSE_MAKEFILE=ON"
fi

cmake $CMAKE_ARGS ..

if [ $? -ne 0 ]; then
    echo "❌ CMake configuration failed"
    exit 1
fi

echo "✅ CMake configuration completed"
echo ""

# Build the project
echo "🔨 Building project..."
MAKE_ARGS=""
if [ "$VERBOSE" = true ]; then
    MAKE_ARGS="VERBOSE=1"
fi

# Use all available CPU cores for building
NPROC=$(nproc)
echo "Using $NPROC parallel jobs for building"

make -j$NPROC $MAKE_ARGS

if [ $? -ne 0 ]; then
    echo "❌ Build failed"
    exit 1
fi

echo "✅ Build completed successfully"
echo ""

# List built executables
echo "📋 Built executables:"
cd "$PROJECT_DIR"
if [ -d "$BIN_DIR" ]; then
    find "$BIN_DIR" -type f -executable -printf "   %f\n" 2>/dev/null || \
    find "$BIN_DIR" -type f -perm -111 -exec basename {} \; | sed 's/^/   /'
else
    echo "   No executables found in $BIN_DIR"
fi

echo ""

# Run tests if requested
if [ "$RUN_TESTS" = true ]; then
    echo "🧪 Running tests..."
    
    # Test Aurora SDK compatibility
    if [ -f "$BIN_DIR/aurora_sdk_test" ]; then
        echo "Testing Aurora SDK compatibility..."
        "$BIN_DIR/aurora_sdk_test"
        if [ $? -eq 0 ]; then
            echo "✅ Aurora SDK test passed"
        else
            echo "❌ Aurora SDK test failed"
        fi
    else
        echo "⚠️  Aurora SDK test executable not found"
    fi
    
    echo ""
fi

# Set executable permissions for scripts
echo "🔧 Setting executable permissions for scripts..."
chmod +x "$PROJECT_DIR/scripts"/*.sh 2>/dev/null || true
chmod +x "$PROJECT_DIR"/*.sh 2>/dev/null || true

echo "🎉 Build process completed successfully!"
echo ""
echo "📋 Next steps:"
echo "1. Run 'python3 aurora_gui.py' to start the GUI application"
echo "2. Or run individual tools from the bin/ directory"
echo ""
echo "🔧 Available tools:"
echo "   - aurora_gui.py: Main GUI application"
echo "   - bin/colored_extractor: Point cloud extractor"
echo "   - bin/cloud_processor_tool: Point cloud processor"
echo "   - bin/advanced_building_detector: Building element detector"
if [ -f "$BIN_DIR/aurora_sdk_test" ]; then
    echo "   - bin/aurora_sdk_test: Aurora SDK compatibility test"
fi
echo ""
echo "💡 For troubleshooting, check the project documentation or run with --verbose flag"
