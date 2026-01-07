#!/bin/bash
# Aurora GUI Startup Script for Linux
# This script starts the Aurora Point Cloud Processing GUI on Linux systems

set -e  # Exit on any error

echo "🚀 Starting Aurora Point Cloud Processing GUI"
echo "============================================="
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

# Check if we're in the right directory
if [ ! -f "$PROJECT_DIR/aurora_gui.py" ]; then
    echo "❌ Error: aurora_gui.py not found in $PROJECT_DIR"
    echo "Please run this script from the project root directory."
    exit 1
fi

# Check Python installation
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 not found. Please install Python3 first."
    echo "Run: sudo apt install python3"
    exit 1
fi

# Check tkinter availability
if ! python3 -c "import tkinter" 2>/dev/null; then
    echo "❌ Python tkinter not available. Please install python3-tk."
    echo "Run: sudo apt install python3-tk"
    exit 1
fi

# Check if project is built
BIN_DIR="$PROJECT_DIR/bin"
if [ ! -d "$BIN_DIR" ] || [ -z "$(ls -A "$BIN_DIR" 2>/dev/null)" ]; then
    echo "⚠️  Project appears to be not built yet."
    echo "Would you like to build it now? (y/n)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        echo "🏗️ Building project..."
        if [ -f "$PROJECT_DIR/build_project.sh" ]; then
            bash "$PROJECT_DIR/build_project.sh"
        else
            echo "❌ Build script not found. Please build the project manually."
            exit 1
        fi
    else
        echo "⚠️  Starting GUI without building. Some features may not work."
    fi
fi

# Set environment variables for better GUI experience
export DISPLAY=${DISPLAY:-:0}

# Check if running in a desktop environment
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    echo "⚠️  Warning: No display environment detected."
    echo "Make sure you're running this in a desktop environment or with X11 forwarding enabled."
fi

# Check Aurora SDK availability
AURORA_SDK_DIR="$PROJECT_DIR/../aurora_sdk/aurora_remote_public"
if [ ! -d "$AURORA_SDK_DIR" ]; then
    echo "⚠️  Warning: Aurora SDK not found at expected location: $AURORA_SDK_DIR"
    echo "Some features may not work properly."
fi

# Set library path for Aurora SDK
ARCH=$(uname -m)
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    PLATFORM_DIR="linux_aarch64"
else
    PLATFORM_DIR="linux_x86_64"
fi

AURORA_SDK_LIB_DIR="$AURORA_SDK_DIR/lib/$PLATFORM_DIR"
if [ -d "$AURORA_SDK_LIB_DIR" ]; then
    export LD_LIBRARY_PATH="$AURORA_SDK_LIB_DIR:$LD_LIBRARY_PATH"
    echo "✅ Aurora SDK library path set: $AURORA_SDK_LIB_DIR"
fi

# Set library path for project binaries
if [ -d "$BIN_DIR" ]; then
    export LD_LIBRARY_PATH="$BIN_DIR:$LD_LIBRARY_PATH"
fi

# Change to project directory
cd "$PROJECT_DIR"

echo "🖥️ Starting Aurora GUI..."
echo "   Project Directory: $PROJECT_DIR"
echo "   Python Version: $(python3 --version)"
echo "   Architecture: $ARCH"
echo ""

# Start the GUI
python3 aurora_gui.py

echo ""
echo "👋 Aurora GUI has been closed."
