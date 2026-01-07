#!/bin/bash
# Aurora Project Dependencies Installation Script for Ubuntu 22.04
# This script installs all necessary dependencies for the Aurora Point Cloud Processing project

set -e  # Exit on any error

echo "🚀 Aurora Project Dependencies Installation for Ubuntu 22.04"
echo "============================================================="
echo ""

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "❌ This script is designed for Ubuntu. Current OS:"
    cat /etc/os-release | grep PRETTY_NAME
    echo "Please install dependencies manually or adapt this script."
    exit 1
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
echo "📋 Detected Ubuntu version: $UBUNTU_VERSION"

if [[ "$UBUNTU_VERSION" < "20.04" ]]; then
    echo "⚠️  Warning: This script is optimized for Ubuntu 20.04+. Your version may need manual adjustments."
fi

echo ""

# Update package list
echo "📦 Updating package list..."
sudo apt update

# Install basic build tools
echo "🔧 Installing basic build tools..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config \
    software-properties-common

# Install Python and tkinter
echo "🐍 Installing Python and GUI dependencies..."
sudo apt install -y \
    python3 \
    python3-pip \
    python3-tk \
    python3-dev

# Install PCL (Point Cloud Library) dependencies
echo "☁️ Installing Point Cloud Library (PCL) dependencies..."
sudo apt install -y \
    libpcl-dev \
    pcl-tools \
    libpcl-common1.12 \
    libpcl-features1.12 \
    libpcl-filters1.12 \
    libpcl-io1.12 \
    libpcl-kdtree1.12 \
    libpcl-keypoints1.12 \
    libpcl-ml1.12 \
    libpcl-octree1.12 \
    libpcl-outofcore1.12 \
    libpcl-people1.12 \
    libpcl-recognition1.12 \
    libpcl-registration1.12 \
    libpcl-sample-consensus1.12 \
    libpcl-search1.12 \
    libpcl-segmentation1.12 \
    libpcl-stereo1.12 \
    libpcl-surface1.12 \
    libpcl-tracking1.12 \
    libpcl-visualization1.12

# Install additional dependencies for point cloud processing
echo "📊 Installing additional dependencies..."
sudo apt install -y \
    libeigen3-dev \
    libflann-dev \
    libvtk9-dev \
    libboost-all-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libgtest-dev \
    freeglut3-dev \
    pkg-config \
    libxmu-dev \
    libxi-dev \
    mono-complete \
    qt5-default || true  # qt5-default might not be available in newer versions

# Try alternative Qt installation for newer Ubuntu versions
if ! dpkg -l | grep -q qt5-default; then
    echo "📱 Installing Qt5 (alternative method)..."
    sudo apt install -y \
        qtbase5-dev \
        qtchooser \
        qt5-qmake \
        qtbase5-dev-tools || true
fi

# Install file manager utilities
echo "📁 Installing file manager utilities..."
sudo apt install -y \
    xdg-utils \
    nautilus || true

# Install Python packages
echo "🐍 Installing Python packages..."
pip3 install --user \
    pathlib2 \
    pyinstaller

# Check if we're on ARM64 (Jetson Nano)
ARCH=$(uname -m)
echo "🏗️ Detected architecture: $ARCH"

if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    echo "🤖 Detected ARM64 architecture (likely Jetson Nano)"
    echo "Installing additional ARM64-specific optimizations..."
    
    # Install CUDA if available (for Jetson Nano)
    if command -v nvcc &> /dev/null; then
        echo "🚀 CUDA detected, installing CUDA-related packages..."
        sudo apt install -y \
            cuda-toolkit-11-4 \
            libcudnn8-dev || true
    else
        echo "ℹ️  CUDA not detected, skipping CUDA packages"
    fi
fi

# Verify installations
echo ""
echo "✅ Verifying installations..."

# Check CMake
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -n1)
    echo "✅ CMake: $CMAKE_VERSION"
else
    echo "❌ CMake not found"
fi

# Check Python
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version)
    echo "✅ Python: $PYTHON_VERSION"
else
    echo "❌ Python3 not found"
fi

# Check PCL
if pkg-config --exists pcl_common; then
    PCL_VERSION=$(pkg-config --modversion pcl_common)
    echo "✅ PCL: $PCL_VERSION"
else
    echo "❌ PCL not found or not properly configured"
fi

# Check tkinter
if python3 -c "import tkinter" 2>/dev/null; then
    echo "✅ Python tkinter: Available"
else
    echo "❌ Python tkinter not available"
fi

echo ""
echo "🎉 Dependencies installation completed!"
echo ""
echo "📋 Next steps:"
echo "1. Run './build_project.sh' to build the project"
echo "2. Run 'python3 aurora_gui.py' to start the GUI"
echo ""
echo "💡 If you encounter any issues:"
echo "   - Make sure you're running on Ubuntu 20.04 or later"
echo "   - Check that all dependencies are properly installed"
echo "   - Verify that the Aurora SDK libraries are in the correct location"
echo ""
echo "🔗 For more information, see the project README.md"
