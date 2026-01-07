# Aurora Point Cloud Processing - Jetson Nano Ubuntu 22.04 Guide

This guide explains how to set up and run the Aurora Point Cloud Processing project on NVIDIA Jetson Nano with Ubuntu 22.04.

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# Make the installation script executable
chmod +x install_dependencies_ubuntu.sh

# Run the dependency installation script
./install_dependencies_ubuntu.sh
```

### 2. Build the Project

```bash
# Make the build script executable
chmod +x build_project.sh

# Build the project
./build_project.sh
```

### 3. Start the GUI

```bash
# Make the startup script executable
chmod +x start_aurora_gui.sh

# Start the Aurora GUI
./start_aurora_gui.sh
```

## 📋 System Requirements

- **Hardware**: NVIDIA Jetson Nano (4GB recommended)
- **OS**: Ubuntu 22.04 LTS (ARM64)
- **Memory**: At least 4GB RAM
- **Storage**: At least 8GB free space
- **Network**: Ethernet connection for Aurora device communication

## 🔧 Manual Installation Steps

If the automatic installation script doesn't work, follow these manual steps:

### Install System Dependencies

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y build-essential cmake git wget curl pkg-config

# Install Python and GUI dependencies
sudo apt install -y python3 python3-pip python3-tk python3-dev

# Install PCL (Point Cloud Library)
sudo apt install -y libpcl-dev pcl-tools

# Install additional dependencies
sudo apt install -y libeigen3-dev libflann-dev libvtk9-dev libboost-all-dev
sudo apt install -y libqhull-dev libusb-1.0-0-dev freeglut3-dev
sudo apt install -y xdg-utils

# Install Python packages
pip3 install --user pathlib2 pyinstaller
```

### Build the Project

```bash
# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake -DCMAKE_BUILD_TYPE=Release ..

# Build (use all CPU cores)
make -j$(nproc)

# Return to project directory
cd ..
```

## 🏗️ Project Structure

```
aurora_project/
├── aurora_gui.py                    # Main GUI application (cross-platform)
├── CMakeLists.txt                   # CMake configuration (Linux-compatible)
├── src/                             # C++ source files
│   ├── colored_point_extractor.cpp  # Aurora device interface
│   ├── cloud_processor_tool.cpp     # Point cloud processing
│   ├── advanced_building_detector.cpp # Building element detection
│   └── aurora_sdk_test.cpp          # SDK compatibility test
├── scripts/                         # Processing scripts
│   ├── extract_point_cloud.sh       # Point cloud extraction (Bash)
│   ├── process_point_cloud_walls.sh # Wall enhancement (Bash)
│   ├── advanced_building_detection.sh # Building detection (Bash)
├── bin/                             # Built executables
├── data/                            # Data files and output
└── build/                           # Build files
```

## 🔌 Aurora Device Connection

1. **Network Setup**: Connect Aurora device to the same network as Jetson Nano
2. **IP Configuration**: Default IP is `192.168.11.1`, modify in GUI if needed
3. **Firewall**: Ensure port 7447 is open for Aurora communication

## 🖥️ GUI Features

The GUI provides the same functionality as the Windows version:

1. **Extract Point Cloud**: Connect to Aurora device and extract point cloud data
2. **Process Point Cloud**: Enhance walls and detect building elements
3. **Advanced Detection**: Detect doors, windows, floors, and ceilings with color coding

Note: The legacy "Convert to CAD / DXF" step has been removed from this repository. For true-scene mapping and related features, use the official host tool at `/home/jetson/ros2_ws/aurora_remote-release-2.1.0-rc2`.

## 🎨 Building Element Colors

- 🧱 **Walls**: Gray
- 🚪 **Doors**: Brown  
- 🪟 **Windows**: Blue
- 🟢 **Floors**: Green
- 🔴 **Ceilings**: Red

## 🛠️ Troubleshooting

### Common Issues

1. **"CMake not found"**
   ```bash
   sudo apt install cmake
   ```

2. **"PCL not found"**
   ```bash
   sudo apt install libpcl-dev
   ```

3. **"Python tkinter not available"**
   ```bash
   sudo apt install python3-tk
   ```

4. **"Aurora SDK library not found"**
   - Ensure the `aurora_sdk` directory is in the parent directory
   - Check that ARM64 libraries exist in `aurora_sdk/aurora_remote_public/lib/linux_aarch64/`

5. **"Permission denied" when running scripts**
   ```bash
   chmod +x *.sh
   chmod +x scripts/*.sh
   ```

6. **GUI doesn't start**
   - Ensure you're in a desktop environment
   - Check DISPLAY environment variable: `echo $DISPLAY`
   - Try: `export DISPLAY=:0`

### Performance Optimization for Jetson Nano

1. **Enable maximum performance mode**:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```

2. **Increase swap space** (if needed):
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

3. **Monitor system resources**:
   ```bash
   htop
   jtop  # If tegrastats is available
   ```

## 🧪 Testing

Run the Aurora SDK compatibility test:

```bash
./bin/aurora_sdk_test
```

This will verify that the Aurora SDK is properly configured for your platform.

## 📊 Expected Performance

On Jetson Nano 4GB:
- Point cloud extraction: 30-60 seconds
- Wall enhancement: 1-3 minutes  
- Building detection: 2-5 minutes

Performance depends on point cloud size and complexity.

## 🔗 Additional Resources

- [NVIDIA Jetson Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)
- [PCL Documentation](https://pointclouds.org/documentation/)
- [SLAMTEC Aurora Documentation](https://www.slamtec.com/en/support)

## 📞 Support

If you encounter issues specific to Jetson Nano deployment:

1. Check system logs: `dmesg | tail`
2. Monitor memory usage: `free -h`
3. Check GPU usage: `tegrastats`
4. Verify Aurora SDK: `./bin/aurora_sdk_test`

For general project issues, refer to the main project documentation.
