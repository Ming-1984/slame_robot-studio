#!/bin/bash
# Aurora Point Cloud Extractor
# This script extracts point cloud data directly from Aurora device to PLY and XYZ formats

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
BIN_DIR="$PROJECT_DIR/bin/Release"
DATA_DIR="$PROJECT_DIR/data"
SRC_DIR="$PROJECT_DIR/src"

# Parse command line arguments
DEVICE_IP="192.168.11.1"
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -DeviceIP)
            DEVICE_IP="$2"
            shift 2
            ;;
        -OutputDir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        *)
            echo "Unknown parameter: $1"
            shift
            ;;
    esac
done

echo "Aurora Point Cloud Extractor"
echo ""

# Check necessary directories and files
echo "Checking files and directories..."
if [ ! -d "$SRC_DIR" ]; then
    echo -e "\033[31mError: Source directory does not exist: $SRC_DIR\033[0m"
    exit 1
fi

if [ ! -f "$SRC_DIR/colored_point_extractor.cpp" ]; then
    echo -e "\033[31mError: Colored point extractor source file does not exist\033[0m"
    exit 1
fi

echo "Aurora Device Configuration:"
echo "Device IP: $DEVICE_IP"
echo "Connection will be established automatically"

# Create build directory if it doesn't exist
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
fi

# Use custom output directory if specified, otherwise use default data directory
if [ -n "$OUTPUT_DIR" ]; then
    DATA_DIR="$OUTPUT_DIR"
    echo -e "\033[36mUsing custom output directory: $DATA_DIR\033[0m"
fi

# Ensure data directory exists
if [ ! -d "$DATA_DIR" ]; then
    mkdir -p "$DATA_DIR"
    echo -e "\033[32mCreated data directory: $DATA_DIR\033[0m"
fi

# Build the project
echo ""
echo -e "\033[36mBuilding the project...\033[0m"
cd "$BUILD_DIR"

# Run CMake configuration
echo "Running CMake configuration..."
cmake -DCMAKE_BUILD_TYPE=Release ..

if [ $? -ne 0 ]; then
    echo -e "\033[31mError: CMake configuration failed\033[0m"
    exit 1
fi

# Build the project
echo "Building the project..."
cmake --build . --config Release

if [ $? -ne 0 ]; then
    echo -e "\033[31mError: Project build failed\033[0m"
    exit 1
fi

# Check if executable exists
EXE_PATH="$BIN_DIR/colored_extractor"
if [ ! -f "$EXE_PATH" ]; then
    echo -e "\033[31mError: Executable not found: $EXE_PATH\033[0m"
    exit 1
fi

# Define output file paths
XYZ_OUTPUT_PATH="$DATA_DIR/colored_point_cloud.xyz"
PLY_OUTPUT_PATH="$DATA_DIR/colored_point_cloud.ply"

# Prepare arguments for device connection
BASE_ARGS=("-d" "$DEVICE_IP" "-t" "10")
echo ""
echo -e "\033[36mConnecting to Aurora device at IP: $DEVICE_IP\033[0m"

# Run program - Generate XYZ format file
echo ""
echo -e "\033[36mExtracting point cloud data (XYZ format)...\033[0m"
cd "$PROJECT_DIR"
"$EXE_PATH" "${BASE_ARGS[@]}" -o "$XYZ_OUTPUT_PATH" -f xyz

if [ $? -ne 0 ]; then
    echo -e "\033[33mWarning: Error extracting XYZ format point cloud data\033[0m"
fi

# Run program - Generate PLY format file
echo ""
echo -e "\033[36mExtracting point cloud data (PLY format)...\033[0m"
cd "$PROJECT_DIR"
"$EXE_PATH" "${BASE_ARGS[@]}" -o "$PLY_OUTPUT_PATH" -f ply

if [ $? -ne 0 ]; then
    echo -e "\033[33mWarning: Error extracting PLY format point cloud data\033[0m"
fi

# Check output files
echo ""
echo -e "\033[36mChecking output files...\033[0m"
FILES_GENERATED=0

if [ -f "$XYZ_OUTPUT_PATH" ]; then
    FILE_SIZE=$(du -k "$XYZ_OUTPUT_PATH" | cut -f1)
    echo -e "\033[32mXYZ file generated: $XYZ_OUTPUT_PATH (Size: ${FILE_SIZE} KB)\033[0m"
    ((FILES_GENERATED++))
fi

if [ -f "$PLY_OUTPUT_PATH" ]; then
    FILE_SIZE=$(du -k "$PLY_OUTPUT_PATH" | cut -f1)
    echo -e "\033[32mPLY file generated: $PLY_OUTPUT_PATH (Size: ${FILE_SIZE} KB)\033[0m"
    ((FILES_GENERATED++))
fi

if [ $FILES_GENERATED -eq 0 ]; then
    echo -e "\033[31mError: No output files were generated\033[0m"
    exit 1
elif [ $FILES_GENERATED -eq 2 ]; then
    echo ""
    echo -e "\033[32mSuccess: All point cloud files have been generated and saved to the data directory\033[0m"
else
    echo ""
    echo -e "\033[33mPartial success: Some point cloud files have been generated and saved to the data directory\033[0m"
fi

echo ""
echo -e "\033[36mPoint cloud extraction complete!\033[0m"
