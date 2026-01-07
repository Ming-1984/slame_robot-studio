#!/bin/bash
# Aurora Point Cloud Processor - Wall Enhancement Version
# This script focuses on maximizing the preservation of wall details in point cloud processing

# Parse command line arguments
OUTPUT_DIR=""
NO_PROMPT=false
AUTO_ADVANCED=false
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -OutputDir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -NoPrompt)
            NO_PROMPT=true
            shift
            ;;
        -AutoAdvanced)
            AUTO_ADVANCED=true
            NO_PROMPT=true
            shift
            ;;
        -Verbose)
            VERBOSE=true
            shift
            ;;
        *)
            echo "Unknown parameter: $1"
            shift
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"
BIN_DIR="$PROJECT_DIR/bin/Release"
DATA_DIR="$PROJECT_DIR/data"

# Use custom output directory if specified
if [ -n "$OUTPUT_DIR" ]; then
    DATA_DIR="$OUTPUT_DIR"
    echo -e "\033[36mUsing custom output directory: $DATA_DIR\033[0m"
fi

echo -e "\033[35mAurora Point Cloud Processor - Wall Enhancement Version\033[0m"
echo -e "\033[35mThis version focuses on maximizing the preservation of building wall details\033[0m"
echo ""

# Create build directory if not exists
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
fi

# Use UTF-8 encoded source file
SOURCE_FILE="$PROJECT_DIR/src/cloud_processor_tool_utf8.cpp"
TARGET_FILE="$PROJECT_DIR/src/cloud_processor_tool.cpp"

if [ -f "$SOURCE_FILE" ]; then
    cp "$SOURCE_FILE" "$TARGET_FILE"
    echo -e "\033[32mUsing UTF-8 encoded source file for better Chinese character support\033[0m"
fi

# Ensure data directory exists
if [ ! -d "$DATA_DIR" ]; then
    mkdir -p "$DATA_DIR"
    echo -e "\033[32mCreated data directory: $DATA_DIR\033[0m"
fi

# Check for input files
INPUT_XYZ="$DATA_DIR/colored_point_cloud.xyz"
INPUT_PLY="$DATA_DIR/colored_point_cloud.ply"

if [ ! -f "$INPUT_XYZ" ] && [ ! -f "$INPUT_PLY" ]; then
    echo -e "\033[31mError: No input point cloud files found in $DATA_DIR\033[0m"
    echo "Expected files: colored_point_cloud.xyz or colored_point_cloud.ply"
    echo "Please run the point cloud extraction step first."
    exit 1
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
EXE_PATH="$BIN_DIR/cloud_processor_tool"
if [ ! -f "$EXE_PATH" ]; then
    echo -e "\033[31mError: Executable not found: $EXE_PATH\033[0m"
    exit 1
fi

# Process point cloud
echo ""
echo -e "\033[36mProcessing point cloud with wall enhancement...\033[0m"
cd "$PROJECT_DIR"

# Use PLY file if available, otherwise use XYZ
if [ -f "$INPUT_PLY" ]; then
    INPUT_FILE="$INPUT_PLY"
    echo "Using PLY input file: $INPUT_FILE"
else
    INPUT_FILE="$INPUT_XYZ"
    echo "Using XYZ input file: $INPUT_FILE"
fi

OUTPUT_FILE="$DATA_DIR/walls_enhanced.ply"

# Run the processor
"$EXE_PATH" "$INPUT_FILE" "$OUTPUT_FILE"

if [ $? -ne 0 ]; then
    echo -e "\033[31mError: Point cloud processing failed\033[0m"
    exit 1
fi

# Check output file
if [ -f "$OUTPUT_FILE" ]; then
    FILE_SIZE=$(du -k "$OUTPUT_FILE" | cut -f1)
    echo -e "\033[32mWall-enhanced point cloud generated: $OUTPUT_FILE (Size: ${FILE_SIZE} KB)\033[0m"
else
    echo -e "\033[31mError: Output file was not generated\033[0m"
    exit 1
fi

# Run advanced detection if requested
if [ "$AUTO_ADVANCED" = true ]; then
    echo ""
    echo -e "\033[36mRunning advanced building element detection...\033[0m"
    
    ADVANCED_SCRIPT="$SCRIPT_DIR/advanced_building_detection.sh"
    if [ -f "$ADVANCED_SCRIPT" ]; then
        bash "$ADVANCED_SCRIPT" -OutputDir "$DATA_DIR"
        if [ $? -eq 0 ]; then
            echo -e "\033[32mAdvanced detection completed successfully\033[0m"
        else
            echo -e "\033[33mWarning: Advanced detection encountered issues\033[0m"
        fi
    else
        echo -e "\033[33mWarning: Advanced detection script not found: $ADVANCED_SCRIPT\033[0m"
    fi
fi

echo ""
echo -e "\033[32mPoint cloud processing with wall enhancement completed successfully!\033[0m"
echo -e "\033[36mOutput file: $OUTPUT_FILE\033[0m"
