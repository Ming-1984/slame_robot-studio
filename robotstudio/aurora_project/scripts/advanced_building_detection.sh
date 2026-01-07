#!/bin/bash
# Advanced Building Element Detection Script
# 高级建筑元素检测脚本 - 检测门窗、墙面、地面、屋顶并进行智能着色

# Default values
INPUT_FILE=""
OUTPUT_DIR=""
INPUT_DIR=""
VERBOSE=false
HELP=false

# Function to show help
show_help() {
    echo -e "\033[33mAurora Advanced Building Element Detection Script\033[0m"
    echo -e "\033[33m=================================================\033[0m"
    echo ""
    echo -e "\033[32mThis script performs advanced building element detection including:\033[0m"
    echo -e "  🧱 Wall detection and classification"
    echo -e "  🚪 Door detection and marking"
    echo -e "  🪟 Window detection and marking"
    echo -e "  🟢 Floor/ground detection"
    echo -e "  🔴 Ceiling/roof detection"
    echo ""
    echo -e "\033[36mUsage:\033[0m"
    echo -e "\033[37m  ./advanced_building_detection.sh -InputFile <file> -OutputDir <dir> [-Verbose]\033[0m"
    echo -e "\033[37m  ./advanced_building_detection.sh -InputDir <dir> -OutputDir <dir> [-Verbose]\033[0m"
    echo ""
    echo -e "\033[36mParameters:\033[0m"
    echo -e "\033[37m  -InputFile    : Input PLY file path\033[0m"
    echo -e "\033[37m  -InputDir     : Input directory (uses walls_enhanced_cloud.ply)\033[0m"
    echo -e "\033[37m  -OutputDir    : Output directory for results\033[0m"
    echo -e "\033[37m  -Verbose      : Enable verbose output\033[0m"
    echo -e "\033[37m  -Help         : Show this help message\033[0m"
    echo ""
    echo -e "\033[36mExamples:\033[0m"
    echo -e "\033[37m  ./advanced_building_detection.sh -InputDir '/data' -OutputDir '/output' -Verbose\033[0m"
    echo -e "\033[37m  ./advanced_building_detection.sh -InputFile 'input.ply' -OutputDir '/output'\033[0m"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -InputFile)
            INPUT_FILE="$2"
            shift 2
            ;;
        -OutputDir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -InputDir)
            INPUT_DIR="$2"
            shift 2
            ;;
        -Verbose)
            VERBOSE=true
            shift
            ;;
        -Help)
            HELP=true
            shift
            ;;
        *)
            echo "Unknown parameter: $1"
            shift
            ;;
    esac
done

if [ "$HELP" = true ]; then
    show_help
    exit 0
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BIN_DIR="$PROJECT_DIR/bin/Release"
BUILD_DIR="$PROJECT_DIR/build"

# Tool paths
ADVANCED_DETECTOR="$BIN_DIR/advanced_building_detector"

# Determine input file
if [ -n "$INPUT_FILE" ]; then
    if [ ! -f "$INPUT_FILE" ]; then
        echo -e "\033[31mError: Input file not found: $INPUT_FILE\033[0m"
        exit 1
    fi
elif [ -n "$INPUT_DIR" ]; then
    INPUT_FILE="$INPUT_DIR/walls_enhanced.ply"
    if [ ! -f "$INPUT_FILE" ]; then
        # Try alternative filename
        INPUT_FILE="$INPUT_DIR/walls_enhanced_cloud.ply"
        if [ ! -f "$INPUT_FILE" ]; then
            echo -e "\033[31mError: No suitable input file found in $INPUT_DIR\033[0m"
            echo "Expected: walls_enhanced.ply or walls_enhanced_cloud.ply"
            exit 1
        fi
    fi
else
    echo -e "\033[31mError: Either -InputFile or -InputDir must be specified\033[0m"
    show_help
    exit 1
fi

# Set default output directory if not specified
if [ -z "$OUTPUT_DIR" ]; then
    OUTPUT_DIR="$PROJECT_DIR/data"
fi

# Ensure output directory exists
if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir -p "$OUTPUT_DIR"
    echo -e "\033[32mCreated output directory: $OUTPUT_DIR\033[0m"
fi

echo -e "\033[35m🏗️ Starting Advanced Building Element Detection\033[0m"
echo -e "\033[36mInput file: $INPUT_FILE\033[0m"
echo -e "\033[36mOutput directory: $OUTPUT_DIR\033[0m"
echo ""

# Build the project if needed
if [ ! -f "$ADVANCED_DETECTOR" ]; then
    echo -e "\033[36mBuilding advanced building detector...\033[0m"
    cd "$BUILD_DIR"
    
    cmake -DCMAKE_BUILD_TYPE=Release ..
    if [ $? -ne 0 ]; then
        echo -e "\033[31mError: CMake configuration failed\033[0m"
        exit 1
    fi
    
    cmake --build . --config Release
    if [ $? -ne 0 ]; then
        echo -e "\033[31mError: Build failed\033[0m"
        exit 1
    fi
fi

# Check if executable exists
if [ ! -f "$ADVANCED_DETECTOR" ]; then
    echo -e "\033[31mError: Advanced building detector executable not found: $ADVANCED_DETECTOR\033[0m"
    exit 1
fi

# Run advanced building detection
echo -e "\033[36mRunning advanced building element detection...\033[0m"
echo -e "\033[32mDetecting and color-coding:\033[0m"
echo -e "🧱 Walls (Gray), 🚪 Doors (Brown), 🪟 Windows (Blue)"
echo -e "🟢 Floors (Green), 🔴 Ceilings (Red)"
echo ""

cd "$PROJECT_DIR"

OUTPUT_FILE="$OUTPUT_DIR/building_elements_detected.ply"

# Run the detector
if [ "$VERBOSE" = true ]; then
    "$ADVANCED_DETECTOR" "$INPUT_FILE" "$OUTPUT_FILE" --verbose
else
    "$ADVANCED_DETECTOR" "$INPUT_FILE" "$OUTPUT_FILE"
fi

if [ $? -ne 0 ]; then
    echo -e "\033[31mError: Advanced building detection failed\033[0m"
    exit 1
fi

# Check output file
if [ -f "$OUTPUT_FILE" ]; then
    FILE_SIZE=$(du -k "$OUTPUT_FILE" | cut -f1)
    echo ""
    echo -e "\033[32m✅ Advanced building detection completed successfully!\033[0m"
    echo -e "\033[36mOutput file: $OUTPUT_FILE (Size: ${FILE_SIZE} KB)\033[0m"
    echo ""
    echo -e "\033[32mBuilding elements detected and color-coded:\033[0m"
    echo -e "🧱 Walls - Gray color"
    echo -e "🚪 Doors - Brown color"
    echo -e "🪟 Windows - Blue color"
    echo -e "🟢 Floors - Green color"
    echo -e "🔴 Ceilings - Red color"
else
    echo -e "\033[31mError: Output file was not generated\033[0m"
    exit 1
fi

echo ""
echo -e "\033[36mAdvanced building element detection complete!\033[0m"
