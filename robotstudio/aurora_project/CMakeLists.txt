cmake_minimum_required(VERSION 3.10)
project(AuroraPointCloudProcessor)

# Detect platform and architecture
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(PLATFORM_DIR "linux_aarch64")
        message(STATUS "Detected platform: Linux ARM64")
    else()
        set(PLATFORM_DIR "linux_x86_64")
        message(STATUS "Detected platform: Linux x86_64")
    endif()
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PLATFORM_DIR "win64")
    message(STATUS "Detected platform: Windows x64")
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Find required packages
find_package(PCL 1.8 REQUIRED)

# Aurora SDK configuration
set(AURORA_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../aurora_sdk/aurora_remote_public")
set(AURORA_SDK_INCLUDE_DIR "${AURORA_SDK_ROOT}/include")
set(AURORA_SDK_LIB_DIR "${AURORA_SDK_ROOT}/lib/${PLATFORM_DIR}")

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${AURORA_SDK_INCLUDE_DIR}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${AURORA_SDK_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/../bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/../bin/Release)

# Add source files
set(SOURCES
    src/cloud_processor.cpp
    src/cloud_visualizer.cpp
)

# Create main library
add_library(aurora_lib STATIC ${SOURCES})
target_link_libraries(aurora_lib ${PCL_LIBRARIES})

# Aurora SDK library name (platform-specific)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(AURORA_SDK_LIBRARY "slamtec_aurora_remote_sdk")
else()
    set(AURORA_SDK_LIBRARY "slamtec_aurora_remote_sdk")
endif()

# Add executables
add_executable(cloud_processor_tool src/cloud_processor_tool.cpp)
target_link_libraries(cloud_processor_tool ${PCL_LIBRARIES})

# Advanced building element detector tool
add_executable(advanced_building_detector src/advanced_building_detector.cpp)
target_link_libraries(advanced_building_detector aurora_lib ${PCL_LIBRARIES})

# Colored point extractor tool (Aurora SDK) - Now properly configured
add_executable(colored_extractor src/colored_point_extractor.cpp)
target_link_libraries(colored_extractor ${AURORA_SDK_LIBRARY} ${PCL_LIBRARIES})

# Door window detector tool (if needed)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/door_window_detector.cpp")
    add_executable(door_window_detector src/door_window_detector.cpp)
    target_link_libraries(door_window_detector aurora_lib ${PCL_LIBRARIES})
endif()

# Aurora SDK compatibility test
add_executable(aurora_sdk_test src/aurora_sdk_test.cpp)
target_link_libraries(aurora_sdk_test ${AURORA_SDK_LIBRARY})

# Platform-specific library deployment
if(WIN32)
    # Copy DLLs for Windows
    if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
        set(PCL_BIN_DIR "${PCL_ROOT}/bin")
        file(GLOB PCL_DLLS "${PCL_BIN_DIR}/*.dll")
        file(COPY ${PCL_DLLS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
        file(COPY ${PCL_DLLS} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)

        # Copy Aurora SDK DLL
        file(COPY "${AURORA_SDK_LIB_DIR}/slamtec_aurora_remote_sdk.dll"
             DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Debug)
        file(COPY "${AURORA_SDK_LIB_DIR}/slamtec_aurora_remote_sdk.dll"
             DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
    endif()
else()
    # Copy shared libraries for Linux
    file(GLOB AURORA_SDK_SO "${AURORA_SDK_LIB_DIR}/*.so*")
    if(AURORA_SDK_SO)
        file(COPY ${AURORA_SDK_SO} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
        message(STATUS "Copied Aurora SDK shared libraries: ${AURORA_SDK_SO}")
    endif()

    # Set RPATH for Linux executables to find shared libraries
    set_target_properties(colored_extractor PROPERTIES
        INSTALL_RPATH_USE_LINK_PATH TRUE
        BUILD_WITH_INSTALL_RPATH TRUE
        INSTALL_RPATH "$ORIGIN:${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

# Print configuration summary
message(STATUS "Aurora SDK include directory: ${AURORA_SDK_INCLUDE_DIR}")
message(STATUS "Aurora SDK library directory: ${AURORA_SDK_LIB_DIR}")
message(STATUS "Aurora SDK library: ${AURORA_SDK_LIBRARY}")
message(STATUS "Platform directory: ${PLATFORM_DIR}")
