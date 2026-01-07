cmake_minimum_required(VERSION 3.5)
project(slamware_ros_sdk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib)

if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR "${slamware_sdk_INCLUDE_DIR}")
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  message( FATAL_ERROR "SDK files not found. Please ensure rpos.h exists in slamware_sdk/include/rpos/ and librpos_framework.a exists in slamware_sdk/lib/" )
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)

# Determine platform and architecture
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/win64")
    else()
        message(FATAL_ERROR "Unsupported architecture for Windows. Only x64 is supported.")
    endif()
elseif(UNIX AND NOT APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/linux_aarch64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(AURORA_LIB_PATH "${PROJECT_SOURCE_DIR}/../aurora_remote_public/lib/linux_x86_64")
    else()
        message(FATAL_ERROR "Unsupported architecture for Linux. Only aarch64 and x86_64 are supported.")
    endif()
else()
    message(FATAL_ERROR "Unsupported platform. Only Windows and Linux are supported.")
endif()

find_package(OpenCV REQUIRED)

# Set the library name
set(AURORA_LIB_NAME slamtec_aurora_remote_sdk)

# Add the determined library path to the linker search path
include_directories(${PROJECT_SOURCE_DIR}/../aurora_remote_public/include)
link_directories(${AURORA_LIB_PATH})

message(STATUS "Aurora SDK library path: ${AURORA_LIB_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Threads) 

rosidl_generate_interfaces(${PROJECT_NAME} 
 msg/OptionalBool.msg
 msg/OptionalInt8.msg
 msg/OptionalInt16.msg
 msg/OptionalInt32.msg
 msg/OptionalInt64.msg
 msg/OptionalUInt8.msg
 msg/OptionalUInt16.msg
 msg/OptionalUInt32.msg
 msg/OptionalUInt64.msg
 msg/OptionalFlt32.msg
 msg/OptionalFlt64.msg
 msg/RectInt32.msg
 msg/RectFlt32.msg
 msg/Vec2DFlt32.msg
 msg/Vec2DInt32.msg
 msg/RobotDeviceInfo.msg
 msg/MapKind.msg
 msg/RobotBasicState.msg
 msg/SyncMapRequest.msg
 msg/ClearMapRequest.msg
 msg/SetMapUpdateRequest.msg
 msg/SetMapLocalizationRequest.msg
 msg/RelocalizationStatus.msg
 msg/RelocalizationCancelRequest.msg
 msg/SystemStatus.msg
 srv/SyncGetStcm.srv
 srv/SyncSetStcm.srv
 srv/RelocalizationRequest.srv
 DEPENDENCIES nav_msgs sensor_msgs std_msgs
)

ament_export_include_directories(include)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(${AMENT_PREFIX_PATH})
include_directories($ENV{AMENT_PREFIX_PATH}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/build/slamware_ros_sdk/rosidl_generator_cpp)
include_directories(${OpenCV_INCLUDE_DIRS})

# add_library(${PROJECT_NAME}_client STATIC
#   src/client/slamware_ros_sdk_client.cpp
# )

add_executable(slamware_ros_sdk_client_node 
src/client/slamware_ros_sdk_client.cpp 
src/client/slamware_ros_sdk_client_node.cpp)

ament_target_dependencies(slamware_ros_sdk_client_node rclcpp std_msgs tf2_ros cv_bridge sensor_msgs OpenCV)
rosidl_target_interfaces(slamware_ros_sdk_client_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(slamware_ros_sdk_server_node
  src/server/msg_convert.cpp
  src/server/server_params.cpp
  src/server/server_map_holder.cpp
  src/server/server_work_data.cpp
  src/server/server_worker_base.cpp
  src/server/server_workers.cpp
  src/server/slamware_ros_sdk_server_node.cpp
  src/server/slamware_ros_sdk_server.cpp
)

ament_target_dependencies(slamware_ros_sdk_server_node rclcpp std_msgs std_srvs tf2_ros tf2_geometry_msgs cv_bridge sensor_msgs OpenCV)
rosidl_target_interfaces(slamware_ros_sdk_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(slamware_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(slamware_ros_sdk_server_node 
  PRIVATE -Wno-deprecated-declarations
)

target_link_libraries(slamware_ros_sdk_server_node 
  ${AURORA_LIB_PATH}/libslamtec_aurora_remote_sdk.so
  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
  ${SLTC_SDK_LIB_DIR}/libbase64.a
  ${SLTC_SDK_LIB_DIR}/librlelib.a
  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
  ${SLTC_SDK_LIB_DIR}/libcurl.a
  ${SLTC_SDK_LIB_DIR}/libcares.a
  ${SLTC_SDK_LIB_DIR}/libssl.a
  ${SLTC_SDK_LIB_DIR}/libcrypto.a
  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
  ${SLTC_SDK_LIB_DIR}/libboost_regex.a 
  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
  ${SLTC_SDK_LIB_DIR}/libboost_system.a
  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
  ${SLTC_SDK_LIB_DIR}/libboost_random.a
  ${SLTC_SDK_LIB_DIR}/libz.a
  pthread
  dl
  rt
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  slamware_ros_sdk_server_node
  slamware_ros_sdk_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY 
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
