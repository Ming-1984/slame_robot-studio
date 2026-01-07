cmake_minimum_required(VERSION 3.8)

project(acamana)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

# Install Python scripts
install(PROGRAMS
	scripts/topic_relay_node.py
	DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
	config
	launch
	meshes
	models
	urdf
	textures
	worlds
	DESTINATION share/${PROJECT_NAME}/
)

ament_package()
