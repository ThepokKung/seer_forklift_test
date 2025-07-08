cmake_minimum_required(VERSION 3.5)
project(seer_robotic)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 (updated from C++14 for Jazzy)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
find_package(geometry_msgs REQUIRED)		# message types
find_package(sensor_msgs REQUIRED)		# message types
find_package(std_msgs REQUIRED)		# message types
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/robot_state.py
  scripts/test_robot_state.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts directory (including backend package)
install(DIRECTORY
  scripts/backend/
  DESTINATION lib/${PROJECT_NAME}/backend
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install config files to lib directory for script access
install(DIRECTORY
  config/
  DESTINATION lib/${PROJECT_NAME}/config
  FILES_MATCHING
  PATTERN "*.yaml"
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  config
  launch
  logs
  DESTINATION share/${PROJECT_NAME})

ament_package()
