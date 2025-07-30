cmake_minimum_required(VERSION 3.5)
project(seer_robotic_pkg)

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
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ### Dot env ###
  scripts/.env
  ### Main scripts ###
  scripts/n_robot_state.py
  scripts/n_robot_navigation.py
  scripts/n_task_management.py
  ### Backend scripts ###
  scripts/bn_client_api.py
  scripts/bn_msg_packer.py
  scripts/bn_pallet_loader.py
  scripts/bn_robot_navigation_api.py
  scripts/bn_robot_status_api.py
  scripts/bn_json_command_builder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  # cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  configs
  launch
  # data
  DESTINATION share/${PROJECT_NAME})

ament_package()
