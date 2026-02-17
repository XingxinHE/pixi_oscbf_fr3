cmake_minimum_required(VERSION 3.8)
project(oscbf_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)
find_package(oscbf_control_msgs REQUIRED)

# Install Python package code
# The Python package directory is named oscbf_control
ament_python_install_package(${PROJECT_NAME})

# C++ Executables
add_executable(franka_impedance_controller src/franka_impedance_controller.cpp)

target_include_directories(franka_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(franka_impedance_controller
  franka
)

ament_target_dependencies(franka_impedance_controller
  rclcpp
  sensor_msgs
  std_msgs
  oscbf_control_msgs
)

# Install C++ targets
install(TARGETS franka_impedance_controller
  DESTINATION lib/${PROJECT_NAME})

# Install Python nodes from scripts directory
install(PROGRAMS
  scripts/franka_control_node.py
  scripts/oculus_node.py
  scripts/pybullet_sim_node.py
  scripts/traj_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
