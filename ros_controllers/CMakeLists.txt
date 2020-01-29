cmake_minimum_required(VERSION 3.5)
project(ros_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_parameter_server REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(
  default_controllers
  SHARED
  src/diff_drive_controller.cpp
  src/joint_trajectory_controller.cpp
  src/joint_state_controller.cpp
  src/trajectory.cpp
)
target_include_directories(default_controllers PRIVATE include)
ament_target_dependencies(
  default_controllers
  "builtin_interfaces"
  "class_loader"
  "controller_interface"
  "controller_manager"
  "controller_parameter_server"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
  "rcutils"
  "sensor_msgs"
  "trajectory_msgs"
)
controller_manager_register_controller(
  default_controllers
  "ros_controllers::JointStateController"
  "ros_controllers::JointTrajectoryController"
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(default_controllers PRIVATE "ROS_CONTROLLERS_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  default_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(controller_parameter_server)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_include_directories(test_trajectory PRIVATE include)
  target_link_libraries(test_trajectory default_controllers)

  ament_add_gtest(
    test_trajectory_controller test/test_trajectory_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_joint_trajectory_controller.yaml)
  target_include_directories(test_trajectory_controller PRIVATE include)
  target_link_libraries(
    test_trajectory_controller
    default_controllers
  )
  ament_target_dependencies(
    test_trajectory_controller
    "controller_interface"
    "controller_parameter_server"
    "hardware_interface"
    "lifecycle_msgs"
    "rclcpp"
    "rcutils"
    "test_robot_hardware"
    "trajectory_msgs"
  )
endif()

ament_export_dependencies(
  controller_interface
  rclcpp_lifecycle
  sensor_msgs
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  default_controllers
)
ament_package()
