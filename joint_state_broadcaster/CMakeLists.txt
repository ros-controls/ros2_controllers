cmake_minimum_required(VERSION 3.5)
project(joint_state_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(joint_state_broadcaster
  SHARED
  src/joint_state_broadcaster.cpp
)
target_include_directories(joint_state_broadcaster PRIVATE include)
ament_target_dependencies(joint_state_broadcaster
  builtin_interfaces
  control_msgs
  controller_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  sensor_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_state_broadcaster PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  joint_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_joint_state_broadcaster
    test/test_load_joint_state_broadcaster.cpp
  )
  target_include_directories(test_load_joint_state_broadcaster PRIVATE include)
  ament_target_dependencies(test_load_joint_state_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_joint_state_broadcaster
    test/test_joint_state_broadcaster.cpp
  )
  target_include_directories(test_joint_state_broadcaster PRIVATE include)
  target_link_libraries(test_joint_state_broadcaster
    joint_state_broadcaster
  )
endif()

ament_export_dependencies(
  control_msgs
  controller_interface
  rclcpp_lifecycle
  sensor_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  joint_state_broadcaster
)
ament_package()
