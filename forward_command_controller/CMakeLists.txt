cmake_minimum_required(VERSION 3.5)
project(forward_command_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(forward_command_controller
  SHARED
  src/forward_command_controller.cpp
)
target_include_directories(forward_command_controller PRIVATE include)
ament_target_dependencies(forward_command_controller
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(forward_command_controller PRIVATE "FORWARD_COMMAND_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface forward_command_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  forward_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_forward_command_controller
    test/test_load_forward_command_controller.cpp
  )
  target_include_directories(test_load_forward_command_controller PRIVATE include)
  ament_target_dependencies(test_load_forward_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_forward_command_controller
    test/test_forward_command_controller.cpp
  )
  target_include_directories(test_forward_command_controller PRIVATE include)
  target_link_libraries(test_forward_command_controller
    forward_command_controller
  )
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  forward_command_controller
)
ament_package()
