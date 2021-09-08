cmake_minimum_required(VERSION 3.5)
project(position_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(position_controllers
  SHARED
  src/joint_group_position_controller.cpp
)
target_include_directories(position_controllers PRIVATE include)
ament_target_dependencies(position_controllers
  forward_command_controller
  pluginlib
  rclcpp
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(position_controllers PRIVATE "POSITION_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface position_controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  position_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_joint_group_position_controller
    test/test_load_joint_group_position_controller.cpp
  )
  target_include_directories(test_load_joint_group_position_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_group_position_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_joint_group_position_controller
    test/test_joint_group_position_controller.cpp
  )
  target_include_directories(test_joint_group_position_controller PRIVATE include)
  target_link_libraries(test_joint_group_position_controller
    position_controllers
  )
endif()

ament_export_dependencies(
  forward_command_controller
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  position_controllers
)
ament_package()
