cmake_minimum_required(VERSION 3.5)
project(diff_drive_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(diff_drive_controller SHARED
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_include_directories(diff_drive_controller PRIVATE include)
ament_target_dependencies(diff_drive_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diff_drive_controller PRIVATE "DIFF_DRIVE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface diff_drive_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS diff_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_diff_drive_controller
    test/test_diff_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_diff_drive_controller.yaml)
  target_include_directories(test_diff_drive_controller PRIVATE include)
  target_link_libraries(test_diff_drive_controller
    diff_drive_controller
  )

  ament_target_dependencies(test_diff_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_diff_drive_controller
    test/test_load_diff_drive_controller.cpp
  )
  target_include_directories(test_load_diff_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_diff_drive_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_accumulator
    test/test_accumulator.cpp
  )

  target_include_directories(test_accumulator PRIVATE include)
  ament_target_dependencies(test_accumulator)

endif()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  diff_drive_controller
)
ament_package()
