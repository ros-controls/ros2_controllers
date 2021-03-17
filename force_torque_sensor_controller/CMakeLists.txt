cmake_minimum_required(VERSION 3.5)
project(force_torque_sensor_controller)

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
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(
  force_torque_sensor_controller
  SHARED
  src/force_torque_sensor_controller.cpp
)
target_include_directories(
  force_torque_sensor_controller
  PUBLIC
  include
)
ament_target_dependencies(
  force_torque_sensor_controller
  geometry_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# prevent pluginlib from using boost
target_compile_definitions(force_torque_sensor_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  controller_interface force_torque_sensor_controller.xml)

install(
  TARGETS
  force_torque_sensor_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(
    test_load_force_torque_sensor_controller 
    test/test_load_force_torque_sensor_controller.cpp
  )
  target_include_directories(test_load_force_torque_sensor_controller PRIVATE include)
  ament_target_dependencies(test_load_force_torque_sensor_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
  target_compile_definitions(test_load_force_torque_sensor_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_force_torque_sensor_controller 
    test/test_force_torque_sensor_controller.cpp
  )
  target_include_directories(test_force_torque_sensor_controller PRIVATE include)
  target_link_libraries(test_force_torque_sensor_controller
    force_torque_controller
  )
endif()

ament_target_dependencies(test_load_force_torque_sensor_controller
  controller_manager
  hardware_interface
  ros2_control_test_assets
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  force_torque_sensor_controller
)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  hardware_interface
)

ament_package()
