cmake_minimum_required(VERSION 3.5)
project(imu_sensor_broadcaster)

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
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(
  imu_sensor_broadcaster
  SHARED
  src/imu_sensor_broadcaster.cpp
)
target_include_directories(
  imu_sensor_broadcaster
  PUBLIC
  include
)
ament_target_dependencies(
  imu_sensor_broadcaster
  sensor_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(imu_sensor_broadcaster PRIVATE "IMU_SENSOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface imu_sensor_broadcaster.xml)

install(
  TARGETS
  imu_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_imu_sensor_broadcaster
    test/test_load_imu_sensor_broadcaster.cpp
  )
  target_include_directories(test_load_imu_sensor_broadcaster PRIVATE include)
  ament_target_dependencies(test_load_imu_sensor_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_imu_sensor_broadcaster
    test/test_imu_sensor_broadcaster.cpp
  )
  target_include_directories(test_imu_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_imu_sensor_broadcaster
    imu_sensor_broadcaster
  )
  ament_target_dependencies(test_imu_sensor_broadcaster
    hardware_interface
  )
  ament_target_dependencies(test_load_imu_sensor_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  imu_sensor_broadcaster
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
