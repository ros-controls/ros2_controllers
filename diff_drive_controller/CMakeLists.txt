cmake_minimum_required(VERSION 3.5)
project(diff_drive_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_parameter_server REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(
  diff_drive_controller
  SHARED
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)

target_include_directories(diff_drive_controller PRIVATE include)
ament_target_dependencies(
  diff_drive_controller
  "builtin_interfaces"
  "class_loader"
  "rclcpp"
  "rclcpp_lifecycle"
  "rcutils"
  "hardware_interface"
  "controller_interface"
  "controller_manager"
  "controller_parameter_server"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "realtime_tools"
)

target_compile_definitions(diff_drive_controller PRIVATE "ROS_CONTROLLERS_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  diff_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  realtime_tools
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  diff_drive_controller
)

ament_package()