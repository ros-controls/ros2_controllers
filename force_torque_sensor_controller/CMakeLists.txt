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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

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
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_force_torque_sensor_controller test/test_load_force_torque_sensor_controller.cpp)
  target_include_directories(test_force_torque_sensor_controller PRIVATE include)
  ament_target_dependencies(
    test_force_torque_sensor_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  force_torque_sensor_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
