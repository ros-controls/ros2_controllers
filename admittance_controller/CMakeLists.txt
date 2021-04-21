cmake_minimum_required(VERSION 3.5)
project(admittance_controller)

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
  admittance_controller
  SHARED
  src/admittance_controller.cpp
)
target_include_directories(
  admittance_controller
  PUBLIC
  include
)
ament_target_dependencies(
  admittance_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(admittance_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)

install(
  TARGETS admittance_controller
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

  ament_add_gmock(test_admittance_controller test/test_load_admittance_controller.cpp)
  target_include_directories(test_admittance_controller PRIVATE include)
  ament_target_dependencies(
    test_admittance_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_package()
