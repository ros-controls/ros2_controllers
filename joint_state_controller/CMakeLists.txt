cmake_minimum_required(VERSION 3.5)
project(joint_state_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(pluginlib REQUIRED)

add_library(joint_state_controller
  SHARED
  src/joint_state_controller.cpp
)
ament_target_dependencies(joint_state_controller
  joint_state_broadcaster
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_state_controller PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(joint_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

install(
  TARGETS
  joint_state_controller
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
    test_load_joint_state_controller
    test/test_load_joint_state_controller.cpp
  )
  target_include_directories(test_load_joint_state_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_state_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
  target_compile_definitions(test_load_joint_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
endif()

ament_export_dependencies(
  joint_state_broadcaster
)
ament_export_libraries(
  joint_state_controller
)
ament_package()
