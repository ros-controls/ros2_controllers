cmake_minimum_required(VERSION 3.5)
project(joint_state_controller)

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
find_package(pluginlib)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(joint_state_controller
  SHARED
  src/joint_state_controller.cpp
)
target_include_directories(joint_state_controller PRIVATE include)
ament_target_dependencies(joint_state_controller
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
target_compile_definitions(joint_state_controller PRIVATE "JOINT_STATE_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(joint_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  joint_state_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(
    test_load_joint_state_controller
    test/test_load_joint_state_controller.cpp
  )
  target_include_directories(test_load_joint_state_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_state_controller
    controller_manager
  )
  target_compile_definitions(test_load_joint_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  ament_add_gmock(
    test_joint_state_controller
    test/test_joint_state_controller.cpp
  )
  target_include_directories(test_joint_state_controller PRIVATE include)
  target_link_libraries(test_joint_state_controller
    joint_state_controller
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
  joint_state_controller
)
ament_package()
