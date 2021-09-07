cmake_minimum_required(VERSION 3.5)
project(joint_trajectory_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(joint_trajectory_controller SHARED
  src/joint_trajectory_controller.cpp
  src/trajectory.cpp
)
target_include_directories(joint_trajectory_controller PRIVATE include)
ament_target_dependencies(joint_trajectory_controller
  angles
  builtin_interfaces
  controller_interface
  control_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_trajectory_controller PRIVATE "JOINT_TRAJECTORY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_include_directories(test_trajectory PRIVATE include)
  target_link_libraries(test_trajectory joint_trajectory_controller)

  ament_add_gtest(test_trajectory_controller
    test/test_trajectory_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_joint_trajectory_controller.yaml)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 180)
  target_include_directories(test_trajectory_controller PRIVATE include)
  target_link_libraries(test_trajectory_controller
    joint_trajectory_controller
  )
  ament_target_dependencies(test_trajectory_controller
    control_msgs
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    ros2_control_test_assets
    trajectory_msgs
  )

  ament_add_gtest(
    test_load_joint_trajectory_controller
    test/test_load_joint_trajectory_controller.cpp
  )
  target_include_directories(test_load_joint_trajectory_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_trajectory_controller
    controller_manager
    realtime_tools
    ros2_control_test_assets
  )

  ament_add_gtest(
    test_trajectory_actions
    test/test_trajectory_actions.cpp
  )
  target_include_directories(test_trajectory_actions PRIVATE include)
  target_link_libraries(test_trajectory_actions
    joint_trajectory_controller
  )
  ament_target_dependencies(test_trajectory_actions
    control_msgs
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    ros2_control_test_assets
    trajectory_msgs
    realtime_tools
  )
endif()

ament_export_dependencies(
  controller_interface
  control_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  joint_trajectory_controller
)
ament_package()
