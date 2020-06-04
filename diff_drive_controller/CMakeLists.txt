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
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(urdf REQUIRED)

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
  sensor_msgs
  tf2
  tf2_msgs
  control_msgs
  urdf
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
  find_package(ros_testing REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(test_robot_hardware REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Unit test: test_load_diff_drive_controller
  ament_add_gtest(
    test_load_diff_drive_controller
    test/test_load_diff_drive_controller.cpp
  )

  target_include_directories(test_load_diff_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_diff_drive_controller
    controller_manager
    test_robot_hardware
  )
  # End Unit test: test_load_diff_drive_controller

  ament_add_gtest(
    test_accumulator
    test/test_accumulator.cpp
  )

  target_include_directories(test_accumulator PRIVATE include)
  ament_target_dependencies(test_accumulator)


  # Unit test: test_diff_drive_controller
  ament_add_gtest(test_diff_drive_controller
    test/test_diff_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_diff_drive_controller.yaml)
  target_include_directories(test_diff_drive_controller PRIVATE include)
  target_link_libraries(test_diff_drive_controller
    diff_drive_controller
  )
  ament_target_dependencies(test_diff_drive_controller test_robot_hardware)
  # End Unit test: test_diff_drive_controller

  find_package(rosgraph_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2_ros REQUIRED)

  # Diffbot
  # Robot with a DiffDriveController used for integration testing
  add_executable(diffbot test/diffbot.h test/diffbot.cpp)
  ament_target_dependencies(diffbot hardware_interface controller_manager rclcpp rosgraph_msgs lifecycle_msgs std_srvs)
  # End Diffbot

  # Gtest executables used in launch tests
  set(launch_gtests
    test_diff_drive
    test_diff_drive_nan
    test_diff_drive_limits
    test_diff_drive_timeout
    test_diff_drive_fail
    test_diff_drive_odom_tf
    test_diff_drive_default_cmd_vel_out
    test_diff_drive_pub_cmd_vel_out
    test_diff_drive_default_wheel_joint_controller_state
    test_diff_drive_pub_wheel_joint_controller_state
    test_diff_drive_default_odom_frame
    test_diff_drive_odom_frame
    test_diff_drive_multiple_cmd_vel_publishers
    )

  foreach(launch_gtest IN LISTS launch_gtests)
    ament_add_gtest_executable(${launch_gtest}
      test/test_common.h
      test/${launch_gtest}.cpp)
    ament_target_dependencies(${launch_gtest} tf2_ros std_srvs)
    target_link_libraries(${launch_gtest} diff_drive_controller)
    add_ros_test(test/${launch_gtest}.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  endforeach()

  add_ros_test(test/test_diff_drive_multipliers.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_diff_drive_open_loop.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  add_ros_test(test/test_diff_drive_radius_sphere.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_diff_drive_radius_param.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_diff_drive_radius_param_fail.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_diff_drive_separation_param.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_diff_drive_bad_urdf.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

  # Skidsteerbot
  # Robot with a DiffDriveController used for integration testing
  add_executable(skidsteerbot test/diffbot.h test/skidsteerbot.cpp)
  ament_target_dependencies(skidsteerbot hardware_interface controller_manager rclcpp rosgraph_msgs lifecycle_msgs std_srvs)
  # End Skidsteerbot

  add_ros_test(test/test_skid_steer.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
  add_ros_test(test/test_skid_steer_no_wheels.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif ()

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
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
