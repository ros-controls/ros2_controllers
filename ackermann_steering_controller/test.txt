cmake_minimum_required(VERSION 3.8)
project(ros2_ackermann_cont)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

cmake_minimum_required(VERSION 3.8)
project(ros2_ackermann_cont)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add ackermann_steering_controller_ros2 library related compile commands
generate_parameter_library(ackermann_steering_controller_ros2_parameters
  src/ackermann_steering_controller_ros2.yaml
  # include/ros2_ackermann_cont/validate_ackermann_steering_controller_ros2_parameters.hpp
)
add_library(
  ackermann_steering_controller_ros2
  SHARED
  src/ackermann_steering_controller_ros2.cpp
  src/odometry.cpp
)
target_include_directories(ackermann_steering_controller_ros2 PRIVATE include)
target_link_libraries(ackermann_steering_controller_ros2 ackermann_steering_controller_ros2_parameters)
ament_target_dependencies(ackermann_steering_controller_ros2 ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(ackermann_steering_controller_ros2 PRIVATE "ACKERMANN_STEERING_CONTROLLER_ROS2_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface ros2_ackermann_cont.xml)

install(
  TARGETS
  ackermann_steering_controller_ros2
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
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_ackermann_steering_controller_ros2 test/test_load_ackermann_steering_controller_ros2.cpp)
  target_include_directories(test_load_ackermann_steering_controller_ros2 PRIVATE include)
  ament_target_dependencies(
    test_load_ackermann_steering_controller_ros2
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # add_rostest_with_parameters_gmock(test_ackermann_steering_controller_ros2 test/# # test_ackermann_steering_controller_ros2.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/ackermann_steering_controller_ros2_params.yaml)
  # target_include_directories(test_ackermann_steering_controller_ros2 PRIVATE include)
  # target_link_libraries(test_ackermann_steering_controller_ros2 ackermann_steering_controller_ros2)
  #ament_target_dependencies(
  #  test_ackermann_steering_controller_ros2
  #  controller_interface
  #  hardware_interface
  #)

  # add_rostest_with_parameters_gmock(test_ackermann_steering_controller_ros2_preceeding test/test_ackermann_steering_controller_ros2_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/ackermann_steering_controller_ros2_preceeding_params.yaml)
  # target_include_directories(test_ackermann_steering_controller_ros2_preceeding PRIVATE include)
  # target_link_libraries(test_ackermann_steering_controller_ros2_preceeding ackermann_steering_controller_ros2)
  # ament_target_dependencies(
  #   test_ackermann_steering_controller_ros2_preceeding
  #   controller_interface
  #   hardware_interface
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  
)
ament_export_libraries(
  ackermann_steering_controller_ros2
)

ament_package()
