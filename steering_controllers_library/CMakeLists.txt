cmake_minimum_required(VERSION 3.16)
project(steering_controllers_library)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
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
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(steering_controllers_library_parameters
  src/steering_controllers_library.yaml
)

add_library(
  steering_controllers_library
  SHARED
  src/steering_controllers_library.cpp
  src/steering_kinematics.cpp
  src/steering_odometry.cpp
)
target_compile_features(steering_controllers_library PUBLIC cxx_std_17)
target_include_directories(steering_controllers_library PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/steering_controllers_library>")
target_link_libraries(steering_controllers_library PUBLIC
                      steering_controllers_library_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      tf2::tf2
                      tf2_geometry_msgs::tf2_geometry_msgs
                      ${tf2_msgs_TARGETS}
                      ${geometry_msgs_TARGETS}
                      ${control_msgs_TARGETS}
                      ${nav_msgs_TARGETS}
                      ${std_srvs_TARGETS})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_rostest_with_parameters_gmock(
    test_steering_controllers_library test/test_steering_controllers_library.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/steering_controllers_library_params.yaml)
  target_include_directories(test_steering_controllers_library PRIVATE include)
  target_link_libraries(test_steering_controllers_library steering_controllers_library)

  ament_add_gmock(test_steering_odometry test/test_steering_odometry.cpp)
  target_link_libraries(test_steering_odometry steering_controllers_library)

endif()

install(
  DIRECTORY include/
  DESTINATION include/steering_controllers_library
)

install(
  TARGETS steering_controllers_library steering_controllers_library_parameters
  EXPORT export_steering_controllers_library
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_steering_controllers_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
