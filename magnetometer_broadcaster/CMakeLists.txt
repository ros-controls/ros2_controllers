cmake_minimum_required(VERSION 3.16)
project(magnetometer_broadcaster)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(magnetometer_broadcaster_parameters
  src/magnetometer_broadcaster_parameters.yaml
)

add_library(magnetometer_broadcaster SHARED
  src/magnetometer_broadcaster.cpp
)
target_compile_features(magnetometer_broadcaster PUBLIC cxx_std_20)
target_link_libraries(magnetometer_broadcaster PUBLIC
                      magnetometer_broadcaster_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${sensor_msgs_TARGETS})

pluginlib_export_plugin_description_file(
  controller_interface magnetometer_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_magnetometer_broadcaster
    test/test_load_magnetometer_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/magnetometer_broadcaster_params.yaml)
  target_include_directories(test_load_magnetometer_broadcaster PRIVATE include)
  target_link_libraries(test_load_magnetometer_broadcaster
    magnetometer_broadcaster
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_magnetometer_broadcaster
    test/test_magnetometer_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/magnetometer_broadcaster_params.yaml)
  target_link_libraries(test_magnetometer_broadcaster
    magnetometer_broadcaster
    ros2_control_test_assets::ros2_control_test_assets
    magnetometer_broadcaster
  )
endif()

install(
  TARGETS
    magnetometer_broadcaster
    magnetometer_broadcaster_parameters
  EXPORT export_magnetometer_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_magnetometer_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
