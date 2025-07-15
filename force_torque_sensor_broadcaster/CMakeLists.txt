cmake_minimum_required(VERSION 3.16)
project(force_torque_sensor_broadcaster)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  filters
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(force_torque_sensor_broadcaster_parameters
  src/force_torque_sensor_broadcaster_parameters.yaml
)

add_library(force_torque_sensor_broadcaster SHARED
  src/force_torque_sensor_broadcaster.cpp
)
target_compile_features(force_torque_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(force_torque_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(force_torque_sensor_broadcaster PUBLIC
                      force_torque_sensor_broadcaster_parameters
                      controller_interface::controller_interface
                      filters::filter_chain
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${geometry_msgs_TARGETS})

pluginlib_export_plugin_description_file(
  controller_interface force_torque_sensor_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_force_torque_sensor_broadcaster test/test_load_force_torque_sensor_broadcaster.cpp)
  target_include_directories(test_load_force_torque_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_load_force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_force_torque_sensor_broadcaster
    test/test_force_torque_sensor_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/force_torque_sensor_broadcaster_params.yaml)
  target_include_directories(test_force_torque_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster_parameters
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/force_torque_sensor_broadcaster
)
install(
  TARGETS
    force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster_parameters
  EXPORT export_force_torque_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_force_torque_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
