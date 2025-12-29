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
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(force_torque_sensor_broadcaster_parameters
  src/force_torque_sensor_broadcaster_parameters.yaml
)

generate_parameter_library(wrench_transformer_parameters
  src/wrench_transformer_parameters.yaml
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

# Wrench transformer library
add_library(wrench_transformer SHARED
  src/wrench_transformer.cpp
)
target_compile_features(wrench_transformer PUBLIC cxx_std_17)
target_include_directories(wrench_transformer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(wrench_transformer PUBLIC
  wrench_transformer_parameters
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
)

# Wrench transformer executable
add_executable(wrench_transformer_node
  src/wrench_transformer_main.cpp
)
target_compile_features(wrench_transformer_node PUBLIC cxx_std_17)
target_include_directories(wrench_transformer_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(wrench_transformer_node PUBLIC
  wrench_transformer
)

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
  )

  ament_add_gmock(test_wrench_transformer test/test_wrench_transformer.cpp)
  target_compile_features(test_wrench_transformer PUBLIC cxx_std_17)
  target_include_directories(test_wrench_transformer PRIVATE include)
  target_compile_definitions(test_wrench_transformer PRIVATE
    TEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_wrench_transformer
    wrench_transformer
    tf2_ros::tf2_ros
  )

  add_library(dummy_filter SHARED
    test/dummy_filter.cpp
  )
  target_compile_features(dummy_filter PUBLIC cxx_std_17)
  target_include_directories(dummy_filter PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
  )
  target_link_libraries(dummy_filter PUBLIC
    filters::increment
    ${geometry_msgs_TARGETS}
    pluginlib::pluginlib
    rclcpp::rclcpp
  )
  install(
    TARGETS
      dummy_filter
    EXPORT export_force_torque_sensor_broadcaster
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

  pluginlib_export_plugin_description_file(filters "test/dummy_filter_plugin.xml")
endif()

install(
  DIRECTORY include/
  DESTINATION include/force_torque_sensor_broadcaster
)
install(
  TARGETS
    force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster_parameters
    wrench_transformer
    wrench_transformer_parameters
  EXPORT export_force_torque_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executable to lib/<package_name> for launch files to find it
install(
  TARGETS
    wrench_transformer_node
  RUNTIME DESTINATION lib/force_torque_sensor_broadcaster
)

ament_export_targets(export_force_torque_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
