cmake_minimum_required(VERSION 3.16)
project(state_interfaces_broadcaster)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(state_interfaces_broadcaster_parameters
  src/state_interfaces_broadcaster_parameters.yaml
)

add_library(state_interfaces_broadcaster SHARED
  src/state_interfaces_broadcaster.cpp
)
target_compile_features(state_interfaces_broadcaster PUBLIC cxx_std_17)
target_include_directories(state_interfaces_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/state_interfaces_broadcaster>
)
target_link_libraries(state_interfaces_broadcaster PUBLIC
                      state_interfaces_broadcaster_parameters
                      controller_interface::controller_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface state_interfaces_broadcaster_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_state_interfaces_broadcaster
    test/test_load_state_interfaces_broadcaster.cpp
  )
  target_link_libraries(test_load_state_interfaces_broadcaster
    state_interfaces_broadcaster
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    rclcpp::rclcpp
    ros2_control_test_assets::ros2_control_test_assets)
  target_compile_definitions(
    test_load_state_interfaces_broadcaster
    PRIVATE PARAMETERS_FILE_PATH="${CMAKE_CURRENT_LIST_DIR}/test/test_state_interfaces_broadcaster_params.yaml")

  ament_add_gmock(test_state_interfaces_broadcaster
    test/test_state_interfaces_broadcaster.cpp
  )
  target_link_libraries(test_state_interfaces_broadcaster
    state_interfaces_broadcaster
    ros2_control_test_assets::ros2_control_test_assets
    rclcpp::rclcpp
    ${control_msgs_TARGETS})
endif()

install(
  DIRECTORY include/
  DESTINATION include/state_interfaces_broadcaster
)
install(
  TARGETS
    state_interfaces_broadcaster
    state_interfaces_broadcaster_parameters
  EXPORT export_state_interfaces_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_state_interfaces_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
