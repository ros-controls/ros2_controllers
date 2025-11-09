cmake_minimum_required(VERSION 3.16)
project(generic_state_broadcaster)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
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

generate_parameter_library(generic_state_broadcaster_parameters
  src/generic_state_broadcaster_parameters.yaml
)

add_library(generic_state_broadcaster SHARED
  src/generic_state_broadcaster.cpp
)
target_compile_features(generic_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(generic_state_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/generic_state_broadcaster>
)
target_link_libraries(generic_state_broadcaster PUBLIC
                      generic_state_broadcaster_parameters
                      controller_interface::controller_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})
pluginlib_export_plugin_description_file(controller_interface generic_state_broadcaster_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # ament_add_gmock(test_load_generic_state_broadcaster
  #   test/test_load_generic_state_broadcaster.cpp
  # )
  # target_link_libraries(test_load_generic_state_broadcaster
  #   generic_state_broadcaster
  #   controller_manager::controller_manager
  #   hardware_interface::hardware_interface
  #   rclcpp::rclcpp
  #   ros2_control_test_assets::ros2_control_test_assets)

  # ament_add_gmock(test_generic_state_broadcaster
  #   test/test_generic_state_broadcaster.cpp
  # )
  # target_link_libraries(test_generic_state_broadcaster
  #   generic_state_broadcaster
  #   ros2_control_test_assets::ros2_control_test_assets)
endif()

install(
  DIRECTORY include/
  DESTINATION include/generic_state_broadcaster
)
install(
  TARGETS
    generic_state_broadcaster
    generic_state_broadcaster_parameters
  EXPORT export_generic_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_generic_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
