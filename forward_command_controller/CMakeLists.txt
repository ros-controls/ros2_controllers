cmake_minimum_required(VERSION 3.16)
project(forward_command_controller)

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
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  forward_command_controller_parameters
  src/forward_command_controller_parameters.yaml
)
generate_parameter_library(
  multi_interface_forward_command_controller_parameters
  src/multi_interface_forward_command_controller_parameters.yaml
)

add_library(forward_command_controller SHARED
  src/forward_controllers_base.cpp
  src/forward_command_controller.cpp
  src/multi_interface_forward_command_controller.cpp
)
target_compile_features(forward_command_controller PUBLIC cxx_std_17)
target_include_directories(forward_command_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/forward_command_controller>
)
target_link_libraries(forward_command_controller PUBLIC
                      forward_command_controller_parameters
                      multi_interface_forward_command_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${std_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface forward_command_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_forward_command_controller
    test/test_load_forward_command_controller.cpp
  )
  target_link_libraries(test_load_forward_command_controller
    forward_command_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_forward_command_controller
    test/test_forward_command_controller.cpp
  )
  target_link_libraries(test_forward_command_controller
    forward_command_controller
  )

  ament_add_gmock(test_load_multi_interface_forward_command_controller
    test/test_load_multi_interface_forward_command_controller.cpp
  )
  target_link_libraries(test_load_multi_interface_forward_command_controller
    forward_command_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_multi_interface_forward_command_controller
    test/test_multi_interface_forward_command_controller.cpp
  )
  target_link_libraries(test_multi_interface_forward_command_controller
    forward_command_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/forward_command_controller
)
install(
  TARGETS
    forward_command_controller
    forward_command_controller_parameters
    multi_interface_forward_command_controller_parameters
  EXPORT export_forward_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_forward_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
