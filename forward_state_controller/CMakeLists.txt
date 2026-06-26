cmake_minimum_required(VERSION 3.16)
project(forward_state_controller)

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
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  forward_state_controller_parameters
  src/forward_state_controller_parameters.yaml
)

add_library(forward_state_controller SHARED
  src/forward_state_controller.cpp
)
target_compile_features(forward_state_controller PUBLIC cxx_std_17)
target_include_directories(forward_state_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/forward_state_controller>
)
target_link_libraries(forward_state_controller PUBLIC
                      forward_state_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(controller_interface forward_state_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gmock(test_load_forward_state_controller
    test/test_load_forward_state_controller.cpp
  )
  target_link_libraries(test_load_forward_state_controller
    forward_state_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_forward_state_controller
    test/test_forward_state_controller.cpp
  )
  target_link_libraries(test_forward_state_controller
    forward_state_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/forward_state_controller
)
install(
  TARGETS
    forward_state_controller
    forward_state_controller_parameters
  EXPORT export_forward_state_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_forward_state_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
