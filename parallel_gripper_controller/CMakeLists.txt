cmake_minimum_required(VERSION 3.16)
project(parallel_gripper_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(parallel_gripper_action_controller_parameters
  src/gripper_action_controller_parameters.yaml
)

add_library(parallel_gripper_action_controller SHARED
        src/parallel_gripper_action_controller.cpp
)
target_compile_features(parallel_gripper_action_controller PUBLIC cxx_std_17)
target_include_directories(parallel_gripper_action_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/parallel_gripper_action_controller>
)
target_link_libraries(parallel_gripper_action_controller PUBLIC
                      parallel_gripper_action_controller_parameters
                      control_toolbox::control_toolbox
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_action::rclcpp_action
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS})

pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_parallel_gripper_action_controllers test/test_load_parallel_gripper_action_controller.cpp)
  target_include_directories(test_load_parallel_gripper_action_controllers PRIVATE include)
  target_link_libraries(test_load_parallel_gripper_action_controllers
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_parallel_gripper_controller
  test/test_parallel_gripper_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/gripper_action_controller_params.yaml)
  target_include_directories(test_parallel_gripper_controller PRIVATE include)
  target_link_libraries(test_parallel_gripper_controller
          parallel_gripper_action_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/parallel_gripper_action_controller
)
install(
  TARGETS
        parallel_gripper_action_controller
        parallel_gripper_action_controller_parameters
  EXPORT export_parallel_gripper_action_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_parallel_gripper_action_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
