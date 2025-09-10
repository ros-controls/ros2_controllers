cmake_minimum_required(VERSION 3.10)
project(motion_primitives_controllers)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(motion_primitives_forward_controller_parameters
  src/motion_primitives_forward_controller_parameter.yaml
)
generate_parameter_library(motion_primitives_from_trajectory_controller_parameters
  src/motion_primitives_from_trajectory_controller_parameter.yaml
)
add_library(
  ${PROJECT_NAME}
  SHARED
  src/motion_primitives_base_controller.cpp
  src/motion_primitives_forward_controller.cpp
  src/motion_primitives_from_trajectory_controller.cpp
  src/fk_client.cpp
  src/rdp.cpp
  src/approx_primitives_with_rdp.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  motion_primitives_forward_controller_parameters
  motion_primitives_from_trajectory_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface controller_plugins.xml)

install(
  TARGETS motion_primitives_forward_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_motion_primitives_forward_controller
    test/test_load_motion_primitives_forward_controller.cpp
  )
  target_link_libraries(test_load_motion_primitives_forward_controller
    ${PROJECT_NAME}
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_motion_primitives_forward_controller
    test/test_motion_primitives_forward_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_primitives_forward_controller_params.yaml
  )
  target_link_libraries(test_motion_primitives_forward_controller
    ${PROJECT_NAME}
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()
