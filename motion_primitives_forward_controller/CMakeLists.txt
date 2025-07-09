cmake_minimum_required(VERSION 3.8)
project(motion_primitives_forward_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

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
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(motion_primitives_forward_controller_parameters
  src/motion_primitives_forward_controller.yaml
)
add_library(
  motion_primitives_forward_controller
  SHARED
  src/motion_primitives_forward_controller.cpp
)
add_library(
  motion_primitives_from_trajectory_controller
  SHARED
  src/motion_primitives_from_trajectory_controller.cpp
  src/fk_client.cpp
  src/rdp.cpp
  src/approx_primitives_with_rdp.cpp
)
target_include_directories(motion_primitives_forward_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(motion_primitives_from_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(motion_primitives_forward_controller PUBLIC
  motion_primitives_forward_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ${std_srvs_TARGETS}
)
target_link_libraries(motion_primitives_from_trajectory_controller PUBLIC
  motion_primitives_forward_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_compile_definitions(motion_primitives_forward_controller PRIVATE "MOTION_PRIMITIVES_FORWARD_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface motion_primitives_forward_controller.xml)

pluginlib_export_plugin_description_file(
  controller_interface motion_primitives_from_trajectory_controller.xml)

install(
  TARGETS
    motion_primitives_forward_controller
    motion_primitives_from_trajectory_controller
    motion_primitives_forward_controller_parameters
  EXPORT motion_primitives_forward_controller_targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  motion_primitives_forward_controller
  motion_primitives_from_trajectory_controller
)

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
    motion_primitives_forward_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_motion_primitives_forward_controller
    test/test_motion_primitives_forward_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_primitives_forward_controller_params.yaml
  )
  target_link_libraries(test_motion_primitives_forward_controller
    motion_primitives_forward_controller
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_motion_primitives_forward_controller_preceeding
    test/test_motion_primitives_forward_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_primitives_forward_controller_preceeding_params.yaml
  )
  target_link_libraries(test_motion_primitives_forward_controller_preceeding
    motion_primitives_forward_controller
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()
