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
  industrial_robot_motion_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(motion_primitives_forward_controller_parameters
  src/motion_primitives_forward_controller.yaml
  include/motion_primitives_forward_controller/validate_motion_primitives_forward_controller_parameters.hpp
)
add_library(
  motion_primitives_forward_controller
  SHARED
  src/motion_primitives_forward_controller.cpp
)
target_include_directories(motion_primitives_forward_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(motion_primitives_forward_controller motion_primitives_forward_controller_parameters)
ament_target_dependencies(motion_primitives_forward_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_compile_definitions(motion_primitives_forward_controller PRIVATE "MOTION_PRIMITIVES_FORWARD_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface motion_primitives_forward_controller.xml)

install(
  TARGETS
    motion_primitives_forward_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_motion_primitives_forward_controller
    test/test_load_motion_primitives_forward_controller.cpp
  )
  target_include_directories(
    test_load_motion_primitives_forward_controller PRIVATE include
  )
  ament_target_dependencies(
    test_load_motion_primitives_forward_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_motion_primitives_forward_controller
    test/test_motion_primitives_forward_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_primitives_forward_controller_params.yaml
  )
  target_include_directories(
    test_motion_primitives_forward_controller PRIVATE include
  )
  target_link_libraries(
    test_motion_primitives_forward_controller motion_primitives_forward_controller
  )
  ament_target_dependencies(
    test_motion_primitives_forward_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_motion_primitives_forward_controller_preceeding
    test/test_motion_primitives_forward_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/motion_primitives_forward_controller_preceeding_params.yaml
  )
  target_include_directories(
    test_motion_primitives_forward_controller_preceeding PRIVATE include
  )
  target_link_libraries(
    test_motion_primitives_forward_controller_preceeding motion_primitives_forward_controller
  )
  ament_target_dependencies(
    test_motion_primitives_forward_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  motion_primitives_forward_controller
)

ament_package()
