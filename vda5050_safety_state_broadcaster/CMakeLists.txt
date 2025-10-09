cmake_minimum_required(VERSION 3.8)
project(vda5050_safety_state_broadcaster)

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
  urdf
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(vda5050_safety_state_broadcaster_parameters
  src/vda5050_safety_state_broadcaster.yaml
)

add_library(vda5050_safety_state_broadcaster SHARED
  src/vda5050_safety_state_broadcaster.cpp
)

target_compile_features(vda5050_safety_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(vda5050_safety_state_broadcaster
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/vda5050_safety_state_broadcaster>
)
target_link_libraries(vda5050_safety_state_broadcaster PUBLIC
                      vda5050_safety_state_broadcaster_parameters
                      controller_interface::controller_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})


pluginlib_export_plugin_description_file(controller_interface vda5050_safety_state_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_vda5050_safety_state_broadcaster test/test_load_vda5050_safety_state_broadcaster.cpp)
  target_include_directories(test_load_vda5050_safety_state_broadcaster PRIVATE include)
  target_link_libraries(test_load_vda5050_safety_state_broadcaster
    vda5050_safety_state_broadcaster
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_vda5050_safety_state_broadcaster
    test/test_vda5050_safety_state_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/vda5050_safety_state_broadcaster_params.yaml)
  target_include_directories(test_vda5050_safety_state_broadcaster PRIVATE include)
  target_link_libraries(test_vda5050_safety_state_broadcaster
    vda5050_safety_state_broadcaster
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/vda5050_safety_state_broadcaster
)
install(
  TARGETS
    vda5050_safety_state_broadcaster
    vda5050_safety_state_broadcaster_parameters
  EXPORT export_vda5050_safety_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_vda5050_safety_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
