cmake_minimum_required(VERSION 3.16)
project(joint_state_broadcaster)

include(../ros2_controllers.cmake)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  sensor_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_state_broadcaster_parameters
  src/joint_state_broadcaster_parameters.yaml
)

add_library(joint_state_broadcaster SHARED
  src/joint_state_broadcaster.cpp
)
target_compile_features(joint_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(joint_state_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_state_broadcaster>
)
target_link_libraries(joint_state_broadcaster PUBLIC
  joint_state_broadcaster_parameters
)
ament_target_dependencies(joint_state_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_joint_state_broadcaster
    test/test_load_joint_state_broadcaster.cpp
  )
  target_link_libraries(test_load_joint_state_broadcaster
    joint_state_broadcaster
  )
  ament_target_dependencies(test_load_joint_state_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_joint_state_broadcaster
    test/test_joint_state_broadcaster.cpp
  )
  target_link_libraries(test_joint_state_broadcaster
    joint_state_broadcaster
  )
  ament_target_dependencies(test_joint_state_broadcaster
    hardware_interface
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/joint_state_broadcaster
)
install(
  TARGETS
    joint_state_broadcaster
    joint_state_broadcaster_parameters
  EXPORT export_joint_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
