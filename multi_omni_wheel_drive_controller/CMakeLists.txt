cmake_minimum_required(VERSION 3.16)
project(multi_omni_wheel_drive_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
  Eigen3
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/${PROJECT_NAME}_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/odometry.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_parameters
  Eigen3::Eigen
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2::tf2
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
)
pluginlib_export_plugin_description_file(controller_interface multi_omni_wheel_drive_plugin.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}/
)
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_${PROJECT_NAME} test/test_load_${PROJECT_NAME}.cpp)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  target_link_libraries(test_load_${PROJECT_NAME}
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
