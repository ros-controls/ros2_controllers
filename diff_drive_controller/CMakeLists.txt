cmake_minimum_required(VERSION 3.16)
project(diff_drive_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_toolbox
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

# get include dirs from control_toolbox for the custom validators
get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(diff_drive_controller_parameters
  src/diff_drive_controller_parameter.yaml
  ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
)

add_library(diff_drive_controller SHARED
  src/diff_drive_controller.cpp
  src/odometry.cpp
)
target_compile_features(diff_drive_controller PUBLIC cxx_std_17)
target_include_directories(diff_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/diff_drive_controller>
)
target_link_libraries(diff_drive_controller
  PUBLIC
    diff_drive_controller_parameters
    control_toolbox::rate_limiter_parameters
    control_toolbox::control_toolbox
    controller_interface::controller_interface
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    rcpputils::rcpputils
    realtime_tools::realtime_tools
    tf2::tf2
    ${tf2_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface diff_drive_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_diff_drive_controller
    test/test_diff_drive_controller.cpp
  )
  target_link_libraries(test_diff_drive_controller
    diff_drive_controller
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_diff_drive_controller test/test_load_diff_drive_controller.cpp)
  target_link_libraries(test_load_diff_drive_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_odometry
    test/test_odometry.cpp
  )
  target_link_libraries(test_odometry
    diff_drive_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/diff_drive_controller
)
install(TARGETS diff_drive_controller diff_drive_controller_parameters
  EXPORT export_diff_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_diff_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
