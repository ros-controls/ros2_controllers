cmake_minimum_required(VERSION 3.16)
project(generic_steering_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# --- Dependencies ---
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  control_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  tf2_geometry_msgs
  std_msgs
  lifecycle_msgs
  generate_parameter_library
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# --- Parameter Library ---
generate_parameter_library(generic_steering_controller_parameters
  config/generic_steering_controller_parameters.yaml
)

# --- Main Controller Library ---
add_library(generic_steering_controller SHARED
  src/generic_steering_controller.cpp
)
target_compile_features(generic_steering_controller PUBLIC cxx_std_17)
target_include_directories(generic_steering_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/generic_steering_controller>"
)
target_link_libraries(generic_steering_controller PUBLIC
  generic_steering_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
)

# --- Plugin Export for the REAL Controller ---
pluginlib_export_plugin_description_file(controller_interface generic_steering_controller.xml)

# --- Testing Block ---
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Build the Mock Plugin as a separate SHARED library
  add_library(mock_kinematic_plugin SHARED
    test/mock_kinematic_plugin/mock_kinematic_plugin.cpp
  )
  target_include_directories(mock_kinematic_plugin PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test/mock_kinematic_plugin>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(mock_kinematic_plugin PUBLIC
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${nav_msgs_TARGETS}
    ${control_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
  )

  # Build the Test Executable
  ament_add_gmock(
    test_generic_steering_controller
    test/test_generic_steering_controller.cpp
    test/test_generic_steering_controller.hpp
  )
  target_include_directories(test_generic_steering_controller PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/test>
  )
  target_link_libraries(test_generic_steering_controller
    generic_steering_controller
    generic_steering_controller_parameters
  )

  # Install the mock plugin so it can be found by pluginlib
  install(TARGETS mock_kinematic_plugin
    LIBRARY DESTINATION lib
  )

  # Export the mock plugin description
  pluginlib_export_plugin_description_file(
    generic_steering_controller
    test/mock_kinematic_plugin/mock_kinematic_plugin.xml
  )
endif()
# --- Install Rules ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    generic_steering_controller
    generic_steering_controller_parameters
  EXPORT export_generic_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install the REAL controller's plugin description
install(
  FILES generic_steering_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- Ament Export ---
ament_export_include_directories(include)
ament_export_targets(export_generic_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
