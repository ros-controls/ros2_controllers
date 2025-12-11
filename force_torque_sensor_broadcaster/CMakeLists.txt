cmake_minimum_required(VERSION 3.16)
project(force_torque_sensor_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(force_torque_sensor_broadcaster_parameters
  src/force_torque_sensor_broadcaster_parameters.yaml
)

generate_parameter_library(wrench_transformer_parameters
  src/wrench_transformer_parameters.yaml
)

add_library(force_torque_sensor_broadcaster SHARED
  src/force_torque_sensor_broadcaster.cpp
)
target_compile_features(force_torque_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(force_torque_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(force_torque_sensor_broadcaster PUBLIC
  force_torque_sensor_broadcaster_parameters
)
ament_target_dependencies(force_torque_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(force_torque_sensor_broadcaster PRIVATE "FORCE_TORQUE_SENSOR_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface force_torque_sensor_broadcaster.xml)

# Wrench transformer library
add_library(wrench_transformer SHARED
  src/wrench_transformer.cpp
)
target_compile_features(wrench_transformer PUBLIC cxx_std_17)
target_include_directories(wrench_transformer PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(wrench_transformer PUBLIC
  wrench_transformer_parameters
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
)

# Wrench transformer executable
add_executable(wrench_transformer_node
  src/wrench_transformer_main.cpp
)
target_compile_features(wrench_transformer_node PUBLIC cxx_std_17)
target_include_directories(wrench_transformer_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/force_torque_sensor_broadcaster>
)
target_link_libraries(wrench_transformer_node PUBLIC
  wrench_transformer
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_rostest_with_parameters_gmock(test_load_force_torque_sensor_broadcaster
    test/test_load_force_torque_sensor_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/force_torque_sensor_broadcaster_params.yaml)
  target_include_directories(test_load_force_torque_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_load_force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster
  )
  ament_target_dependencies(test_load_force_torque_sensor_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_force_torque_sensor_broadcaster
    test/test_force_torque_sensor_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/force_torque_sensor_broadcaster_params.yaml)
  target_include_directories(test_force_torque_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster
  )
  ament_target_dependencies(test_force_torque_sensor_broadcaster
    hardware_interface
  )

  ament_add_gmock(test_wrench_transformer test/test_wrench_transformer.cpp)
  target_compile_features(test_wrench_transformer PUBLIC cxx_std_17)
  target_include_directories(test_wrench_transformer PRIVATE include)
  target_compile_definitions(test_wrench_transformer PRIVATE
    TEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_wrench_transformer
    wrench_transformer
    tf2_ros::tf2_ros
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/force_torque_sensor_broadcaster
)
install(
  TARGETS
    force_torque_sensor_broadcaster
    force_torque_sensor_broadcaster_parameters
    wrench_transformer
    wrench_transformer_parameters
  EXPORT export_force_torque_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executable to lib/<package_name> for launch files to find it
install(
  TARGETS
    wrench_transformer_node
  RUNTIME DESTINATION lib/force_torque_sensor_broadcaster
)

ament_export_targets(export_force_torque_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
