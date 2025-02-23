cmake_minimum_required(VERSION 3.16)
project(gps_sensor_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                    -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                    -Werror=missing-braces)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


generate_parameter_library(gps_sensor_broadcaster_parameters
  src/gps_sensor_broadcaster_parameters.yaml
)

add_library(gps_sensor_broadcaster SHARED
  src/gps_sensor_broadcaster.cpp
)

target_compile_features(gps_sensor_broadcaster PUBLIC cxx_std_17)
target_include_directories(gps_sensor_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gps_sensor_broadcaster>
)

target_link_libraries(gps_sensor_broadcaster PUBLIC gps_sensor_broadcaster_parameters)
ament_target_dependencies(gps_sensor_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface gps_sensor_broadcaster.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_gps_sensor_broadcaster
    test/test_load_gps_sensor_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/gps_sensor_broadcaster_params.yaml)
  target_include_directories(test_load_gps_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_load_gps_sensor_broadcaster
    gps_sensor_broadcaster
  )
  ament_target_dependencies(test_load_gps_sensor_broadcaster
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_gps_sensor_broadcaster
    test/test_gps_sensor_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/gps_sensor_broadcaster_params.yaml)
  target_include_directories(test_gps_sensor_broadcaster PRIVATE include)
  target_link_libraries(test_gps_sensor_broadcaster
    gps_sensor_broadcaster
  )
  ament_target_dependencies(test_gps_sensor_broadcaster
    hardware_interface
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/gps_sensor_broadcaster
)
install(
  TARGETS
    gps_sensor_broadcaster
    gps_sensor_broadcaster_parameters
  EXPORT export_gps_sensor_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_gps_sensor_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
