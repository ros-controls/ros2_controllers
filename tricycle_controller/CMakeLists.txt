cmake_minimum_required(VERSION 3.16)
project(tricycle_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ackermann_msgs
  builtin_interfaces
  controller_interface
  geometry_msgs
  generate_parameter_library
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  std_srvs
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

generate_parameter_library(tricycle_controller_parameters
  src/tricycle_controller_parameter.yaml
)

add_library(tricycle_controller SHARED
  src/tricycle_controller.cpp
  src/odometry.cpp
  src/traction_limiter.cpp
  src/steering_limiter.cpp
)
target_compile_features(tricycle_controller PUBLIC cxx_std_17)
target_include_directories(tricycle_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/tricycle_controller>
)
target_link_libraries(tricycle_controller PUBLIC tricycle_controller_parameters)
ament_target_dependencies(tricycle_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface tricycle_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_tricycle_controller
    test/test_tricycle_controller.cpp)
  target_link_libraries(test_tricycle_controller
    tricycle_controller
  )
  ament_target_dependencies(test_tricycle_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_tricycle_controller test/test_load_tricycle_controller.cpp)
  target_link_libraries(test_load_tricycle_controller
    tricycle_controller
  )
  ament_target_dependencies(test_load_tricycle_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(test_traction_limiter
    test/test_traction_limiter.cpp)
  target_link_libraries(test_traction_limiter
    tricycle_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/tricycle_controller
)
install(
  TARGETS
    tricycle_controller
    tricycle_controller_parameters
  EXPORT export_tricycle_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_tricycle_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
