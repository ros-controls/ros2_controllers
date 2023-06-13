cmake_minimum_required(VERSION 3.16)
project(ackermann_steering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  steering_controllers_library
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(ackermann_steering_controller_parameters
  src/ackermann_steering_controller.yaml
)

add_library(
  ackermann_steering_controller
  SHARED
  src/ackermann_steering_controller.cpp
)
target_compile_features(ackermann_steering_controller PUBLIC cxx_std_17)
target_include_directories(ackermann_steering_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ackermann_steering_controller PUBLIC
  ackermann_steering_controller_parameters)
ament_target_dependencies(ackermann_steering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ackermann_steering_controller PRIVATE "ACKERMANN_STEERING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface ackermann_steering_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)


  add_rostest_with_parameters_gmock(test_load_ackermann_steering_controller
    test/test_load_ackermann_steering_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ackermann_steering_controller_params.yaml
  )
  ament_target_dependencies(test_load_ackermann_steering_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_ackermann_steering_controller
    test/test_ackermann_steering_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ackermann_steering_controller_params.yaml
  )
  target_include_directories(test_ackermann_steering_controller PRIVATE include)
  target_link_libraries(test_ackermann_steering_controller ackermann_steering_controller)
  ament_target_dependencies(
    test_ackermann_steering_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_ackermann_steering_controller_preceeding test/test_ackermann_steering_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/ackermann_steering_controller_preceeding_params.yaml)
  target_include_directories(test_ackermann_steering_controller_preceeding PRIVATE include)
  target_link_libraries(test_ackermann_steering_controller_preceeding ackermann_steering_controller)
  ament_target_dependencies(
    test_ackermann_steering_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/ackermann_steering_controller
)

install(
  TARGETS ackermann_steering_controller ackermann_steering_controller_parameters
  EXPORT export_ackermann_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_ackermann_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
