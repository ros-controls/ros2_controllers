cmake_minimum_required(VERSION 3.8)
project(tricycle_steering_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  steering_controllers_library
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add tricycle_steering_controller library related compile commands
generate_parameter_library(tricycle_steering_controller_parameters
  src/tricycle_steering_controller.yaml
)
add_library(
  tricycle_steering_controller
  SHARED
  src/tricycle_steering_controller.cpp
)
target_include_directories(tricycle_steering_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tricycle_steering_controller tricycle_steering_controller_parameters)
ament_target_dependencies(tricycle_steering_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(tricycle_steering_controller PRIVATE "TRICYCLE_STEERING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface tricycle_steering_controller.xml)

install(
  TARGETS
  tricycle_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_tricycle_steering_controller test/test_load_tricycle_steering_controller.cpp)
  target_include_directories(test_load_tricycle_steering_controller PRIVATE include)
  ament_target_dependencies(
    test_load_tricycle_steering_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_tricycle_steering_controller test/test_tricycle_steering_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/tricycle_steering_controller_params.yaml)
  target_include_directories(test_tricycle_steering_controller PRIVATE include)
  target_link_libraries(test_tricycle_steering_controller tricycle_steering_controller)
  ament_target_dependencies(
    test_tricycle_steering_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_tricycle_steering_controller_preceeding test/test_tricycle_steering_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/tricycle_steering_controller_preceeding_params.yaml)
  target_include_directories(test_tricycle_steering_controller_preceeding PRIVATE include)
  target_link_libraries(test_tricycle_steering_controller_preceeding tricycle_steering_controller)
  ament_target_dependencies(
    test_tricycle_steering_controller_preceeding
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
  tricycle_steering_controller
)

ament_package()
