cmake_minimum_required(VERSION 3.5)
project(forward_command_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  forward_command_controller_parameters
  src/forward_command_controller_parameters.yaml
)
generate_parameter_library(
  multi_interface_forward_command_controller_parameters
  src/multi_interface_forward_command_controller_parameters.yaml
)

add_library(forward_command_controller
  SHARED
  src/forward_controllers_base.cpp
  src/forward_command_controller.cpp
  src/multi_interface_forward_command_controller.cpp
)
target_include_directories(forward_command_controller PRIVATE include)
ament_target_dependencies(forward_command_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(forward_command_controller
  forward_command_controller_parameters
  multi_interface_forward_command_controller_parameters
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(forward_command_controller PRIVATE "FORWARD_COMMAND_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface forward_command_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    forward_command_controller
    forward_command_controller_parameters
    multi_interface_forward_command_controller_parameters
  EXPORT export_forward_command_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_forward_command_controller
    test/test_load_forward_command_controller.cpp
  )
  target_include_directories(test_load_forward_command_controller PRIVATE include)
  ament_target_dependencies(test_load_forward_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_forward_command_controller
    test/test_forward_command_controller.cpp
  )
  target_include_directories(test_forward_command_controller PRIVATE include)
  target_link_libraries(test_forward_command_controller
    forward_command_controller
  )

  ament_add_gmock(
    test_load_multi_interface_forward_command_controller
    test/test_load_multi_interface_forward_command_controller.cpp
  )
  target_include_directories(test_load_multi_interface_forward_command_controller PRIVATE include)
  ament_target_dependencies(
    test_load_multi_interface_forward_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_multi_interface_forward_command_controller
    test/test_multi_interface_forward_command_controller.cpp
  )
  target_include_directories(test_multi_interface_forward_command_controller PRIVATE include)
  target_link_libraries(test_multi_interface_forward_command_controller
    forward_command_controller
  )
endif()

ament_export_targets(export_forward_command_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
