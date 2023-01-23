cmake_minimum_required(VERSION 3.5)
project(gripper_controllers)

if(APPLE OR WIN32)
  message(WARNING "gripper controllers are not available on OSX or Windows")
  return()
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    control_msgs
    control_toolbox
    controller_interface
    generate_parameter_library
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_action
    realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(gripper_action_controller_parameters
  src/gripper_action_controller_parameters.yaml
)

add_library(gripper_action_controller SHARED
  src/gripper_action_controller.cpp
)
ament_target_dependencies(gripper_action_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(gripper_action_controller
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<INSTALL_INTERFACE:include>
)
target_link_libraries(gripper_action_controller
  gripper_action_controller_parameters
)
pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_gripper_action_controllers
    test/test_load_gripper_action_controllers.cpp
  )
  ament_target_dependencies(test_load_gripper_action_controllers
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_gripper_controllers
    test/test_gripper_controllers.cpp
  )
  target_include_directories(test_gripper_controllers PRIVATE include)
  target_link_libraries(test_gripper_controllers
    gripper_action_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    gripper_action_controller
    gripper_action_controller_parameters
  EXPORT export_gripper_action_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_gripper_action_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
