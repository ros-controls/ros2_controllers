cmake_minimum_required(VERSION 3.16)
project(gripper_controllers LANGUAGES CXX)

if(APPLE OR WIN32)
  message(WARNING "gripper controllers are not available on OSX or Windows")
  return()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
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
target_compile_features(gripper_action_controller PUBLIC cxx_std_17)
target_include_directories(gripper_action_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gripper_action_controller>
)
target_link_libraries(gripper_action_controller PUBLIC
  gripper_action_controller_parameters
)
ament_target_dependencies(gripper_action_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gripper_action_controller PRIVATE "GRIPPER_ACTION_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_gripper_action_controllers
    test/test_load_gripper_action_controllers.cpp
  )
  ament_target_dependencies(test_load_gripper_action_controllers
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(test_gripper_controllers
    test/test_gripper_controllers.cpp
  )
  target_link_libraries(test_gripper_controllers
    gripper_action_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/gripper_action_controller
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
