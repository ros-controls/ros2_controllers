cmake_minimum_required(VERSION 3.16)
project(gpio_controllers)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(gpio_command_controller_parameters
  src/gpio_command_controller_parameters.yaml
)
add_library(gpio_controllers SHARED
  src/gpio_command_controller.cpp
)
target_include_directories(gpio_controllers PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(gpio_controllers PUBLIC
                      gpio_command_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${control_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})

generate_parameter_library(gpio_tool_controller_parameters
  src/gpio_tool_controller.yaml
)
add_library(gpio_tool_controllers SHARED
  src/gpio_tool_controller.cpp
)
target_include_directories(gpio_tool_controllers PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(gpio_tool_controllers PUBLIC
  gpio_tool_controller_parameters
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${control_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${std_srvs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface gpio_controllers_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_gpio_command_controller
    test/test_load_gpio_command_controller.cpp
  )

  target_link_libraries(test_load_gpio_command_controller
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(
    test_gpio_command_controller
    test/test_gpio_command_controller.cpp
  )
  target_include_directories(test_gpio_command_controller PRIVATE include)
  target_link_libraries(test_gpio_command_controller
    gpio_controllers
    ros2_control_test_assets::ros2_control_test_assets
  )
  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(
    test_load_gpio_tool_controller
    test/test_load_gpio_tool_controller.cpp
  )
  target_include_directories(test_load_gpio_tool_controller PRIVATE include)
  target_link_libraries(test_load_gpio_tool_controller
    gpio_tool_controllers
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_gpio_tool_controller
    test/gpio_tool_controller/test_gpio_tool_controller.cpp
  #   test/gpio_tool_controller/test_gpio_tool_controller_open.cpp
  #   test/gpio_tool_controller/test_gpio_tool_controller_close.cpp
    test/gpio_tool_controller/test_gpio_tool_controller_all_param_set.cpp
  #   test/gpio_tool_controller/test_gpio_tool_controller_open_close_action.cpp
  #   test/gpio_tool_controller/test_gpio_tool_controller_reconfigure.cpp
  #   test/gpio_tool_controller/test_gpio_tool_controller_reconfigure_action.cpp
  )
  target_include_directories(test_gpio_tool_controller PRIVATE include)
  target_link_libraries(test_gpio_tool_controller
    gpio_tool_controllers
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/gpio_controllers
)

install(
  TARGETS
  gpio_controllers
  gpio_command_controller_parameters
  gpio_tool_controllers
  gpio_tool_controller_parameters
  EXPORT export_gpio_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_gpio_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
