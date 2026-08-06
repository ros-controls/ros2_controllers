cmake_minimum_required(VERSION 3.16)
project(gpio_controllers)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(control_msgs REQUIRED)

generate_parameter_library(gpio_command_controller_parameters
  src/gpio_command_controller_parameters.yaml
)

add_library(gpio_controllers
  SHARED
  src/gpio_command_controller.cpp
)
target_include_directories(gpio_controllers PRIVATE include)
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

pluginlib_export_plugin_description_file(controller_interface gpio_controllers_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  gpio_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

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
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  gpio_controllers
)
ament_package()
