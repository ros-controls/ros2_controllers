cmake_minimum_required(VERSION 3.8)
project(gpio_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
  -Werror=return-type -Werror=format -Werror=missing-braces)
endif()

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
target_link_libraries(gpio_controllers PUBLIC gpio_command_controller_parameters)
ament_target_dependencies(gpio_controllers PUBLIC
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  control_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gpio_controllers PRIVATE "GPIO_COMMAND_CONTROLLER_BUILDING_DLL")
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

  target_include_directories(test_load_gpio_command_controller PRIVATE include)
  ament_target_dependencies(test_load_gpio_command_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_gpio_command_controller
    test/test_gpio_command_controller.cpp
  )
  target_include_directories(test_gpio_command_controller PRIVATE include)
  target_link_libraries(test_gpio_command_controller
    gpio_controllers
  )
  ament_target_dependencies(test_gpio_command_controller
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    ros2_control_test_assets
    control_msgs
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
