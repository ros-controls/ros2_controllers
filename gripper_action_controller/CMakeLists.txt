cmake_minimum_required(VERSION 3.5)
project(gripper_action_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

add_library(gripper_action_controller SHARED
   src/gripper_action_controller.cpp
)
ament_target_dependencies(gripper_action_controller
  controller_interface
  control_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcutils
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)
#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include
)

# Install targets
install(TARGETS gripper_action_controller
   LIBRARY DESTINATION lib
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
