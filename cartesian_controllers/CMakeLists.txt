cmake_minimum_required(VERSION 3.8)
project(cartesian_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(twist_controller src/twist_controller.cpp)
target_include_directories(twist_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  twist_controller
  "controller_interface"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
  "realtime_tools"
  "geometry_msgs"
  "std_msgs"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(twist_controller PRIVATE "CARTESIAN_CONTROLLERS_BUILDING_LIBRARY")
pluginlib_export_plugin_description_file(controller_interface twist_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS twist_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_twist_controller test/test_load_twist_controller.cpp)
  target_include_directories(test_load_twist_controller PRIVATE include)
  ament_target_dependencies(
          test_load_twist_controller
          controller_manager
          hardware_interface
          ros2_control_test_assets
  )

  ament_add_gmock(test_twist_controller test/test_twist_controller.cpp)
  target_include_directories(test_twist_controller PRIVATE include)
  target_link_libraries(test_twist_controller twist_controller)
  ament_target_dependencies(
          test_twist_controller
          controller_interface
          hardware_interface
  )
endif()

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  geometry_msgs
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  twist_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
