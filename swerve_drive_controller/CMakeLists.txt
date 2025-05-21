cmake_minimum_required(VERSION 3.8)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

pluginlib_export_plugin_description_file(controller_interface swerve_drive_plugin.xml)

add_library(swerve_drive_controller SHARED
  src/swerve_drive_controller.cpp
  src/swerve_drive_kinematics.cpp
)

target_include_directories(swerve_drive_controller PRIVATE include)
ament_target_dependencies(swerve_drive_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS swerve_drive_controller
  EXPORT export_swerve_drive_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES swerve_drive_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)

  find_package(controller_manager REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gtest(test_swerve_drive_controller
    test/test_swerve_drive_controller.cpp
  )

  target_include_directories(test_swerve_drive_controller PRIVATE include)
  ament_target_dependencies(test_swerve_drive_controller
    controller_interface
    hardware_interface
    rclcpp
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_swerve_drive_controller
    ${PROJECT_NAME}
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_swerve_drive_controller test/test_load_swerve_drive_controller.cpp)
  ament_target_dependencies(test_load_swerve_drive_controller
    controller_manager
    ros2_control_test_assets
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(swerve_drive_controller)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle)


ament_package()
