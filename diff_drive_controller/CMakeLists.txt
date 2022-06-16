cmake_minimum_required(VERSION 3.5)
project(diff_drive_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
    rcpputils
    realtime_tools
    tf2
    tf2_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/diff_drive_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFF_DRIVE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface diff_drive_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_diff_drive_controller
    test/test_diff_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_diff_drive_controller.yaml)
  target_include_directories(test_diff_drive_controller PRIVATE include)
  target_link_libraries(test_diff_drive_controller
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_diff_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(
    test_load_diff_drive_controller
    test/test_load_diff_drive_controller.cpp
  )
  target_include_directories(test_load_diff_drive_controller PRIVATE include)
  ament_target_dependencies(test_load_diff_drive_controller
    controller_manager
    ros2_control_test_assets
  )

endif()

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
