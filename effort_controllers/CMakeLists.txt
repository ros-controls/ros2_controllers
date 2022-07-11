cmake_minimum_required(VERSION 3.5)
project(effort_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    forward_command_controller
    pluginlib
    rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  SHARED
  src/joint_group_effort_controller.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "EFFORT_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface effort_controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_joint_group_effort_controller
    test/test_load_joint_group_effort_controller.cpp
  )
  target_include_directories(test_load_joint_group_effort_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_group_effort_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(
    test_joint_group_effort_controller
    test/test_joint_group_effort_controller.cpp
  )
  target_include_directories(test_joint_group_effort_controller PRIVATE include)
  target_link_libraries(test_joint_group_effort_controller
    ${PROJECT_NAME}
  )
endif()

ament_export_dependencies(
  forward_command_controller
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
