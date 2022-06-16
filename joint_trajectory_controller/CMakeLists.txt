cmake_minimum_required(VERSION 3.5)
project(joint_trajectory_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    angles
    controller_interface
    control_msgs
    control_toolbox
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    trajectory_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/joint_trajectory_controller.cpp
  src/trajectory.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_TRAJECTORY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_include_directories(test_trajectory PRIVATE include)
  target_link_libraries(test_trajectory ${PROJECT_NAME})

  ament_add_gtest(test_trajectory_controller
    test/test_trajectory_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_joint_trajectory_controller.yaml)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_include_directories(test_trajectory_controller PRIVATE include)
  target_link_libraries(test_trajectory_controller
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_trajectory_controller
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  ament_add_gtest(
    test_load_joint_trajectory_controller
    test/test_load_joint_trajectory_controller.cpp
  )
  target_include_directories(test_load_joint_trajectory_controller PRIVATE include)
  ament_target_dependencies(test_load_joint_trajectory_controller
    controller_manager
    control_toolbox
    realtime_tools
    ros2_control_test_assets
  )

  # TODO(andyz): Disabled due to flakiness
  # ament_add_gtest(
  #   test_trajectory_actions
  #   test/test_trajectory_actions.cpp
  # )
  # target_include_directories(test_trajectory_actions PRIVATE include)
  # target_link_libraries(test_trajectory_actions
  #   ${PROJECT_NAME}
  # )
  # ament_target_dependencies(test_trajectory_actions
  #   ${THIS_PACKAGE_INCLUDE_DEPENDS}
  # )
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
