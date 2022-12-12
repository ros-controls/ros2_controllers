cmake_minimum_required(VERSION 3.8)
project(tricycle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  nav_msgs
  std_srvs
  geometry_msgs
  rclcpp
  rcpputils
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
  ackermann_msgs
  steering_odometry
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(
  ${PROJECT_NAME}
  SHARED
  src/tricycle_controller.cpp
  src/traction_limiter.cpp
  src/steering_limiter.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface tricycle_controller.xml)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_tricycle_controller
    test/test_tricycle_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_tricycle_controller.yaml)
  target_include_directories(test_tricycle_controller PRIVATE include)
  target_link_libraries(test_tricycle_controller
    ${PROJECT_NAME}
  )

  ament_target_dependencies(test_tricycle_controller
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
    test_load_tricycle_controller
    test/test_load_tricycle_controller.cpp
  )
  target_include_directories(test_load_tricycle_controller PRIVATE include)
  ament_target_dependencies(test_load_tricycle_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
