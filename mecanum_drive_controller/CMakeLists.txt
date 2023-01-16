cmake_minimum_required(VERSION 3.8)
project(mecanum_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add mecanum_drive_controller library related compile commands
generate_parameter_library(mecanum_drive_controller_parameters
  src/mecanum_drive_controller.yaml
)
add_library(
  mecanum_drive_controller
  SHARED
  src/mecanum_drive_controller.cpp
  src/odometry.cpp
)
target_include_directories(mecanum_drive_controller PUBLIC 
"$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mecanum_drive_controller mecanum_drive_controller_parameters)
ament_target_dependencies(mecanum_drive_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(mecanum_drive_controller PRIVATE "MECANUM_DRIVE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface mecanum_drive_controller.xml)

install(
  TARGETS
  mecanum_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_mecanum_drive_controller test/test_load_mecanum_drive_controller.cpp)
  target_include_directories(test_load_mecanum_drive_controller PRIVATE include)
  ament_target_dependencies(
    test_load_mecanum_drive_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_mecanum_drive_controller test/test_mecanum_drive_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/mecanum_drive_controller_params.yaml)
  target_include_directories(test_mecanum_drive_controller PRIVATE include)
  target_link_libraries(test_mecanum_drive_controller mecanum_drive_controller)
  ament_target_dependencies(
    test_mecanum_drive_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_mecanum_drive_controller_preceeding test/test_mecanum_drive_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/mecanum_drive_controller_preceeding_params.yaml)
  target_include_directories(test_mecanum_drive_controller_preceeding PRIVATE include)
  target_link_libraries(test_mecanum_drive_controller_preceeding mecanum_drive_controller)
  ament_target_dependencies(
    test_mecanum_drive_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  mecanum_drive_controller
)

ament_package()
