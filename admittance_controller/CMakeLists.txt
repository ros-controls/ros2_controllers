cmake_minimum_required(VERSION 3.5)
project(admittance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  kinematics_interface
  Eigen3
  generate_parameter_library
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(admittance_controller SHARED src/admittance_controller.cpp)
target_include_directories(admittance_controller PRIVATE include)
generate_parameter_library(admittance_controller_parameters src/admittance_controller_parameters.yaml)
target_link_libraries(admittance_controller admittance_controller_parameters)
ament_target_dependencies(admittance_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(admittance_controller PRIVATE "ADMITTANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS admittance_controller admittance_controller_parameters
  EXPORT export_admittance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_interface REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # test loading admittance controller
  add_rostest_with_parameters_gmock(test_load_admittance_controller test/test_load_admittance_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml)
  target_include_directories(test_load_admittance_controller PUBLIC ${GMOCK_INCLUDE_DIRS})
  target_link_libraries(test_load_admittance_controller ${GMOCK_LIBRARIES})
  ament_target_dependencies(
    test_load_admittance_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
  # test admittance controller function
  add_rostest_with_parameters_gmock(test_admittance_controller test/test_admittance_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml)
  target_include_directories(test_admittance_controller PRIVATE include)
  target_link_libraries(test_admittance_controller admittance_controller)
  ament_target_dependencies(
    test_admittance_controller
    control_msgs
    controller_interface
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_targets(
  export_admittance_controller HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()
