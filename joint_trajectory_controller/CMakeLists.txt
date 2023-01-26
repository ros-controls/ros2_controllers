cmake_minimum_required(VERSION 3.16)
project(joint_trajectory_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rsl REQUIRED)
find_package(tl_expected REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
)

generate_parameter_library(joint_trajectory_controller_parameters
  src/joint_trajectory_controller_parameters.yaml
  include/joint_trajectory_controller/validate_jtc_parameters.hpp
)

add_library(joint_trajectory_controller SHARED
  src/joint_trajectory_controller.cpp
  src/trajectory.cpp
)
target_compile_features(joint_trajectory_controller PUBLIC cxx_std_17)
target_include_directories(joint_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_controller>
)
target_link_libraries(joint_trajectory_controller PUBLIC
  joint_trajectory_controller_parameters
)
ament_target_dependencies(joint_trajectory_controller PUBLIC
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_trajectory_controller PRIVATE "JOINT_TRAJECTORY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")
pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory joint_trajectory_controller)

  ament_add_gmock(test_trajectory_controller
    test/test_trajectory_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_joint_trajectory_controller.yaml)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_link_libraries(test_trajectory_controller
    joint_trajectory_controller
  )

  ament_add_gmock(test_load_joint_trajectory_controller
    test/test_load_joint_trajectory_controller.cpp
  )
  target_link_libraries(test_load_joint_trajectory_controller
    joint_trajectory_controller
  )
  ament_target_dependencies(test_load_joint_trajectory_controller
    controller_manager
    control_toolbox
    realtime_tools
    ros2_control_test_assets
  )

  # TODO(andyz): Disabled due to flakiness
  # ament_add_gmock(test_trajectory_actions
  #   test/test_trajectory_actions.cpp
  # )
  # target_link_libraries(test_trajectory_actions
  #   joint_trajectory_controller
  # )
endif()


install(
  DIRECTORY include/
  DESTINATION include/joint_trajectory_controller
)
install(TARGETS
    joint_trajectory_controller
    joint_trajectory_controller_parameters
  EXPORT export_joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
)
ament_package()
