cmake_minimum_required(VERSION 3.16)
project(cartesian_trajectory_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  joint_trajectory_controller
  kinematics_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

generate_parameter_library(cartesian_trajectory_controller_parameters
  src/cartesian_trajectory_controller_parameters.yaml
)

add_library(cartesian_trajectory_controller SHARED
  src/cartesian_trajectory_controller.cpp
  src/cartesian_trajectory.cpp
)
target_include_directories(cartesian_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/cartesian_trajectory_controller>
)
target_link_libraries(cartesian_trajectory_controller PUBLIC
  cartesian_trajectory_controller_parameters
  Eigen3::Eigen
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  joint_trajectory_controller::joint_trajectory_controller
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  tf2_eigen::tf2_eigen
  ${geometry_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface cartesian_trajectory_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Dynamically loaded during tests
  find_package(kinematics_interface_kdl REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")

  ament_add_gmock(test_load_cartesian_trajectory_controller
    test/test_load_cartesian_trajectory_controller.cpp)
  target_link_libraries(test_load_cartesian_trajectory_controller
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_cartesian_trajectory test/test_cartesian_trajectory.cpp)
  target_link_libraries(test_cartesian_trajectory
    cartesian_trajectory_controller
    Eigen3::Eigen
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/cartesian_trajectory_controller
)

install(TARGETS cartesian_trajectory_controller cartesian_trajectory_controller_parameters
  EXPORT export_cartesian_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_cartesian_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
