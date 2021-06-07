cmake_minimum_required(VERSION 3.5)
project(admittance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(
  admittance_controller
  SHARED
  src/admittance_controller.cpp
  src/admittance_rule.cpp
  src/moveit_kinematics.cpp
)
target_include_directories(
  admittance_controller
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(
  admittance_controller
  control_msgs
  controller_interface
  ${Eigen_LIBRARIES}
  filters
  geometry_msgs
  hardware_interface
  moveit_ros_move_group
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(admittance_controller PRIVATE "ADMITTANCE_CONTROLLER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(admittance_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)

install(
  TARGETS admittance_controller
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

  ament_add_gmock(test_load_admittance_controller test/test_load_admittance_controller.cpp)
  target_include_directories(test_load_admittance_controller PRIVATE include)
  ament_target_dependencies(
    test_load_admittance_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_admittance_controller test/test_admittance_controller.cpp)
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  admittance_controller
)
ament_export_dependencies(
  control_msgs
  controller_interface
  filters
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
