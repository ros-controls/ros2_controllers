cmake_minimum_required(VERSION 3.8)
project(io_gripper_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_cmake_gmock
  control_msgs
  controller_interface
  controller_manager
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ros2_control_test_assets
  sensor_msgs
  std_msgs
  std_srvs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(io_gripper_controller_parameters
  src/io_gripper_controller.yaml
)
add_library(
  io_gripper_controller
  SHARED
  src/io_gripper_controller.cpp
)
target_include_directories(io_gripper_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(io_gripper_controller io_gripper_controller_parameters)
ament_target_dependencies(io_gripper_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(io_gripper_controller PRIVATE "io_gripper_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface io_gripper_controller.xml)

install(
  TARGETS
  io_gripper_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_gmock(test_load_io_gripper_controller test/test_load_io_gripper_controller.cpp)
  target_include_directories(test_load_io_gripper_controller PRIVATE include)
  ament_target_dependencies(
    test_load_io_gripper_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_io_gripper_controller 
    test/test_io_gripper_controller.cpp
    test/test_io_gripper_controller_open.cpp
    test/test_io_gripper_controller_close.cpp
    test/test_io_gripper_controller_all_param_set.cpp
    test/test_io_gripper_controller_open_close_action.cpp
    test/test_io_gripper_controller_reconfigure.cpp
    test/test_io_gripper_controller_reconfigure_action.cpp
  )
  target_include_directories(test_io_gripper_controller PRIVATE include)
  target_link_libraries(test_io_gripper_controller io_gripper_controller)
  ament_target_dependencies(
    test_io_gripper_controller
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
  io_gripper_controller
)

ament_package()
