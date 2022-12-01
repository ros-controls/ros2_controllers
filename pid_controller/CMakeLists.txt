cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# find dependencies
find_package(ament_cmake REQUIRED)

# Add pid_controller library related compile commands
generate_parameter_library(pid_controller_parameters
  src/pid_controller.yaml
)
add_library(
  pid_controller
  SHARED
  src/pid_controller.cpp
)
target_include_directories(pid_controller PRIVATE include)
target_link_libraries(pid_controller pid_controller_parameters)
ament_target_dependencies(pid_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(pid_controller PRIVATE "PID_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pid_controller.xml)

install(
  TARGETS
  pid_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_pid_controller test/test_load_pid_controller.cpp)
  target_include_directories(test_load_pid_controller PRIVATE include)
  ament_target_dependencies(
    test_load_pid_controller
    controller_manager
    ros2_control_test_assets
  )

  #add_rostest_with_parameters_gmock(
    #test_pid_controller
    #test/test_pid_controller.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/test/pid_controller_params.yaml
  #)
  #target_include_directories(test_pid_controller PRIVATE include)
  #target_link_libraries(test_pid_controller pid_controller)
  #ament_target_dependencies(
    #test_pid_controller
    #controller_interface
    #hardware_interface
  #)

  #add_rostest_with_parameters_gmock(
    #test_pid_controller_preceeding
    #test/test_pid_controller_preceeding.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}/test/pid_controller_preceeding_params.yaml
  #)
  #target_include_directories(test_pid_controller_preceeding PRIVATE include)
  #target_link_libraries(test_pid_controller_preceeding pid_controller)
  #ament_target_dependencies(
    #test_pid_controller_preceeding
    #controller_interface
    #hardware_interface
  #)
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  pid_controller
)

ament_package()
