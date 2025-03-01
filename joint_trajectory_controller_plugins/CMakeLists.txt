cmake_minimum_required(VERSION 3.8)
project(joint_trajectory_controller_plugins LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wconversion)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_toolbox
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(pid_trajectory_plugin_parameters
  src/pid_trajectory_plugin_parameters.yaml
)

add_library(pid_trajectory_plugin SHARED src/pid_trajectory_plugin.cpp)
add_library(joint_trajectory_controller_plugins::pid_trajectory_plugin ALIAS pid_trajectory_plugin)
target_compile_features(pid_trajectory_plugin PUBLIC cxx_std_17)
target_include_directories(pid_trajectory_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(pid_trajectory_plugin PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(pid_trajectory_plugin PUBLIC
  pid_trajectory_plugin_parameters
)
pluginlib_export_plugin_description_file(joint_trajectory_controller_plugins plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS pid_trajectory_plugin pid_trajectory_plugin_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  ament_add_gmock(test_load_pid_trajectory test/test_load_pid_trajectory.cpp)
  target_link_libraries(test_load_pid_trajectory pid_trajectory_plugin pid_trajectory_plugin_parameters)
  ament_target_dependencies(test_load_pid_trajectory ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gmock(test_pid_trajectory test/test_pid_trajectory.cpp)
  target_link_libraries(test_pid_trajectory pid_trajectory_plugin pid_trajectory_plugin_parameters)
  ament_target_dependencies(test_pid_trajectory ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  pid_trajectory_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
