cmake_minimum_required(VERSION 3.8)
project(trajectory_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(pid_trajectory_plugin_parameters
  src/pid_trajectory_plugin_parameters.yaml
)

add_library(pid_trajectory_plugin SHARED src/pid_trajectory_plugin.cpp)
add_library(trajectory_plugins::pid_trajectory_plugin ALIAS pid_trajectory_plugin)
target_compile_features(pid_trajectory_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pid_trajectory_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(pid_trajectory_plugin PUBLIC
  "joint_trajectory_controller"
  "control_toolbox"
  "pluginlib"
  "generate_parameter_library"
)
target_link_libraries(pid_trajectory_plugin PUBLIC
  pid_trajectory_plugin_parameters
)
pluginlib_export_plugin_description_file(joint_trajectory_controller plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pid_trajectory_plugin PRIVATE "TRAJECTORY_PLUGINS_BUILDING_LIBRARY")

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
  target_link_libraries(test_load_pid_trajectory pid_trajectory_plugin_parameters)
  ament_target_dependencies(test_load_pid_trajectory
    "joint_trajectory_controller"
    "rclcpp"
    "pluginlib"
    "generate_parameter_library"
  )
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

ament_package()
