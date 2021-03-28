cmake_minimum_required(VERSION 3.5)
project(joint_state_controller)

find_package(ament_cmake REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(pluginlib REQUIRED)

add_library(joint_state_controller
  SHARED
  src/joint_state_controller.cpp
)
ament_target_dependencies(joint_state_controller
  joint_state_broadcaster
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_state_controller PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(joint_state_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface joint_state_plugin.xml)

install(
  TARGETS
  joint_state_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  joint_state_broadcaster
)
ament_export_libraries(
  joint_state_controller
)
ament_package()
