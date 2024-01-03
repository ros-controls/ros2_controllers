cmake_minimum_required(VERSION 3.16)
project(position_controllers LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  forward_command_controller
  pluginlib
  rclcpp
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(position_controllers SHARED
  src/joint_group_position_controller.cpp
)
target_compile_features(position_controllers PUBLIC cxx_std_17)
target_include_directories(position_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/position_controllers>
)
ament_target_dependencies(position_controllers PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(position_controllers PRIVATE "POSITION_CONTROLLERS_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface position_controllers_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_joint_group_position_controller
    test/test_load_joint_group_position_controller.cpp
  )
  target_link_libraries(test_load_joint_group_position_controller
    position_controllers
  )
  ament_target_dependencies(test_load_joint_group_position_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(test_joint_group_position_controller
    test/test_joint_group_position_controller.cpp
  )
  target_link_libraries(test_joint_group_position_controller
    position_controllers
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/position_controllers
)
install(
  TARGETS position_controllers
  EXPORT export_position_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_position_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
