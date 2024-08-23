cmake_minimum_required(VERSION 3.16)
project(joint_trajectory_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  joint_limits
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  ruckig
  tl_expected
  trajectory_msgs
  tf2
  tf2_geometry_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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
  ruckig::ruckig
)
ament_target_dependencies(joint_trajectory_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_trajectory_controller PRIVATE "JOINT_TRAJECTORY_CONTROLLER_BUILDING_DLL" "_USE_MATH_DEFINES")

add_library(cartesian_trajectory_generator SHARED
  src/cartesian_trajectory_generator.cpp
)
target_compile_features(cartesian_trajectory_generator PUBLIC cxx_std_17)
target_include_directories(cartesian_trajectory_generator PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_trajectory_controller>
)
target_link_libraries(cartesian_trajectory_generator PUBLIC
  joint_trajectory_controller
)
ament_target_dependencies(cartesian_trajectory_generator PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface joint_trajectory_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # TODO(henrygerardmoore): re-enable when ros2_controllers is synced with main
  # ament_add_gmock(test_trajectory test/test_trajectory.cpp)
  # target_link_libraries(test_trajectory joint_trajectory_controller)
  # target_compile_definitions(test_trajectory PRIVATE _USE_MATH_DEFINES)

  ament_add_gmock(test_trajectory_controller
    test/test_trajectory_controller.cpp)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_link_libraries(test_trajectory_controller
    joint_trajectory_controller
  )
  target_compile_definitions(joint_trajectory_controller PRIVATE _USE_MATH_DEFINES)

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

ament_add_gmock(test_load_cartesian_trajectory_generator
  test/test_load_cartesian_trajectory_generator.cpp
)
target_link_libraries(test_load_cartesian_trajectory_generator
  cartesian_trajectory_generator
)
ament_target_dependencies(test_load_cartesian_trajectory_generator
  controller_manager
  control_toolbox
  realtime_tools
  ros2_control_test_assets
)

  ament_add_gmock(test_trajectory_actions
    test/test_trajectory_actions.cpp
  )
  set_tests_properties(test_trajectory_actions PROPERTIES TIMEOUT 300)
  target_link_libraries(test_trajectory_actions
    joint_trajectory_controller
  )
endif()


install(
  DIRECTORY include/
  DESTINATION include/joint_trajectory_controller
)
install(TARGETS
    cartesian_trajectory_generator
    joint_trajectory_controller
    joint_trajectory_controller_parameters
  EXPORT export_joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
