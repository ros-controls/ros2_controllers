cmake_minimum_required(VERSION 3.16)
project(admittance_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
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
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)

generate_parameter_library(admittance_controller_parameters
  src/admittance_controller_parameters.yaml
)

add_library(admittance_controller SHARED
  src/admittance_controller.cpp
)
target_compile_features(admittance_controller PUBLIC cxx_std_17)
target_include_directories(admittance_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/admittance_controller>
)
target_link_libraries(admittance_controller PUBLIC
  admittance_controller_parameters
  Eigen3::Eigen
)
ament_target_dependencies(admittance_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(admittance_controller PRIVATE "ADMITTANCE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Dynamically loaded during test
  find_package(kinematics_interface_kdl REQUIRED)

  # test loading admittance controller
  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_admittance_controller test/test_load_admittance_controller.cpp)
  ament_target_dependencies(test_load_admittance_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # test admittance controller function
  add_rostest_with_parameters_gmock(test_admittance_controller
    test/test_admittance_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )
  target_link_libraries(test_admittance_controller admittance_controller)
  ament_target_dependencies(test_admittance_controller
    control_msgs
    controller_interface
    hardware_interface
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/admittance_controller
)

install(TARGETS admittance_controller admittance_controller_parameters
  EXPORT export_admittance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_admittance_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
