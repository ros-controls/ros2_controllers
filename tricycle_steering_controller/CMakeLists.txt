cmake_minimum_required(VERSION 3.16)
project(tricycle_steering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  generate_parameter_library
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  steering_controllers_library
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

generate_parameter_library(tricycle_steering_controller_parameters
  src/tricycle_steering_controller.yaml
)

add_library(
  tricycle_steering_controller
  SHARED
  src/tricycle_steering_controller.cpp
)
target_compile_features(tricycle_steering_controller PUBLIC cxx_std_17)
target_include_directories(tricycle_steering_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tricycle_steering_controller PUBLIC
  tricycle_steering_controller_parameters)
ament_target_dependencies(tricycle_steering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tricycle_steering_controller PRIVATE "TRICYCLE_STEERING_CONTROLLER__VISIBILITY_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface tricycle_steering_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_tricycle_steering_controller test/test_load_tricycle_steering_controller.cpp)
  ament_target_dependencies(test_load_tricycle_steering_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(
    test_tricycle_steering_controller test/test_tricycle_steering_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/tricycle_steering_controller_params.yaml)
  target_include_directories(test_tricycle_steering_controller PRIVATE include)
  target_link_libraries(test_tricycle_steering_controller tricycle_steering_controller)
  ament_target_dependencies(
    test_tricycle_steering_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(
    test_tricycle_steering_controller_preceeding test/test_tricycle_steering_controller_preceeding.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/tricycle_steering_controller_preceeding_params.yaml)
  target_include_directories(test_tricycle_steering_controller_preceeding PRIVATE include)
  target_link_libraries(test_tricycle_steering_controller_preceeding tricycle_steering_controller)
  ament_target_dependencies(
    test_tricycle_steering_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/tricycle_steering_controller
)

install(
  TARGETS tricycle_steering_controller tricycle_steering_controller_parameters
  EXPORT export_tricycle_steering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_tricycle_steering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
