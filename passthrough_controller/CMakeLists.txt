cmake_minimum_required(VERSION 3.16)
project(passthrough_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  generate_parameter_library
  controller_interface
  hardware_interface
  parameter_traits
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(passthrough_controller_parameters
  src/passthrough_controller_parameters.yaml
)

add_library(passthrough_controller SHARED
  src/passthrough_controller.cpp
)
target_compile_features(passthrough_controller PUBLIC cxx_std_17)
target_include_directories(passthrough_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/passthrough_controller>
)
 target_link_libraries(passthrough_controller PUBLIC
   passthrough_controller_parameters
 )
ament_target_dependencies(passthrough_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(passthrough_controller PRIVATE "passthrough_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(controller_interface passthrough_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_controller test/test_load_controller.cpp)
  target_include_directories(test_load_controller PRIVATE include)
  ament_target_dependencies(
    test_load_controller
    controller_manager
    ros2_control_test_assets
  )

  ament_add_gmock(test_passthrough_controller
    test/test_passthrough_controller.cpp
  )
  target_link_libraries(test_passthrough_controller
    passthrough_controller
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/passthrough_controller
)

install(TARGETS
    passthrough_controller
    passthrough_controller_parameters
  EXPORT export_passthrough_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_passthrough_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
