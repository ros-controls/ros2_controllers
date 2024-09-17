cmake_minimum_required(VERSION 3.16)
project(pose_broadcaster
  LANGUAGES
    CXX
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(pose_broadcaster_parameters
  src/pose_broadcaster_parameters.yaml
)

add_library(pose_broadcaster SHARED
  src/pose_broadcaster.cpp
)
target_compile_features(pose_broadcaster PUBLIC
    cxx_std_17
)
target_include_directories(pose_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(pose_broadcaster PUBLIC
  pose_broadcaster_parameters
)
ament_target_dependencies(pose_broadcaster PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pose_broadcaster PRIVATE "POSE_BROADCASTER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pose_broadcaster.xml
)

if(BUILD_TESTING)
  #  find_package(ament_lint_auto REQUIRED)
  #  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    pose_broadcaster
    pose_broadcaster_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
