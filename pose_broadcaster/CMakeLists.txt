cmake_minimum_required(VERSION 3.16)
project(pose_broadcaster)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
                      -Werror=missing-braces)
endif()

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_msgs
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

pluginlib_export_plugin_description_file(
  controller_interface pose_broadcaster.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_pose_broadcaster
    test/test_load_pose_broadcaster.cpp
  )
  target_link_libraries(test_load_pose_broadcaster
    pose_broadcaster
  )
  ament_target_dependencies(test_load_pose_broadcaster
    controller_manager
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_pose_broadcaster
    test/test_pose_broadcaster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/pose_broadcaster_params.yaml
  )
  target_link_libraries(test_pose_broadcaster
    pose_broadcaster
  )
  ament_target_dependencies(test_pose_broadcaster
    hardware_interface
  )
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
