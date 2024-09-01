cmake_minimum_required(VERSION 3.16)
project(multi_time_trajectory_controller LANGUAGES CXX)

# add_compile_options(-fsanitize=address)
# add_link_options(-fsanitize=address)
# add_compile_options(-fno-omit-frame-pointer)

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
  joint_trajectory_controller
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

generate_parameter_library(multi_time_trajectory_controller_parameters
  src/multi_time_trajectory_controller_parameters.yaml
  include/multi_time_trajectory_controller/validate_mttc_parameters.hpp
)

add_library(multi_time_trajectory_controller SHARED
  src/multi_time_trajectory_controller.cpp
  src/trajectory.cpp
)
target_compile_features(multi_time_trajectory_controller PUBLIC cxx_std_17)
target_include_directories(multi_time_trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/multi_time_trajectory_controller>
)
target_link_libraries(multi_time_trajectory_controller PUBLIC
  multi_time_trajectory_controller_parameters
  ruckig::ruckig
)
ament_target_dependencies(multi_time_trajectory_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(controller_interface multi_time_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_mttc
    test/test_mttc.cpp)
  set_tests_properties(test_mttc PROPERTIES TIMEOUT 220)
  target_link_libraries(test_mttc
    multi_time_trajectory_controller
  )
  target_compile_definitions(multi_time_trajectory_controller PRIVATE _USE_MATH_DEFINES)

  ament_add_gmock(test_load_mttc
    test/test_load_mttc.cpp
  )
  target_link_libraries(test_load_mttc
    multi_time_trajectory_controller
  )
  ament_target_dependencies(test_load_mttc
    controller_manager
    control_toolbox
    realtime_tools
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/multi_time_trajectory_controller
)
install(TARGETS
    multi_time_trajectory_controller
    multi_time_trajectory_controller_parameters
  EXPORT export_multi_time_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_multi_time_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
