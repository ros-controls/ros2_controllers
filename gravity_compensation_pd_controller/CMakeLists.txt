cmake_minimum_required(VERSION 3.16)
project(gravity_compensation_pd_controller)

if(CMAKE_COMPILER_IS_GNUCXX
  OR CMAKE_CXX_COMPILER_ID
    MATCHES
    "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    Eigen3
    generate_parameter_library
    hardware_interface
    inverse_dynamics_solver
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    urdf
)

find_package(ament_cmake REQUIRED)

foreach(dependency IN
        ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(gravity_compensation_pd_controller_parameters
                          src/gravity_compensation_pd_controller_parameters.yaml
)

add_library(gravity_compensation_pd_controller SHARED
            src/gravity_compensation_pd_controller.cpp
)

target_compile_features(
  gravity_compensation_pd_controller
  PUBLIC cxx_std_17
)

target_include_directories(
  gravity_compensation_pd_controller
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/gravity_compensation_pd_controller>
)

target_link_libraries(
  gravity_compensation_pd_controller
  PUBLIC gravity_compensation_pd_controller_parameters
)

target_link_libraries(
  gravity_compensation_pd_controller
  PUBLIC controller_interface::controller_interface
        hardware_interface::hardware_interface
        hardware_interface::mock_components
        inverse_dynamics_solver::inverse_dynamics_solver_lib
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
        realtime_tools::realtime_tools
        realtime_tools::thread_priority
        urdf::urdf
)

# Causes the visibility macros to use dllexport rather than dllimport, which is appropriate when
# building the dll but not consuming it.
target_compile_definitions(
  gravity_compensation_pd_controller
  PRIVATE "GRAVITY_COMPENSATION_PD_CONTROLLER_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(
  controller_interface gravity_compensation_pd_controller.xml
)

install(DIRECTORY include
        DESTINATION include/${PROJECT_NAME}
)

install(TARGETS gravity_compensation_pd_controller gravity_compensation_pd_controller_parameters
        EXPORT export_gravity_compensation_pd_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_gravity_compensation_pd_controller
    test/test_gravity_compensation_pd_controller.cpp
  )
  target_link_libraries(test_gravity_compensation_pd_controller
    gravity_compensation_pd_controller
    ros2_control_test_assets::ros2_control_test_assets
  )

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_gravity_compensation_pd_controller test/test_load_gravity_compensation_pd_controller.cpp)
  target_link_libraries(test_load_gravity_compensation_pd_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

ament_export_targets(export_gravity_compensation_pd_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
