cmake_minimum_required(VERSION 3.16)
project(steering_controllers_library LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2
  tf2_msgs
  tf2_geometry_msgs
  ackermann_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/steering_controllers_library.cpp
  src/steering_odometry.cpp
)

generate_parameter_library(steering_controllers_library_parameters
  config/steering_controllers_library.yaml
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  steering_controllers_library_parameters)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "STEERING_CONTROLLERS_BUILDING_DLL")

install(
  TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  add_rostest_with_parameters_gmock(
    test_steering_controllers_library test/test_steering_controllers_library.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/steering_controllers_library_params.yaml)
  target_include_directories(test_steering_controllers_library PRIVATE include)
  target_link_libraries(test_steering_controllers_library steering_controllers_library)
  ament_target_dependencies(
    test_steering_controllers_library
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
  include/steering_controllers_library_parameters
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
