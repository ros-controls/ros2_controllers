# cmake_minimum_required and project setup
cmake_minimum_required(VERSION 3.8)
project(chained_filter_controller)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)

# Generate parameters from YAML
generate_parameter_library(
  chained_filter_parameters
  config/chained_filter_parameters.yaml
)

#include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

# Library definition
add_library(${PROJECT_NAME} SHARED
  src/chained_filter.cpp
  #${chained_filter_parameters_INTERFACE_SOURCES}
)

target_link_libraries(${PROJECT_NAME} chained_filter_parameters)

# Include paths
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  #${chained_filter_parameters_INCLUDE_DIRS}
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  filters
  parameter_traits
)

# Export the plugin description
pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  filters
  parameter_traits
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Installation
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  chained_filter_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Testing support
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_chained_filter_controller
    test/test_chained_filter.cpp)
  target_link_libraries(test_chained_filter_controller
    ${PROJECT_NAME})

  ament_target_dependencies(test_chained_filter_controller
    controller_interface
    hardware_interface
    pluginlib
    rclcpp)
endif()

ament_package()
