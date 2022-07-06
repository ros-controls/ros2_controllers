cmake_minimum_required(VERSION 3.5)
project(admittance_controller)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rcutils REQUIRED)
find_package(tf2_kdl REQUIRED)


# The admittance controller
add_library(admittance_controller SHARED
        src/admittance_controller.cpp
        )

target_include_directories(
        admittance_controller
        PRIVATE
        include
)

# uncomment to generate struct
#find_package(gen_param_struct REQUIRED)
## specify which yaml root name to convert to struct
#set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/include/admittance_controller/config/definition.yaml)
## specify which yaml root name to convert to struct
#set(OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/admittance_controller/config)
#
#generate_param_struct_header(admittance_controller ${OUT_DIR} ${YAML_FILE})


ament_target_dependencies(
        admittance_controller
        backward_ros
        control_msgs
        control_toolbox
        controller_interface
        kinematics_interface
        Eigen3
        geometry_msgs
        hardware_interface
        joint_trajectory_controller
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        tf2
        tf2_eigen
        tf2_kdl
        tf2_geometry_msgs
        tf2_ros
        angles
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(admittance_controller PRIVATE "ADMITTANCE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)


install(
        TARGETS admittance_controller
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

if (BUILD_TESTING)
    # create custom test function to pass yaml file into test main
    function(add_test_with_yaml_input TARGET SOURCES YAML_FILE)
        add_executable(${TARGET} ${SOURCES})
        _ament_cmake_gmock_find_gmock()
        target_include_directories(${TARGET} PUBLIC "${GMOCK_INCLUDE_DIRS}")
        target_link_libraries(${TARGET} ${GMOCK_LIBRARIES})
        set(executable "$<TARGET_FILE:${TARGET}>")
        set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.gtest.xml")
        ament_add_test(
            ${TARGET}
            COMMAND ${executable} --ros-args --params-file ${YAML_FILE}
            --gtest_output=xml:${result_file}
            OUTPUT_FILE ${CMAKE_BINARY_DIR}/ament_cmake_gmock/test_load_admittance_controller.txt
            RESULT_FILE ${result_file}
        )
    endfunction()
    # find testing packages
    find_package(ament_cmake_gmock REQUIRED)
    find_package(controller_manager REQUIRED)
    find_package(hardware_interface REQUIRED)
    find_package(ros2_control_test_assets REQUIRED)
    # test loading admittance controller
    add_test_with_yaml_input(test_load_admittance_controller "test/test_load_admittance_controller.cpp"
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml)
    target_include_directories(test_load_admittance_controller PUBLIC "${GMOCK_INCLUDE_DIRS}")
    target_link_libraries(test_load_admittance_controller ${GMOCK_LIBRARIES})
    ament_target_dependencies(
        test_load_admittance_controller
        controller_manager
        hardware_interface
        ros2_control_test_assets
    )
    # test admittance controller function
#    ament_add_gmock(test_admittance_controller test/test_admittance_controller.cpp)
    add_test_with_yaml_input(test_admittance_controller "test/test_admittance_controller.cpp"
        ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml)
    target_include_directories(test_admittance_controller PRIVATE include)
    target_link_libraries(test_admittance_controller admittance_controller)
    ament_target_dependencies(
            test_admittance_controller
            control_msgs
            controller_interface
            hardware_interface
            ros2_control_test_assets
    )

endif ()

ament_export_include_directories(
        include
)

ament_export_libraries(
        admittance_controller
)

ament_export_dependencies(
        backward_ros
        control_msgs
        control_toolbox
        controller_interface
        kinematics_interface
        geometry_msgs
        hardware_interface
        joint_trajectory_controller
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        tf2
        tf2_eigen
        tf2_geometry_msgs tf2_ros
)

ament_package()
