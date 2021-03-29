cmake_minimum_required(VERSION 3.0.2)
project(gripper_action_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin
  REQUIRED COMPONENTS
      actionlib
      control_msgs
      control_toolbox
      controller_interface
      hardware_interface
      pluginlib
      realtime_tools
      roscpp
      urdf
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES gripper_action_controller
CATKIN_DEPENDS
  actionlib
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  roscpp
  urdf
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(gripper_action_controller
   src/gripper_action_controller.cpp
)
target_link_libraries(gripper_action_controller ${catkin_LIBRARIES})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS gripper_action_controller
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES ros_control_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
