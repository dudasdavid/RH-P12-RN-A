################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(rh_p12_rn_a_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  LIBRARIES rh_p12_rn_a_tools CATKIN_DEPENDS roscpp sensor_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

set(SOURCES
  src/rh_p12_rn_a_joint_state_pub_node.cpp
)
add_library(${PROJECT_NAME} ${SOURCES})

add_executable(rh_p12_rn_a_joint_state_pub_node src/rh_p12_rn_a_joint_state_pub_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES})
target_link_libraries(rh_p12_rn_a_joint_state_pub_node ${PROJECT_NAME})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME} rh_p12_rn_a_joint_state_pub_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN "*~" EXCLUDE
)

################################################################################
# Test
################################################################################
