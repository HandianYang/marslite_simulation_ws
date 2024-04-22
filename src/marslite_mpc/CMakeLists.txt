################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++14)

project(marslite_mpc)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  # actionlib_msgs
  actionlib
  # apriltags_ros
  control_msgs
  # detection_msgs
  geometry_msgs
  # message_filters
  # nav_msgs
  # pcl_conversions
  # pcl_ros
  roscpp
  rospy
  sensor_msgs
  # std_msgs
  # tf
  trajectory_msgs
)

find_package(OsqpEigen)
find_package(Eigen3)



################################################################################
# Setup for python modules and scripts
################################################################################
# catkin_python_setup()

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
  CATKIN_DEPENDS
    actionlib
    # apriltags_ros
    control_msgs
    # detection_msgs
    geometry_msgs
    # message_filters
    # nav_msgs
    # pcl_conversions
    # pcl_ros
    roscpp
    rospy
    sensor_msgs
    # std_msgs
    # tf
    trajectory_msgs
  LIBRARIES
    mpc_lib
    tm700_kin_lib
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ../marslite_properties/include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(tm700_kin_lib src/tm_kin.cpp)
set_target_properties(tm700_kin_lib PROPERTIES COMPILE_DEFINITIONS "TM700_DH_PARAMS")

add_library(mpc_lib src/marslite_mpc.cpp)
target_link_libraries(mpc_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen tm700_kin_lib)

add_executable(mpc src/main.cpp)
target_link_libraries(mpc ${catkin_LIBRARIES} mpc_lib)

# add_executable(trajectory_planner src/trajectory_planner_node.cpp)
# target_link_libraries(trajectory_planner ${catkin_LIBRARIES})

# add_executable(MPCExample src/MPCExample.cpp)
# target_link_libraries(MPCExample ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

################################################################################
# Install
################################################################################

catkin_install_python(PROGRAMS
  #
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  # 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    mpc
    # trajectory_planner
    # MPCExample
    # tm700_kin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################
