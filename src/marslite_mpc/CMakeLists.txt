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
  apriltags_ros
  detection_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
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
    apriltags_ros
    detection_msgs
    geometry_msgs
    message_filters
    nav_msgs
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf
  LIBRARIES
    mpc
    tm700_kin
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

add_library(tm700_kin src/tm_kin.cpp)
set_target_properties(tm700_kin PROPERTIES COMPILE_DEFINITIONS "TM700_DH_PARAMS")

add_library(mpc src/marslite_mpc.cpp)
target_link_libraries(mpc ${catkin_LIBRARIES} OsqpEigen::OsqpEigen tm700_kin)

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} mpc)

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
    main
    # tm700_kin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################
