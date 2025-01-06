################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################

cmake_minimum_required(VERSION 3.5)
project(product_detection)

add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
)
find_package(OpenCV REQUIRED)

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
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# add_executable(build_point_cloud src/build_point_cloud.cpp)
# target_link_libraries(build_point_cloud ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/object_detection_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ executables
# install(TARGETS
#     build_point_cloud
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Add launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


################################################################################
# Test
################################################################################