################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++14)

project(marslite_control)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  tf
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
    control_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    tf
    trajectory_msgs
  LIBRARIES
    tm700_kin_lib
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ../marslite_properties/include
  ../tm_driver_t/include
  ${catkin_INCLUDE_DIRS}
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(tm_kinematics_lib
  src/tm_kinematics.cpp
)

add_library(control_lib
  src/model_predictive_control.cpp
  src/marslite_control.cpp
)
target_link_libraries(control_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen tm700_kin_lib tm_kinematics_lib)

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
  #
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################

add_executable(test_default_pose_planning test/test_default_pose_planning.cpp)
target_link_libraries(test_default_pose_planning ${catkin_LIBRARIES} control_lib)

add_executable(test_joystick_teleoperation test/test_joystick_teleoperation.cpp)
target_link_libraries(test_joystick_teleoperation ${catkin_LIBRARIES} control_lib)

add_executable(test_kinematics test/test_kinematics.cpp)
target_link_libraries(test_kinematics ${catkin_LIBRARIES} tm_kinematics_lib tm700_kin_lib)