################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(marslite_navigation)

add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

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
    geometry_msgs
    message_filters
    roscpp
    rospy
    sensor_msgs
    std_msgs
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ library
add_library(teleop_interface
  src/teleop_interface.cpp
)

# Declare C++ executables and
#  link the library to the executables

add_executable(teleop_keyboard src/teleop_keyboard.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES} teleop_interface)

add_executable(teleop_joystick src/teleop_joystick.cpp)
target_link_libraries(teleop_joystick ${catkin_LIBRARIES} teleop_interface)


################################################################################
# Install
################################################################################

# Install Python scripts
catkin_install_python(PROGRAMS
  # 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ executables
install(TARGETS
    teleop_keyboard
    teleop_joystick
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ libraries
install(TARGETS
    teleop_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


################################################################################
# Test
################################################################################
