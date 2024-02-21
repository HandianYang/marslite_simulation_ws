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
  dynamic_reconfigure
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

generate_dynamic_reconfigure_options(
  cfg/VirtualZone.cfg
  cfg/AdaptiveController.cfg
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
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
  ../marslite_properties/include
  ${catkin_INCLUDE_DIRS}
)

# Declare C++ library

add_library(teleop_interface
  src/teleoperation/teleop_interface.cpp
  src/teleoperation/teleop_joystick.cpp
  src/teleoperation/teleop_keyboard.cpp
)
add_dependencies(teleop_interface  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_interface  ${catkin_LIBRARIES})

add_library(virtual_zone
  src/shared_control/virtual_zone.cpp
  src/shared_control/static_virtual_zone.cpp
  src/shared_control/deformable_virtual_zone.cpp
)
add_dependencies(virtual_zone  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(virtual_zone  ${catkin_LIBRARIES})

add_library(adaptive_controller
  src/shared_control/virtual_zone.cpp
  src/shared_control/static_virtual_zone.cpp
  src/shared_control/deformable_virtual_zone.cpp
  src/shared_control/adaptive_controller.cpp
)
add_dependencies(adaptive_controller  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(adaptive_controller  ${catkin_LIBRARIES})

# Declare C++ executables and
#  link the library to the executables

add_executable(teleop_keyboard src/teleoperation/teleop_keyboard_main.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES} teleop_interface adaptive_controller)

add_executable(teleop_joystick src/teleoperation/teleop_joystick_main.cpp)
target_link_libraries(teleop_joystick ${catkin_LIBRARIES} teleop_interface adaptive_controller)


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
    virtual_zone
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
