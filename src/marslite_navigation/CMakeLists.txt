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
  cfg/SharedControl.cfg
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

add_library(teleop_interface_lib
  src/teleoperation/teleop_interface.cpp
  src/teleoperation/teleop_joystick.cpp
  src/teleoperation/teleop_keyboard.cpp
)
add_dependencies(teleop_interface_lib  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_interface_lib  ${catkin_LIBRARIES})

add_library(virtual_zone_lib
  src/shared_control/virtual_zone.cpp
  src/shared_control/static_virtual_zone.cpp
  src/shared_control/deformable_virtual_zone.cpp
)
add_dependencies(virtual_zone_lib  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(virtual_zone_lib  ${catkin_LIBRARIES})

add_library(shared_control_lib
  src/shared_control/virtual_zone.cpp
  src/shared_control/static_virtual_zone.cpp
  src/shared_control/deformable_virtual_zone.cpp
  src/shared_control/shared_control.cpp
)
add_dependencies(shared_control_lib  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(shared_control_lib  ${catkin_LIBRARIES})

# Declare C++ executables and
#  link the library to the executables

add_executable(teleop_keyboard src/teleop_keyboard_main.cpp)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES} teleop_interface_lib)

add_executable(teleop_joystick src/teleop_joystick_main.cpp)
target_link_libraries(teleop_joystick ${catkin_LIBRARIES} teleop_interface_lib)

add_executable(shared_control src/shared_control_main.cpp)
target_link_libraries(shared_control ${catkin_LIBRARIES} shared_control_lib)


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
    shared_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ libraries
install(TARGETS
    teleop_interface_lib
    virtual_zone_lib
    shared_control_lib
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
