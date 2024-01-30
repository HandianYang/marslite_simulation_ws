################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(marslite_shared_control)

add_compile_options(-std=c++14)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sensor_msgs
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
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
    sensor_msgs
  INCLUDE_DIRS include
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Declare C++ library
add_library(virtual_zone
  src/virtual_zone.cpp
)
add_dependencies(virtual_zone  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(virtual_zone  ${catkin_LIBRARIES})

### Declare C++ executables and link the library to the executables
add_executable(static_virtual_zone src/static_virtual_zone.cpp)
target_link_libraries(static_virtual_zone ${catkin_LIBRARIES} virtual_zone)

add_executable(deformable_virtual_zone src/deformable_virtual_zone.cpp)
target_link_libraries(deformable_virtual_zone ${catkin_LIBRARIES} virtual_zone)


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
    virtual_zone
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install C++ libraries
install(TARGETS
    static_virtual_zone
    deformable_virtual_zone
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
