cmake_minimum_required(VERSION 2.8.3)
project(pal_wbc_utils)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

# Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pal_utils
  math_utils
  backward_ros
  pal_robot_tools
  pal_wbc_msgs
  pal_ros_utils
  property_bag
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp pal_utils math_utils pal_robot_tools pal_wbc_msgs pal_ros_utils
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
    SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${CATKIN_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME}
    #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
    #find_package(rostest REQUIRED)
    #find_package(pal_test_utils REQUIRED)
    #INCLUDE_DIRECTORIES(${pal_test_utils_INCLUDE_DIRS})
    #add_rostest_gtest(aruco_detection_state-test test/aruco_detection_state.test test/test_aruco_detection_state.cpp)

    #target_link_libraries(aruco_detection_state-test
                        #aruco_detection_state
                        #${catkin_LIBRARIES})
endif(CATKIN_ENABLE_TESTING)
