cmake_minimum_required(VERSION 2.8.3)
project(pal_wbc_utils)

set(CATKIN_COMPONENTS
  roscpp
  pal_utils
  math_utils
  backward_ros
  pal_ros_utils
  wbc_tasks
  pluginlib
  property_bag
  pal_wbc_msgs
  pal_wbc_controller
  pal_physics_utils
  pal_rosdoc)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_interactive_marker_pose_task src/push_interactive_marker_pose_task.cpp)
target_link_libraries(push_interactive_marker_pose_task ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_interactive_marker_gaze_task src/push_interactive_marker_gaze_task.cpp)
target_link_libraries(push_interactive_marker_gaze_task ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_torso_height_ref src/push_torso_height_ref.cpp)
target_link_libraries(push_torso_height_ref ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(push_interactive_marker_relative_task src/push_interactive_marker_relative_task.cpp)
target_link_libraries(push_interactive_marker_relative_task ${PROJECT_NAME} ${catkin_LIBRARIES})


add_library(common_stacks
    src/common_stacks/dynamic/joint_space_hold_position.cpp
    src/common_stacks/dynamic/joint_space_spline.cpp
    src/common_stacks/dynamic/task_space.cpp
)
target_link_libraries(common_stacks ${catkin_LIBRARIES})


### Install ###
generate_and_install_pal_rosdoc()

install(TARGETS common_stacks
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    push_interactive_marker_pose_task
    push_interactive_marker_gaze_task
    push_interactive_marker_relative_task
    push_torso_height_ref
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


install(FILES common_stacks_wbc_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


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
