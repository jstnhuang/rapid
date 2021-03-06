cmake_minimum_required(VERSION 2.8.3)
project(rapid_pr2)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  pr2_controllers_msgs
  rapid_manipulation
  rapid_robot
  rapid_ros
  rapid_sound
  rapid_utils
  roscpp
  trajectory_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES
    rapid_pr2
    rapid_pr2_arm_joints
    rapid_pr2_gripper
    rapid_pr2_joint_states
    rapid_pr2_torso
  CATKIN_DEPENDS
    actionlib
    control_msgs
    pr2_controllers_msgs
    rapid_manipulation
    rapid_robot
    rapid_ros
    rapid_sound
    rapid_utils
    roscpp
    trajectory_msgs
    visualization_msgs
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rapid_pr2 src/pr2.cpp)
add_dependencies(rapid_pr2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2 ${catkin_LIBRARIES})

add_library(rapid_pr2_arm_joints src/arm_joints.cpp)
add_dependencies(rapid_pr2_arm_joints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_arm_joints ${catkin_LIBRARIES})

add_library(rapid_pr2_gripper src/gripper.cpp)
add_dependencies(rapid_pr2_gripper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_gripper ${catkin_LIBRARIES})

add_library(rapid_pr2_head src/head.cpp)
add_dependencies(rapid_pr2_head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_head ${catkin_LIBRARIES})

add_library(rapid_pr2_joint_states src/joint_states.cpp)
add_dependencies(rapid_pr2_joint_states ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_joint_states ${catkin_LIBRARIES})

add_library(rapid_pr2_torso src/torso.cpp)
add_dependencies(rapid_pr2_torso ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_torso ${catkin_LIBRARIES})

add_executable(rapid_pr2_gripper_main src/gripper_main.cpp)
set_target_properties(rapid_pr2_gripper_main PROPERTIES OUTPUT_NAME gripper PREFIX "")
add_dependencies(rapid_pr2_gripper_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_gripper_main
  rapid_pr2_gripper
  ${catkin_LIBRARIES}
)

add_executable(rapid_pr2_head_main src/head_main.cpp)
set_target_properties(rapid_pr2_head_main PROPERTIES OUTPUT_NAME head PREFIX "")
add_dependencies(rapid_pr2_head_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_head_main
  rapid_pr2_head
  ${catkin_LIBRARIES}
)

add_executable(rapid_pr2_torso_main src/torso_main.cpp)
set_target_properties(rapid_pr2_torso_main PROPERTIES OUTPUT_NAME torso PREFIX "")
add_dependencies(rapid_pr2_torso_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pr2_torso_main
  rapid_pr2_torso
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rapid_pr2 rapid_pr2_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage") # Enable code coverage
endif()

#catkin_add_gtest(rapid_pr2_test test/pr2_test.cpp)
#if(TARGET rapid_pr2_test)
#  target_link_libraries(rapid_pr2_test gmock rapid_pr2)
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
