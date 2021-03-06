cmake_minimum_required(VERSION 2.8.3)
project(rapid_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  pr2_common_action_msgs
  pr2_controllers_msgs
  rapid_perception
  rapid_ros
  rapid_utils
  roscpp
  sensor_msgs
  shape_msgs
  shape_tools
  tf
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

set(gmock_INCLUDE_DIR /usr/include/gmock)
set(gmock_LIB_DIR /usr/lib)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
    rapid_manipulation
    rapid_manipulation_moveit_error_code
    rapid_manipulation_moveit_planning_scene
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    pr2_common_action_msgs
    pr2_controllers_msgs
    rapid_perception
    rapid_ros
    rapid_utils
    roscpp
    rostest
    shape_msgs
    sensor_msgs
    tf
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

## Declare a C++ library
add_library(rapid_manipulation
  src/arm.cpp
  src/gripper.cpp
  src/head.cpp
  src/tuck_arms.cpp
)
add_dependencies(rapid_manipulation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_manipulation
  gmock
  ${catkin_LIBRARIES}
)

add_library(rapid_manipulation_moveit_error_code src/moveit_error_code.cpp)
add_dependencies(rapid_manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_manipulation
  ${catkin_LIBRARIES}
)

add_library(rapid_manipulation_moveit_planning_scene src/moveit_planning_scene.cpp)
add_dependencies(rapid_manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_manipulation
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
# add_executable(rapid_manipulation_node src/rapid_manipulation_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rapid_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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
# install(TARGETS rapid_manipulation rapid_manipulation_node
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
  add_rostest_gtest(rapid_manipulation_gripper_test
    test/gripper_test.launch
    test/gripper_test.cpp
  )
  target_link_libraries(rapid_manipulation_gripper_test
    rapid_manipulation
    ${catkin_LIBRARIES}
  )
  add_rostest_gtest(rapid_manipulation_head_test
    test/head_test.launch
    test/head_test.cpp
  )
  target_link_libraries(rapid_manipulation_head_test
    rapid_manipulation
    ${catkin_LIBRARIES}
  )
  add_rostest_gtest(rapid_manipulation_tuck_arms_test
    test/tuck_arms_test.launch
    test/tuck_arms_test.cpp
  )
  target_link_libraries(rapid_manipulation_tuck_arms_test
    rapid_manipulation
    ${catkin_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
