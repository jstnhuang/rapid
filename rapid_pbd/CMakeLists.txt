cmake_minimum_required(VERSION 2.8.3)
project(rapid_pbd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  mongodb_store
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_goal_builder
  pcl_ros
  pr2_controllers_msgs
  pr2_mechanism_msgs
  rapid_pbd_msgs
  robot_markers
  roscpp
  rospy
  surface_perception
  transform_graph
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#   rapid_pbd_msgs
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
    rapid_pbd_action_clients
    rapid_pbd_action_executor
    rapid_pbd_editor
    rapid_pbd_fetch_actions
    rapid_pbd_joint_state
    rapid_pbd_joint_state_reader
    rapid_pbd_motion_planning
    rapid_pbd_pr2_actions
    rapid_pbd_program_db
    rapid_pbd_program_executor
    rapid_pbd_robot_config
    rapid_pbd_step_executor
    rapid_pbd_surface_segmentation_action
    rapid_pbd_visualizer
    rapid_pbd_world
  CATKIN_DEPENDS
    actionlib
    control_msgs
    mongodb_store
    moveit_goal_builder
    pcl_ros
    pr2_controllers_msgs
    pr2_mechanism_msgs
    rapid_pbd_msgs
    robot_markers
    roscpp
    rospy
    surface_perception
    transform_graph
#  DEPENDS system_lib
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

## Libraries
add_library(rapid_pbd_action_clients src/action_clients.cpp)
add_dependencies(rapid_pbd_action_clients ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_action_clients ${catkin_LIBRARIES})

add_library(rapid_pbd_action_executor src/action_executor.cpp)
add_dependencies(rapid_pbd_action_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_action_executor
  rapid_pbd_action_clients
  rapid_pbd_motion_planning
  rapid_pbd_visualizer
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_editor src/editor.cpp)
add_dependencies(rapid_pbd_editor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_editor
  rapid_pbd_action_clients
  rapid_pbd_program_db
  rapid_pbd_joint_state_reader
  rapid_pbd_robot_config
  rapid_pbd_visualizer
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_fetch_actions src/fetch_actions.cpp)
add_dependencies(rapid_pbd_fetch_actions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_fetch_actions ${catkin_LIBRARIES})

add_library(rapid_pbd_joint_state src/joint_state.cpp)
add_dependencies(rapid_pbd_joint_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_joint_state ${catkin_LIBRARIES})

add_library(rapid_pbd_joint_state_reader src/joint_state_reader.cpp)
add_dependencies(rapid_pbd_joint_state_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_joint_state_reader rapid_pbd_joint_state ${catkin_LIBRARIES})

add_library(rapid_pbd_motion_planning src/motion_planning.cpp)
add_dependencies(rapid_pbd_motion_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_motion_planning
  rapid_pbd_robot_config
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_pr2_actions src/pr2_actions.cpp)
add_dependencies(rapid_pbd_pr2_actions ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_pr2_actions ${catkin_LIBRARIES})

add_library(rapid_pbd_program_db src/program_db.cpp)
add_dependencies(rapid_pbd_program_db ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_program_db ${catkin_LIBRARIES})

add_library(rapid_pbd_program_executor src/program_executor.cpp)
add_dependencies(rapid_pbd_program_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_program_executor
  rapid_pbd_action_clients
  rapid_pbd_robot_config
  rapid_pbd_step_executor
  rapid_pbd_visualizer
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_robot_config src/robot_config.cpp)
add_dependencies(rapid_pbd_robot_config ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_robot_config ${catkin_LIBRARIES})

add_library(rapid_pbd_step_executor src/step_executor.cpp)
add_dependencies(rapid_pbd_step_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_step_executor
  rapid_pbd_action_clients
  rapid_pbd_action_executor
  rapid_pbd_motion_planning
  rapid_pbd_robot_config
  rapid_pbd_visualizer
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_surface_segmentation_action src/surface_segmentation_action.cpp)
add_dependencies(rapid_pbd_surface_segmentation_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_surface_segmentation_action
  rapid_pbd_program_db
  rapid_pbd_robot_config
  ${catkin_LIBRARIES})

add_library(rapid_pbd_visualizer src/visualizer.cpp)
add_dependencies(rapid_pbd_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_visualizer
  rapid_pbd_program_db
  rapid_pbd_robot_config
  rapid_pbd_world
  ${catkin_LIBRARIES})

add_library(rapid_pbd_world src/world.cpp)
add_dependencies(rapid_pbd_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_world
  rapid_pbd_joint_state
  rapid_pbd_robot_config
  ${catkin_LIBRARIES})

## Executables
add_executable(rapid_pbd_editor_node src/editor_node.cpp)
set_target_properties(rapid_pbd_editor_node PROPERTIES OUTPUT_NAME editor_node PREFIX "")
add_dependencies(rapid_pbd_editor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_editor_node
  rapid_pbd_editor
  rapid_pbd_program_db
  rapid_pbd_robot_config
  ${catkin_LIBRARIES})

add_executable(rapid_pbd_fetch_action_node src/fetch_action_node.cpp)
set_target_properties(rapid_pbd_fetch_action_node PROPERTIES OUTPUT_NAME fetch_action_node PREFIX "")
add_dependencies(rapid_pbd_fetch_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_fetch_action_node
  rapid_pbd_fetch_actions
  ${catkin_LIBRARIES}
)

add_executable(rapid_pbd_program_execution_demo src/program_execution_demo.cpp)
set_target_properties(rapid_pbd_program_execution_demo PROPERTIES OUTPUT_NAME program_execution_demo PREFIX "")
add_dependencies(rapid_pbd_program_execution_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_program_execution_demo ${catkin_LIBRARIES})

add_executable(rapid_pbd_program_execution_node src/program_execution_node.cpp)
set_target_properties(rapid_pbd_program_execution_node PROPERTIES OUTPUT_NAME program_execution_node PREFIX "")
add_dependencies(rapid_pbd_program_execution_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_program_execution_node
  rapid_pbd_program_executor
  rapid_pbd_robot_config
  rapid_pbd_visualizer
  ${catkin_LIBRARIES}
)

add_executable(rapid_pbd_pr2_action_node src/pr2_action_node.cpp)
set_target_properties(rapid_pbd_pr2_action_node PROPERTIES OUTPUT_NAME pr2_action_node PREFIX "")
add_dependencies(rapid_pbd_pr2_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_pr2_action_node
  rapid_pbd_pr2_actions
  ${catkin_LIBRARIES}
)

add_executable(rapid_pbd_surface_segmentation_node src/surface_segmentation_node.cpp)
set_target_properties(rapid_pbd_surface_segmentation_node PROPERTIES OUTPUT_NAME surface_segmentation_node PREFIX "")
add_dependencies(rapid_pbd_surface_segmentation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_surface_segmentation_node
  rapid_pbd_program_db
  rapid_pbd_robot_config
  rapid_pbd_surface_segmentation_action
  ${catkin_LIBRARIES}
)

add_executable(rapid_pbd_pr2_gripper_demo src/pr2_gripper_demo.cpp)
set_target_properties(rapid_pbd_pr2_gripper_demo PROPERTIES OUTPUT_NAME pr2_gripper_demo PREFIX "")
add_dependencies(rapid_pbd_pr2_gripper_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_pr2_gripper_demo
  ${catkin_LIBRARIES}
)

add_executable(rapid_pbd_pr2_arm_joint_demo src/pr2_arm_joint_demo.cpp)
set_target_properties(rapid_pbd_pr2_arm_joint_demo PROPERTIES OUTPUT_NAME pr2_arm_joint_demo PREFIX "")
add_dependencies(rapid_pbd_pr2_arm_joint_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rapid_pbd_pr2_arm_joint_demo
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
# install(TARGETS rapid_pbd rapid_pbd_node
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rapid_pbd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
