cmake_minimum_required(VERSION 2.8.3)
project(rapid_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_geometry
  interactive_markers
  pcl_conversions
  pcl_ros
  rapid_msgs
  rapid_ros
  rapid_utils
  rapid_viz
  rosbag
  roscpp
  rostest
  sensor_msgs
  visualization_msgs
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Caffe
set(CAFFE_INSTALL_DIR /usr/local/caffe)
set(CUDA_INSTALL_DIR /usr/local/cuda)

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
set(MY_RAPID_PERCEPTION_EXPORTED_LIBRARIES
  rapid_perception_box3d_roi_server
  rapid_perception_cloud_projection
  rapid_perception_icp_fitness_functions
  rapid_perception_pose_estimation
  rapid_perception_pose_estimation_heat_mapper
  rapid_perception_fpfh_heat_mapper
  rapid_perception_template_matching_heat_mapper
  rapid_perception_random_heat_mapper
  rapid_perception_pr2
  rapid_perception_rgbd
  rapid_perception_scene
  rapid_perception_scene_parsing
  rapid_perception_scene_viz
)

if (${CAFFE_INSTALL_DIR})
  LIST(APPEND MY_RAPID_PERCEPTION_EXPORTED_LIBRARIES
    rapid_perception_image_recognition
    rapid_perception_pose_estimation_cnn_heat_mapper
  )
endif()

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${MY_RAPID_PERCEPTION_EXPORTED_LIBRARIES}
  CATKIN_DEPENDS
    geometry_msgs
    interactive_markers
    pcl_conversions
    pcl_ros
    rapid_msgs
    rapid_ros
    rapid_utils
    rapid_viz
    rosbag
    roscpp
    sensor_msgs
    visualization_msgs
  DEPENDS
    Boost
    Eigen
    OpenCV
    PCL
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
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(rapid_perception_box3d_roi_server src/box3d_roi_server.cpp)
add_dependencies(rapid_perception_box3d_roi_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_box3d_roi_server
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_cloud_projection src/cloud_projection.cpp)
add_dependencies(rapid_perception_cloud_projection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_cloud_projection
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_icp_fitness_functions src/icp_fitness_functions.cpp)
add_dependencies(rapid_perception_icp_fitness_functions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_icp_fitness_functions
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# To use the image recognition library, we must have Caffe installed to /usr/local
# To build with GPU support, we must have Cuda installed to /usr/local/cuda
if (${CAFFE_INSTALL_DIR})
  include_directories(${CAFFE_INSTALL_DIR}/include)
  link_directories(${CAFFE_INSTALL_DIR}/lib)
  find_library(caffe libcaffe.so ${CAFFE_INSTALL_DIR}/lib)
  if (${CUDA_INSTALL_DIR})
    include_directories(${CUDA_INSTALL_DIR}/include)
  else()
    add_definitions(-DCPU_ONLY)
  endif()

  add_library(rapid_perception_image_recognition
    src/image_recognition.cpp
  )
  add_dependencies(rapid_perception_image_recognition
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(rapid_perception_image_recognition
    ${catkin_LIBRARIES}
    caffe
  )
endif()

add_library(rapid_perception_pose_estimation_heat_mapper
  src/pose_estimation_heat_mapper.cpp
)
add_dependencies(rapid_perception_pose_estimation_heat_mapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_pose_estimation_heat_mapper
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

if (${CAFFE_INSTALL_DIR})
  include_directories(${CAFFE_INSTALL_DIR}/include)
  link_directories(${CAFFE_INSTALL_DIR}/lib)
  find_library(caffe libcaffe.so ${CAFFE_INSTALL_DIR}/lib)
  if (${CUDA_INSTALL_DIR})
    include_directories(${CUDA_INSTALL_DIR}/include)
  else()
    add_definitions(-DCPU_ONLY)
  endif()

  add_library(rapid_perception_pose_estimation_cnn_heat_mapper
    src/pose_estimation_cnn_heat_mapper.cpp
  )
  add_dependencies(rapid_perception_pose_estimation_cnn_heat_mapper
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    rapid_perception_cloud_projection
    rapid_perception_image_recognition
    rapid_perception_pose_estimation_heat_mapper
  )
  target_link_libraries(rapid_perception_pose_estimation_cnn_heat_mapper
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    caffe
    rapid_perception_cloud_projection
    rapid_perception_image_recognition
    rapid_perception_pose_estimation_heat_mapper
  )
endif()

add_library(rapid_perception_fpfh_heat_mapper
  src/fpfh_heat_mapper.cpp
)
add_dependencies(rapid_perception_fpfh_heat_mapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rapid_perception_pose_estimation_heat_mapper
)
target_link_libraries(rapid_perception_fpfh_heat_mapper
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  rapid_perception_pose_estimation_heat_mapper
)

add_library(rapid_perception_template_matching_heat_mapper
  src/template_matching_heat_mapper.cpp
)
add_dependencies(rapid_perception_template_matching_heat_mapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rapid_perception_pose_estimation_heat_mapper
)
target_link_libraries(rapid_perception_template_matching_heat_mapper
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  rapid_perception_pose_estimation_heat_mapper
)

add_library(rapid_perception_random_heat_mapper
  src/random_heat_mapper.cpp
)
add_dependencies(rapid_perception_random_heat_mapper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rapid_perception_pose_estimation_heat_mapper
)
target_link_libraries(rapid_perception_random_heat_mapper
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  rapid_perception_pose_estimation_heat_mapper
)

add_library(rapid_perception_pose_estimation
  src/pose_estimation.cpp
)
add_dependencies(rapid_perception_pose_estimation
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  rapid_perception_icp_fitness_functions
  rapid_perception_random_heat_mapper
)
target_link_libraries(rapid_perception_pose_estimation
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  rapid_perception_icp_fitness_functions
  rapid_perception_random_heat_mapper
)

add_library(rapid_perception_pr2 src/pr2.cpp)
add_dependencies(rapid_perception_pr2
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_pr2
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_scene
  src/object.cpp
  src/hsurface.cpp
  src/scene.cpp
)
add_dependencies(rapid_perception_scene
  rapid_perception_rgbd
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_scene
  rapid_perception_rgbd
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_scene_parsing
  src/scene_parsing.cpp
)
add_dependencies(rapid_perception_scene_parsing
  rapid_perception_rgbd
  rapid_perception_scene
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_scene_parsing
  rapid_perception_rgbd
  rapid_perception_scene
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_scene_viz
  src/scene_viz.cpp
)
add_dependencies(rapid_perception_scene_viz
  rapid_perception_scene
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_scene_viz
  rapid_perception_scene
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(rapid_perception_rgbd src/rgbd.cpp)
add_dependencies(rapid_perception_rgbd
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rapid_perception_rgbd
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(record_kinect src/record_kinect_main.cpp)
add_dependencies(record_kinect
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(record_kinect
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(debug_obj_pipeline src/debug_obj_pipeline_main.cpp)
add_dependencies(debug_obj_pipeline
  rapid_perception_pr2
  rapid_perception_rgbd
  rapid_perception_scene
  rapid_perception_scene_parsing
  rapid_perception_scene_viz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(debug_obj_pipeline
  rapid_perception_pr2
  rapid_perception_rgbd
  rapid_perception_scene
  rapid_perception_scene_parsing
  rapid_perception_scene_viz
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against

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
# install(TARGETS rapid_perception rapid_perception_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
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

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rapid_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage") # Enable code coverage
  catkin_add_gtest(rapid_rgbd_test test/rgbd_test.cpp)
  target_link_libraries(rapid_rgbd_test
    rapid_perception_rgbd
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(rapid_scene_test test/scene_test.cpp)
  target_link_libraries(rapid_scene_test
    rapid_perception_scene
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(rapid_scene_parsing_test test/scene_parsing_test.cpp)
  target_link_libraries(rapid_scene_parsing_test
    rapid_perception_scene_parsing
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(rapid_scene_viz_test test/scene_viz_test.cpp)
  target_link_libraries(rapid_scene_viz_test
    rapid_perception_scene_viz
    ${catkin_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
