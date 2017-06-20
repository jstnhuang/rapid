#include <string>

#include "rapid_pbd/surface_segmentation_action.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "surface_segmentation_action_node");
  if (argc < 2) {
    ROS_ERROR("Must supply topic as arg");
    return 1;
  }
  std::string topic(argv[1]);
  rapid::pbd::SurfaceSegmentationAction action(topic);
  ros::spin();
  return 0;
}
