#include "rapid_pbd/landmarks.h"

#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void ProcessSurfaceBox(const rapid_pbd_msgs::Landmark& landmark_in,
                       rapid_pbd_msgs::Landmark* landmark_out) {
  *landmark_out = landmark_in;
  if (landmark_in.type != msgs::Landmark::SURFACE_BOX) {
    ROS_ERROR("Called ProcessSurfaceBox with non-surface box landmark.");
    return;
  }
  double x = landmark_in.surface_box_dims.x;
  double y = landmark_in.surface_box_dims.y;
  if (y == 0) {
    ROS_ERROR("Surface box y dimension is 0!");
    return;
  }
  if (x / y > 1) {
    ROS_ERROR("surface_perception box has x > y!");
    return;
  }
  double cylinder_ratio;
  ros::param::param<double>("cylinder_ratio", cylinder_ratio, 0.85);
  if (x / y > cylinder_ratio) {
    landmark_out->pose_stamped.pose.orientation.w = 1;
    landmark_out->pose_stamped.pose.orientation.x = 0;
    landmark_out->pose_stamped.pose.orientation.y = 0;
    landmark_out->pose_stamped.pose.orientation.z = 0;
  }
}
}  // namespace pbd
}  // namespace rapid
