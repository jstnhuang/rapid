#include "rapid_pbd/landmarks.h"

#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"

namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
void ProcessSurfaceBox(const rapid_pbd_msgs::Landmark& landmark_in,
                       rapid_pbd_msgs::Landmark* landmark_out) {
  *landmark_out = landmark_in;
  if (landmark_in.type == msgs::Landmark::SURFACE_BOX) {
    ROS_ERROR("Called ProcessSurfaceBox with non-surface box landmark.");
    return;
  }
  double x = landmark_in.surface_box_dims.x;
  double y = landmark_in.surface_box_dims.y;
  if (x / y > 1) {
    ROS_ERROR("surface_perception box has x > y!");
    return;
  }
  if (y != 0 && x / y > 0.95) {
    landmark_out->pose_stamped.pose.orientation.w = 1;
    landmark_out->pose_stamped.pose.orientation.x = 0;
    landmark_out->pose_stamped.pose.orientation.y = 0;
    landmark_out->pose_stamped.pose.orientation.z = 0;
  }
}
}  // namespace pbd
}  // namespace rapid
