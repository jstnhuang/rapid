#include "rapid/pr2/head.h"

#include <math.h>

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "pr2_controllers_msgs/PointHeadGoal.h"

namespace rapid {
namespace pr2 {
// void ComputeLookAtPointFromDegrees(const double up, const double left,
//                                   geometry_msgs::Point* point) {
//  const static double kPi = 3.1415926535;
//  const static double kDegsToRads = kPi / 180;
//  point->x = cos(left * kDegsToRads);
//  point->y = sin(left * kDegsToRads);
//  point->z = sin(up * kDegsToRads);
//}

Head::Head() : head_client_("/head_traj_controller/point_head_action") {}

// bool Head::LookAtDegrees(double up, double left) {
//  geometry_msgs::PointStamped point_stamped;
//  point_stamped.header.frame_id = "head_mount_kinect_rgb_link";
//  geometry_msgs::Point point;
//  ComputeLookAtPointFromDegrees(up, left, &point);
//  return LookAt(point_stamped);
//}

bool Head::LookAt(const geometry_msgs::PointStamped& target) {
  pr2_controllers_msgs::PointHeadGoal goal;
  goal.target = target;
  if (goal.target.header.frame_id == "") {
    goal.target.header.frame_id = "torso_lift_link";
    goal.target.point.z += 0.65;
  }
  goal.pointing_frame = "head_mount_kinect_rgb_link";
  goal.pointing_axis.x = 1;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;
  goal.min_duration = ros::Duration(0);
  goal.max_velocity = 1.0;
  head_client_.sendGoal(goal);
  return head_client_.waitForResult(ros::Duration(10));
}
}  // namespace pr2
}  // namespace rapid
