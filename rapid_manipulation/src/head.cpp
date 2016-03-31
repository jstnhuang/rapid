#include "rapid_manipulation/head.h"

#include <math.h>

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "pr2_controllers_msgs/PointHeadGoal.h"

namespace rapid {
namespace manipulation {
Head::Head() : head_client_("/head_traj_controller/point_head_action") {}

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
}  // namespace manipulation
}  // namespace rapid
