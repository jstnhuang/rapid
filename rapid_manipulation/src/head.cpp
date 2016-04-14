#include "rapid_manipulation/head.h"

#include <math.h>

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "pr2_controllers_msgs/PointHeadGoal.h"

using pr2_controllers_msgs::PointHeadAction;
using rapid_ros::ActionClientInterface;

namespace rapid {
namespace manipulation {
Head::Head(ActionClientInterface<PointHeadAction>* client) : client_(client) {}

bool Head::LookAt(const geometry_msgs::PointStamped& target) {
  if (!client_->waitForServer(ros::Duration(5))) {
    ROS_ERROR("The head action server was not available after 5 seconds!");
    return false;
  }
  pr2_controllers_msgs::PointHeadGoal goal;
  goal.target = target;
  // If frame_id is empty, use a "head" frame. I think I used torso_lift_link
  // because none of the other head frames have the right orientation.
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
  client_->sendGoal(goal);
  return client_->waitForResult(ros::Duration(10));
}
}  // namespace manipulation
}  // namespace rapid
