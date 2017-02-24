#include "rapid_fetch/gripper.h"

#include "control_msgs/GripperCommandAction.h"
#include "ros/ros.h"

namespace rapid {
namespace fetch {
const char Gripper::kGripperAction[] = "gripper_controller/gripper_action";
const double Gripper::kClosedPosition = 0;
const double Gripper::kOpenedPosition = 0.115;

Gripper::Gripper() : client_(kGripperAction, true) {
  if (client_.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Gripper server \"%s\" may not be connected.", kGripperAction);
  }
}

bool Gripper::Close(double max_effort) {
  control_msgs::GripperCommandGoal goal;
  goal.command.position = kClosedPosition;
  goal.command.max_effort = max_effort;
  client_.sendGoal(goal);
  return client_.waitForResult(ros::Duration(5.0));
}

bool Gripper::Open(double max_effort) {
  control_msgs::GripperCommandGoal goal;
  goal.command.position = kOpenedPosition;
  goal.command.max_effort = max_effort;
  client_.sendGoal(goal);
  return client_.waitForResult(ros::Duration(5.0));
}
}  // namespace fetch
}  // namespace rapid
