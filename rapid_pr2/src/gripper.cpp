#include "rapid_pr2/gripper.h"

#include "actionlib/client/simple_action_client.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"

typedef actionlib::SimpleActionClient<
    pr2_controllers_msgs::Pr2GripperCommandAction>
    GripperClient;

namespace rapid {
namespace pr2 {
Gripper Gripper::Left() { return Gripper(true); }

Gripper Gripper::Right() { return Gripper(false); }

bool Gripper::StartOpening() {
  if (!client_->waitForServer(ros::Duration(1.0))) {
    ROS_ERROR("Unable to connect to gripper server.");
    return false;
  }
  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = 0.086;
  goal.command.max_effort = -1;
  client_->sendGoal(goal);
  return true;
}

bool Gripper::StartClosing() { return StartClosing(-1); }

bool Gripper::StartClosing(double max_effort) {
  if (!client_->waitForServer(ros::Duration(1.0))) {
    ROS_ERROR("Unable to connect to gripper server.");
    return false;
  }
  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = 0;
  goal.command.max_effort = max_effort;
  client_->sendGoal(goal);
  return true;
}

bool Gripper::IsDone() const { return client_->getState().isDone(); }

void Gripper::Cancel() { client_->cancelAllGoals(); }

Gripper::Gripper(bool is_left)
    : client_(new GripperClient(
          is_left ? kLeftGripperAction : kRightGripperAction, true)) {}
}  // namespace pr2
}  // namespace rapid
