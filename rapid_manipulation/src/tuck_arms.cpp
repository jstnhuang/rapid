#include "rapid_manipulation/tuck_arms.h"

#include "actionlib/client/simple_action_client.h"
#include "pr2_common_action_msgs/TuckArmsAction.h"
#include "ros/ros.h"

using actionlib::SimpleActionClient;
using actionlib::SimpleClientGoalState;
using pr2_common_action_msgs::TuckArmsAction;
using pr2_common_action_msgs::TuckArmsGoal;

namespace rapid {
namespace manipulation {
Pr2TuckArms::Pr2TuckArms() : client_("tuck_arms", true), server_wait_time_(5) {}

bool Pr2TuckArms::TuckArms() { return ExecuteAction(true, true); }
bool Pr2TuckArms::DeployLeft() { return ExecuteAction(false, true); }
bool Pr2TuckArms::DeployRight() { return ExecuteAction(true, false); }
bool Pr2TuckArms::DeployArms() { return ExecuteAction(false, false); }

bool Pr2TuckArms::ExecuteAction(bool tuck_left, bool tuck_right) {
  client_.waitForServer(ros::Duration(server_wait_time_));
  TuckArmsGoal goal;
  goal.tuck_left = tuck_left;
  goal.tuck_right = tuck_right;
  client_.sendGoal(goal);
  bool success = client_.waitForResult(ros::Duration(server_wait_time_));
  if (!success) {
    ROS_ERROR("Tuck arms server didn't return result after %f seconds.",
              server_wait_time_);
    return false;
  }
  SimpleClientGoalState state = client_.getState();
  if (state != SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN("Tuck arms action ended in state %s", state.toString().c_str());
  }
  return true;
}
}  //  namespace manipulation
}  // namespace rapid
