#include "rapid_manipulation/tuck_arms.h"

#include "actionlib/client/simple_client_goal_state.h"
#include "pr2_common_action_msgs/TuckArmsAction.h"
#include "ros/ros.h"

#include "rapid_ros/action_client.h"

using actionlib::SimpleClientGoalState;
using pr2_common_action_msgs::TuckArmsAction;
using pr2_common_action_msgs::TuckArmsGoal;
using rapid_ros::ActionClientInterface;

namespace rapid {
namespace manipulation {
Pr2TuckArms::Pr2TuckArms(ActionClientInterface<TuckArmsAction>* client)
    : client_(client), server_wait_time_(10) {}

bool Pr2TuckArms::TuckArms() { return ExecuteAction(true, true); }
bool Pr2TuckArms::DeployLeft() { return ExecuteAction(false, true); }
bool Pr2TuckArms::DeployRight() { return ExecuteAction(true, false); }
bool Pr2TuckArms::DeployArms() { return ExecuteAction(false, false); }

bool Pr2TuckArms::ExecuteAction(bool tuck_left, bool tuck_right) {
  bool success = client_->waitForServer(ros::Duration(server_wait_time_));
  if (!success) {
    ROS_ERROR("Didn't connect to tuck arms server after %f seconds.",
              server_wait_time_);
    return false;
  }
  TuckArmsGoal goal;
  goal.tuck_left = tuck_left;
  goal.tuck_right = tuck_right;
  client_->sendGoal(goal);
  success = client_->waitForResult(ros::Duration(server_wait_time_));
  if (!success) {
    ROS_ERROR("Tuck arms server didn't return result after %f seconds.",
              server_wait_time_);
    return false;
  }
  SimpleClientGoalState state = client_->getState();
  if (state != SimpleClientGoalState::SUCCEEDED) {
    ROS_WARN("Tuck arms action ended in state %s", state.toString().c_str());
  }
  return true;
}

void Pr2TuckArms::set_server_wait_time(double server_wait_time) {
  server_wait_time_ = server_wait_time;
}
}  //  namespace manipulation
}  // namespace rapid
