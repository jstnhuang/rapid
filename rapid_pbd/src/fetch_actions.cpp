#include "rapid_pbd/fetch_actions.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"
#include "robot_controllers_msgs/ControllerState.h"
#include "robot_controllers_msgs/QueryControllerStatesAction.h"

namespace msgs = rapid_pbd_msgs;
using robot_controllers_msgs::ControllerState;
using robot_controllers_msgs::QueryControllerStatesGoal;
using robot_controllers_msgs::QueryControllerStatesResult;
typedef actionlib::SimpleActionClient<
    robot_controllers_msgs::QueryControllerStatesAction>
    ControllerClient;

namespace rapid {
namespace pbd {
namespace fetch {
ArmControllerManager::ArmControllerManager(const ros::Publisher& state_pub,
                                           ControllerClient* client)
    : state_pub_(state_pub), client_(client), is_arm_active_(true) {}

void ArmControllerManager::Start() { Update(); }

bool ArmControllerManager::HandleFreeze(msgs::FreezeArmRequest& request,
                                        msgs::FreezeArmResponse& response) {
  QueryControllerStatesGoal goal;
  ControllerState state;
  state.name = "arm_controller/follow_joint_trajectory";
  state.state = ControllerState::RUNNING;
  goal.updates.push_back(state);

  client_->sendGoal(goal);
  if (!client_->waitForResult(ros::Duration(5.0))) {
    response.error = "Failed to freeze arm.";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  QueryControllerStatesResult::ConstPtr result = client_->getResult();
  if (!result) {
    response.error = "Got null result when freezing arm.";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();
  return true;
}

bool ArmControllerManager::HandleRelax(msgs::RelaxArmRequest& request,
                                       msgs::RelaxArmResponse& response) {
  QueryControllerStatesGoal goal;
  ControllerState state;
  state.name = "arm_controller/follow_joint_trajectory";
  state.state = ControllerState::STOPPED;
  goal.updates.push_back(state);

  client_->sendGoal(goal);
  if (!client_->waitForResult(ros::Duration(5.0))) {
    response.error = "Failed to relax arm.";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  QueryControllerStatesResult::ConstPtr result = client_->getResult();
  if (!result) {
    response.error = "Got null result when relaxing arm.";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();
  return true;
}

void ArmControllerManager::Update() {
  QueryControllerStatesGoal goal;
  client_->sendGoal(goal);
  if (!client_->waitForResult(ros::Duration(5.0))) {
    ROS_ERROR("Failed to query arm controller state!");
    return;
  }
  QueryControllerStatesResult::ConstPtr result = client_->getResult();
  if (!result) {
    ROS_ERROR("Got null result when querying arm controller state!");
    return;
  }

  msgs::ArmControllerState msg;
  for (size_t i = 0; i < result->state.size(); ++i) {
    const ControllerState& state = result->state[i];
    if (state.name == "arm_controller/follow_joint_trajectory") {
      if (state.state == ControllerState::STOPPED) {
        is_arm_active_ = false;
        msg.arm_controller = msgs::ArmControllerState::RELAXED;
        state_pub_.publish(msg);
        return;
      } else if (state.state == ControllerState::RUNNING) {
        is_arm_active_ = true;
        msg.arm_controller = msgs::ArmControllerState::FROZEN;
        state_pub_.publish(msg);
        return;
      }
    }
  }
}
}  // namespace fetch
}  // namespace pbd
}  // namespace rapid
