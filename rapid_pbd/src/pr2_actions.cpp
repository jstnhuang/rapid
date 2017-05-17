#include "rapid_pbd/pr2_actions.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "pr2_mechanism_msgs/ListControllers.h"
#include "pr2_mechanism_msgs/SwitchController.h"

#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"

using actionlib::SimpleClientGoalState;
using pr2_controllers_msgs::Pr2GripperCommandFeedback;
using pr2_controllers_msgs::Pr2GripperCommandResult;
namespace msgs = rapid_pbd_msgs;

namespace rapid {
namespace pbd {
namespace pr2 {
GripperAction::GripperAction(const std::string& action_name,
                             const std::string& pr2_action_name)
    : server_(action_name, boost::bind(&GripperAction::Execute, this, _1),
              false),
      pr2_client_(pr2_action_name, true) {}

void GripperAction::Start() {
  while (!pr2_client_.waitForServer(ros::Duration(5))) {
    ROS_WARN("Waiting for PR2 gripper server to come up.");
  }
  server_.start();
}

void GripperAction::Execute(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  pr2_controllers_msgs::Pr2GripperCommandGoal pr2_goal;
  pr2_goal.command.position = goal->command.position;
  pr2_goal.command.max_effort = goal->command.max_effort;
  pr2_client_.sendGoal(
      pr2_goal,
      boost::function<void(const SimpleClientGoalState&,
                           const Pr2GripperCommandResult::ConstPtr&)>(),
      boost::function<void()>(),
      boost::bind(&GripperAction::HandleFeedback, this, _1));
  while (!pr2_client_.getState().isDone()) {
    if (server_.isPreemptRequested() || !ros::ok()) {
      pr2_client_.cancelAllGoals();
      server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (pr2_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    pr2_client_.cancelAllGoals();
    server_.setPreempted();
    return;
  } else if (pr2_client_.getState() == SimpleClientGoalState::ABORTED) {
    pr2_client_.cancelAllGoals();
    server_.setAborted();
    return;
  }

  Pr2GripperCommandResult::ConstPtr pr2_result = pr2_client_.getResult();
  control_msgs::GripperCommandResult result;
  result.effort = pr2_result->effort;
  result.position = pr2_result->position;
  result.reached_goal = pr2_result->reached_goal;
  result.stalled = pr2_result->stalled;
  server_.setSucceeded(result);
}

void GripperAction::HandleFeedback(
    const Pr2GripperCommandFeedback::ConstPtr& pr2_feedback) {
  control_msgs::GripperCommandFeedback feedback;
  feedback.effort = pr2_feedback->effort;
  feedback.position = pr2_feedback->position;
  feedback.reached_goal = pr2_feedback->reached_goal;
  feedback.stalled = pr2_feedback->stalled;
  server_.publishFeedback(feedback);
}

ArmControllerManager::ArmControllerManager(
    const ros::Publisher& state_pub, const ros::ServiceClient& list_client,
    const ros::ServiceClient& switch_client)
    : state_pub_(state_pub),
      list_client_(list_client),
      switch_client_(switch_client),
      is_l_arm_active_(true),
      is_r_arm_active_(true) {}

void ArmControllerManager::Start() { Update(); }

bool ArmControllerManager::HandleFreeze(msgs::FreezeArmRequest& request,
                                        msgs::FreezeArmResponse& response) {
  pr2_mechanism_msgs::SwitchControllerRequest req;
  req.strictness = pr2_mechanism_msgs::SwitchControllerRequest::BEST_EFFORT;
  pr2_mechanism_msgs::SwitchControllerResponse res;
  if (request.actuator_group == msgs::Action::LEFT_ARM) {
    req.start_controllers.push_back("l_arm_controller");
  } else if (request.actuator_group == msgs::Action::RIGHT_ARM) {
    req.start_controllers.push_back("r_arm_controller");
  } else {
    response.error =
        "Invalid actuator group \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  bool success = switch_client_.call(req, res);
  if (!success) {
    response.error = "Failed to freeze \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();

  return true;
}

bool ArmControllerManager::HandleRelax(msgs::RelaxArmRequest& request,
                                       msgs::RelaxArmResponse& response) {
  pr2_mechanism_msgs::SwitchControllerRequest req;
  req.strictness = pr2_mechanism_msgs::SwitchControllerRequest::BEST_EFFORT;
  pr2_mechanism_msgs::SwitchControllerResponse res;
  if (request.actuator_group == msgs::Action::LEFT_ARM) {
    req.stop_controllers.push_back("l_arm_controller");
  } else if (request.actuator_group == msgs::Action::RIGHT_ARM) {
    req.stop_controllers.push_back("r_arm_controller");
  } else {
    response.error =
        "Invalid actuator group \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  bool success = switch_client_.call(req, res);
  if (!success) {
    response.error = "Failed to relax \"" + request.actuator_group + "\"";
    ROS_ERROR("%s", response.error.c_str());
    return true;
  }
  Update();

  return true;
}

void ArmControllerManager::Update() {
  pr2_mechanism_msgs::ListControllersRequest req;
  pr2_mechanism_msgs::ListControllersResponse res;
  while (!list_client_.waitForExistence(ros::Duration(5))) {
    ROS_WARN("Waiting for pr2_controller_manager list service...");
  }
  bool success = list_client_.call(req, res);
  if (!success) {
    ROS_ERROR("pr2_controller_manager list service call failed.");
  }
  for (size_t i = 0; i < res.controllers.size(); ++i) {
    const std::string& name = res.controllers[i];
    const std::string& state = res.state[i];
    bool is_running = (state == "running");
    if (name == "l_arm_controller") {
      is_l_arm_active_ = is_running;
    } else if (name == "r_arm_controller") {
      is_r_arm_active_ = is_running;
    }
  }

  msgs::ArmControllerState msg;
  if (is_l_arm_active_) {
    msg.l_arm_controller = msgs::ArmControllerState::FROZEN;
  } else {
    msg.l_arm_controller = msgs::ArmControllerState::RELAXED;
  }
  if (is_r_arm_active_) {
    msg.r_arm_controller = msgs::ArmControllerState::FROZEN;
  } else {
    msg.r_arm_controller = msgs::ArmControllerState::RELAXED;
  }
  state_pub_.publish(msg);
}
}  // namespace pr2
}  // namespace pbd
}  // namespace rapid
