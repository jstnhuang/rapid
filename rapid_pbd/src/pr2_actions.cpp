#include "rapid_pbd/pr2_actions.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

using actionlib::SimpleClientGoalState;
using pr2_controllers_msgs::Pr2GripperCommandFeedback;
using pr2_controllers_msgs::Pr2GripperCommandResult;

namespace rapid {
namespace pbd {
namespace pr2 {
GripperAction::GripperAction(const std::string& action_name,
                             const std::string& pr2_action_name)
    : nh_(),
      server_(nh_, action_name, boost::bind(&GripperAction::Execute, this, _1),
              false),
      pr2_client_(nh_, pr2_action_name, true) {}

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
    if (server_.isPreemptRequested()) {
      pr2_client_.cancelAllGoals();
      server_.setPreempted();
      return;
    }
    ros::spinOnce();
  }
  if (pr2_client_.getState() == SimpleClientGoalState::PREEMPTED) {
    pr2_client_.cancelAllGoals();
    server_.setPreempted();
  } else if (pr2_client_.getState() == SimpleClientGoalState::ABORTED) {
    pr2_client_.cancelAllGoals();
    server_.setAborted();
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
}  // namespace pr2
}  // namespace pbd
}  // namespace rapid
