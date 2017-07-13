#include "rapid_pbd/step_executor.h"

#include "boost/shared_ptr.hpp"
#include "moveit_msgs/MoveGroupGoal.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Step.h"

#include "rapid_pbd/action_executor.h"
#include "rapid_pbd/world.h"

using boost::shared_ptr;
using rapid_pbd_msgs::Action;
using rapid_pbd_msgs::Step;

namespace rapid {
namespace pbd {
StepExecutor::StepExecutor(const rapid_pbd_msgs::Step& step,
                           ActionClients* action_clients,
                           const RobotConfig& robot_config, World* world)
    : step_(step),
      action_clients_(action_clients),
      world_(world),
      motion_planning_(robot_config, world),
      executors_() {}

bool StepExecutor::IsValid(const rapid_pbd_msgs::Step& step) {
  for (size_t i = 0; i < step.actions.size(); ++i) {
    const Action& action = step.actions[i];
    if (!ActionExecutor::IsValid(action)) {
      ROS_ERROR("Action type %s invalid in step %ld", action.type.c_str(), i);
      return false;
    }
  }
  return true;
}

void StepExecutor::Init() {
  for (size_t i = 0; i < step_.actions.size(); ++i) {
    Action action = step_.actions[i];
    shared_ptr<ActionExecutor> ae(
        new ActionExecutor(action, action_clients_, &motion_planning_, world_));
    executors_.push_back(ae);
  }
}

std::string StepExecutor::Start() {
  motion_planning_.ClearGoals();
  std::string error("");
  for (size_t i = 0; i < step_.actions.size(); ++i) {
    error = executors_[i]->Start();
    if (error != "") {
      return error;
    }
  }
  if (motion_planning_.num_goals() > 0) {
    moveit_msgs::MoveGroupGoal goal;
    motion_planning_.BuildGoal(&goal);
    action_clients_->moveit_client.sendGoal(goal);
  }
  return "";
}

bool StepExecutor::IsDone(std::string* error) const {
  for (size_t i = 0; i < executors_.size(); ++i) {
    const shared_ptr<ActionExecutor>& executor = executors_[i];
    if (!executor->IsDone(error)) {
      return false;
    }
    if (*error != "") {
      return true;
    }
  }
  if (motion_planning_.num_goals() > 0) {
    if (!action_clients_->moveit_client.getState().isDone()) {
      return false;
    }
  }
  return true;
}

void StepExecutor::Cancel() {
  for (size_t i = 0; i < executors_.size(); ++i) {
    const shared_ptr<ActionExecutor>& executor = executors_[i];
    executor->Cancel();
  }
  if (motion_planning_.num_goals() > 0) {
    action_clients_->moveit_client.cancelAllGoals();
    motion_planning_.ClearGoals();
  }

  executors_.clear();
}

}  // namespace pbd
}  // namespace rapid
