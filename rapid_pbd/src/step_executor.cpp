#include "rapid_pbd/step_executor.h"

#include <sstream>

#include "boost/shared_ptr.hpp"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/Step.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/action_executor.h"
#include "rapid_pbd/errors.h"
#include "rapid_pbd/motion_planning.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

using boost::shared_ptr;
using rapid_pbd_msgs::Action;
using rapid_pbd_msgs::Step;

namespace rapid {
namespace pbd {
StepExecutor::StepExecutor(const rapid_pbd_msgs::Step& step,
                           ActionClients* action_clients,
                           const RobotConfig& robot_config, World* world,
                           const RuntimeVisualizer& runtime_viz,
                           const tf::TransformListener& tf_listener,
                           const ros::Publisher& planning_scene_pub,
                           const JointStateReader& js_reader)
    : step_(step),
      action_clients_(action_clients),
      robot_config_(robot_config),
      world_(world),
      runtime_viz_(runtime_viz),
      motion_planning_(robot_config, world, tf_listener, planning_scene_pub,
                       js_reader),
      executors_() {}

bool StepExecutor::IsValid(const rapid_pbd_msgs::Step& step) {
  for (size_t i = 0; i < step.actions.size(); ++i) {
    const Action& action = step.actions[i];
    if (!ActionExecutor::IsValid(action)) {
      ROS_ERROR("Action type %s invalid in action %ld", action.type.c_str(), i);
      return false;
    }
  }
  return true;
}

void StepExecutor::Init() {
  for (size_t i = 0; i < step_.actions.size(); ++i) {
    Action action = step_.actions[i];
    shared_ptr<ActionExecutor> ae(
        new ActionExecutor(action, action_clients_, &motion_planning_, world_,
                           robot_config_, runtime_viz_));
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
    moveit_msgs::MoveGroupResultConstPtr result =
        action_clients_->moveit_client.getResult();
    if (!result) {
      *error = "MoveIt returned null result.";
    }
    if (result->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
      std::stringstream ss;
      ss << errors::kUnreachablePose
         << " MoveIt error code: " << ErrorCodeToString(result->error_code);
      *error = ss.str();
    }

    // Insert a delay at the end to avoid "start state out of bounds" errors for
    // subsequent motion plans.
    ros::Duration(0.1).sleep();
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
