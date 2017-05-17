#include "rapid_pbd/program_executor.h"

#include "rapid_pbd/step_executor.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/Program.h"

using rapid_pbd_msgs::Action;
using rapid_pbd_msgs::ExecuteProgramFeedback;
using rapid_pbd_msgs::ExecuteProgramGoal;
using rapid_pbd_msgs::ExecuteProgramResult;
using rapid_pbd_msgs::FreezeArm;
using rapid_pbd_msgs::Step;

namespace rapid {
namespace pbd {
ProgramExecutionServer::ProgramExecutionServer(const std::string& action_name)
    : nh_(),
      server_(action_name,
              boost::bind(&ProgramExecutionServer::Execute, this, _1), false),
      freeze_arm_client_(nh_.serviceClient<FreezeArm>(kFreezeArmService)) {}

void ProgramExecutionServer::Start() { server_.start(); }

void ProgramExecutionServer::Execute(
    const rapid_pbd_msgs::ExecuteProgramGoalConstPtr& goal) {
  if (!IsValid(goal->program)) {
    std::string error("Program \"" + goal->program.name +
                      "\" was not constructed properly.");
    ExecuteProgramResult result;
    result.error = error;
    server_.setAborted(result, error);
    return;
  }

  // Enable controllers.
  while (!freeze_arm_client_.waitForExistence(ros::Duration(5))) {
    ROS_WARN("Waiting for freeze arm service.");
  }
  FreezeArm::Request req;
  FreezeArm::Response res;
  req.actuator_group = Action::LEFT_ARM;
  freeze_arm_client_.call(req, res);
  req.actuator_group = Action::RIGHT_ARM;
  freeze_arm_client_.call(req, res);

  for (size_t i = 0; i < goal->program.steps.size(); ++i) {
    ExecuteProgramFeedback feedback;
    feedback.step_number = i;
    server_.publishFeedback(feedback);

    Step step = goal->program.steps[i];
    StepExecutor executor(step);
    executor.Start();
    while (!executor.IsDone()) {
      if (server_.isPreemptRequested() || !ros::ok()) {
        executor.Cancel();
        ros::spinOnce();
        std::string error("Program \"" + goal->program.name +
                          "\" was preempted.");
        ExecuteProgramResult result;
        result.error = error;
        server_.setPreempted(result, error);
        return;
      }
      ros::spinOnce();
    }
  }
  server_.setSucceeded();
}

bool ProgramExecutionServer::IsValid(const rapid_pbd_msgs::Program& program) {
  for (size_t i = 0; i < program.steps.size(); ++i) {
    const Step& step = program.steps[i];
    if (!StepExecutor::IsValid(step)) {
      return false;
    }
  }
  return true;
}
}  // namespace pbd
}  // namespace rapid
