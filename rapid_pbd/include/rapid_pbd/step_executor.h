#ifndef _RAPID_PBD_STEP_EXECUTOR_H_
#define _RAPID_PBD_STEP_EXECUTOR_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "rapid_pbd_msgs/Step.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/action_executor.h"
#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/motion_planning.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
class StepExecutor {
 public:
  StepExecutor(const rapid_pbd_msgs::Step& step, ActionClients* action_clients,
               const RobotConfig& robot_config, World* world,
               const RuntimeVisualizer& runtime_viz,
               const tf::TransformListener& tf_listener,
               const ros::Publisher& planning_scene_pub,
               const JointStateReader& js_reader);

  // Returns true if the Step message is valid, false otherwise.
  // You should call this method to verify the step message before executing it.
  static bool IsValid(const rapid_pbd_msgs::Step& step);

  // Initializes the step. This should be called before calling Start().
  void Init();

  // Starts the step execution. Returns error message or empty string.
  std::string Start();

  // Returns true if the step execution is complete or if the step is invalid.
  // Leaves error string unchanged if no error, otherwise saves an error string.
  bool IsDone(std::string* error) const;

  // Cancels the execution of the step.
  void Cancel();

 private:
  rapid_pbd_msgs::Step step_;
  ActionClients* action_clients_;
  const RobotConfig& robot_config_;
  World* world_;
  RuntimeVisualizer runtime_viz_;
  MotionPlanning motion_planning_;
  std::vector<boost::shared_ptr<ActionExecutor> > executors_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_STEP_EXECUTOR_H_
