#ifndef _RAPID_PBD_ACTION_EXECUTOR_H_
#define _RAPID_PBD_ACTION_EXECUTOR_H_

#include "rapid_pbd_msgs/Action.h"

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd/motion_planning.h"
#include "rapid_pbd/visualizer.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
// An ActionExecutor takes an Action specification and runs it on the robot.
class ActionExecutor {
 public:
  ActionExecutor(const rapid_pbd_msgs::Action& action,
                 ActionClients* action_clients, MotionPlanning* motion_planning,
                 World* world, const RobotConfig& robot_config,
                 const RuntimeVisualizer& runtime_viz);

  // Returns true if the given action message is valid, false otherwise.
  // Public methods of ActionExecutor will use this at the start and return
  // early if the check doesn't pass. Private methods will assume that the
  // message format is valid.
  static bool IsValid(const rapid_pbd_msgs::Action& action);

  // Start the execution of the action.
  std::string Start();

  // Returns true if the action is done (or if the action is invalid).
  bool IsDone(std::string* error) const;

  // Cancels the execution of the action.
  void Cancel();

 private:
  rapid_pbd_msgs::Action action_;
  ActionClients* clients_;
  MotionPlanning* motion_planning_;
  World* world_;
  const RobotConfig& robot_config_;
  RuntimeVisualizer runtime_viz_;

  void ActuateGripper();
  void MoveToJointGoal();
  void DetectTabletopObjects();

  static void PublishInvalidGroupError(const rapid_pbd_msgs::Action& action);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_EXECUTOR_H_
