#ifndef _RAPID_PBD_ACTION_EXECUTOR_H_
#define _RAPID_PBD_ACTION_EXECUTOR_H_

#include "rapid_pbd/action_clients.h"
#include "rapid_pbd_msgs/Action.h"

namespace rapid {
namespace pbd {
// An ActionExecutor takes an Action specification and runs it on the robot.
class ActionExecutor {
 public:
  ActionExecutor(const rapid_pbd_msgs::Action& action,
                 ActionClients* action_clients);

  // Returns true if the given action message is valid, false otherwise.
  // Public methods of ActionExecutor will use this at the start and return
  // early if the check doesn't pass. Private methods will assume that the
  // message format is valid.
  static bool IsValid(const rapid_pbd_msgs::Action& action);

  // Start the execution of the action.
  void Start();

  // Returns true if the action is done (or if the action is invalid).
  bool IsDone() const;

  // Cancels the execution of the action.
  void Cancel();

 private:
  rapid_pbd_msgs::Action action_;
  ActionClients* clients_;

  void ActuateGripper();
  void MoveToJointGoal();

  static void PublishInvalidGroupError(const rapid_pbd_msgs::Action& action);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_EXECUTOR_H_
