#ifndef _RAPID_PBD_ACTION_SERVER_H_
#define _RAPID_PBD_ACTION_SERVER_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "ros/ros.h"

#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/Action.h"

namespace rapid {
namespace pbd {
// An ActionExecutor takes an Action specification and runs it on the robot.
class ActionExecutor {
 public:
  explicit ActionExecutor(const rapid_pbd_msgs::Action& action);

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

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripper_client_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      l_gripper_client_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      r_gripper_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      arm_joint_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      l_arm_joint_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      r_arm_joint_client_;

  void ActuateGripper();
  void MoveToJointGoal();

  static void PublishInvalidGroupError(const rapid_pbd_msgs::Action& action);
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_SERVER_H_
