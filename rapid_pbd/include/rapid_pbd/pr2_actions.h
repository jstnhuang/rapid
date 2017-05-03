// This file adapts the PR2 actuator actions, which use a PR2-specific
// interface, to use the more standard interfaces in the control_msgs package.

#ifndef _RAPID_PBD_PR2_H_
#define _RAPID_PBD_PR2_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

namespace {
typedef actionlib::SimpleActionClient<
    pr2_controllers_msgs::Pr2GripperCommandAction> Pr2GripperClient;
using pr2_controllers_msgs::Pr2GripperCommandFeedback;
using pr2_controllers_msgs::Pr2GripperCommandResultConstPtr;
}  // namespace

namespace rapid {
namespace pbd {
namespace pr2 {
// Constants for the names of our actions and the names of the actions in the
// PR2 API.
static const char kLeftGripperActionName[] = "rapid_pbd/l_gripper_action";
static const char kRightGripperActionName[] = "rapid_pbd/r_gripper_action";
static const char kPr2LeftGripperActionName[] =
    "l_gripper_controller/gripper_action";
static const char kPr2RightGripperActionName[] =
    "r_gripper_controller/gripper_action";

static const char kLeftArmJointActionName[] = "rapid_pbd/l_arm_joint_action";
static const char kRightArmJointActionName[] = "rapid_pbd/r_arm_joint_action";

class GripperAction {
 public:
  GripperAction(const std::string& name, const std::string& pr2_action_name);

  void Start();
  void Execute(const control_msgs::GripperCommandGoalConstPtr& goal);
  void HandleFeedback(const Pr2GripperCommandFeedback::ConstPtr& pr2_feedback);

 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_;
  Pr2GripperClient pr2_client_;
};
}  // namespace pr2
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PR2_H_
