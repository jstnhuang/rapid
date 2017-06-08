// This file adapts the PR2 actuator actions, which use a PR2-specific
// interface, to use the more standard interfaces in the control_msgs package.

#ifndef _RAPID_PBD_PR2_H_
#define _RAPID_PBD_PR2_H_

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/GripperCommandAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"

namespace {
typedef actionlib::SimpleActionClient<
    pr2_controllers_msgs::Pr2GripperCommandAction>
    Pr2GripperClient;
using pr2_controllers_msgs::Pr2GripperCommandFeedback;
using pr2_controllers_msgs::Pr2GripperCommandResultConstPtr;
}  // namespace

namespace rapid {
namespace pbd {
namespace pr2 {
class GripperAction {
 public:
  GripperAction(const std::string& name, const std::string& pr2_action_name);

  void Start();
  void Execute(const control_msgs::GripperCommandGoalConstPtr& goal);
  void HandleFeedback(const Pr2GripperCommandFeedback::ConstPtr& pr2_feedback);

 private:
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> server_;
  Pr2GripperClient pr2_client_;
};

// ArmControllerManager manages the controllers running on the PR2 arms.
class ArmControllerManager {
 public:
  ArmControllerManager(const ros::Publisher& state_pub,
                       const ros::ServiceClient& list_client,
                       const ros::ServiceClient& switch_client);

  // Publishes an initial state message.
  void Start();

  bool HandleFreeze(rapid_pbd_msgs::FreezeArmRequest& request,
                    rapid_pbd_msgs::FreezeArmResponse& response);
  bool HandleRelax(rapid_pbd_msgs::RelaxArmRequest& request,
                   rapid_pbd_msgs::RelaxArmResponse& response);

 private:
  void Update();

  ros::Publisher state_pub_;
  ros::ServiceClient list_client_;
  ros::ServiceClient switch_client_;

  bool is_l_arm_active_;
  bool is_r_arm_active_;
};
}  // namespace pr2
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_PR2_H_
