// Node that runs PR2 action servers using the control_msgs API.

#include "pr2_mechanism_msgs/ListControllers.h"
#include "pr2_mechanism_msgs/SwitchController.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd/pr2_actions.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;
namespace msgs = rapid_pbd_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_actuator_server");
  ros::NodeHandle nh;

  pbd::pr2::GripperAction left_gripper(pbd::kLeftGripperActionName,
                                       pbd::pr2::kLeftGripperActionName);
  pbd::pr2::GripperAction right_gripper(pbd::kRightGripperActionName,
                                        pbd::pr2::kRightGripperActionName);

  ros::Publisher arm_controller_state_pub =
      nh.advertise<msgs::ArmControllerState>(pbd::kArmControllerStateTopic, 5,
                                             true);
  ros::ServiceClient list_client =
      nh.serviceClient<pr2_mechanism_msgs::ListControllers>(
          pbd::pr2::kListControllersService);
  ros::ServiceClient switch_client =
      nh.serviceClient<pr2_mechanism_msgs::SwitchController>(
          pbd::pr2::kSwitchControllerService);
  pbd::pr2::ArmControllerManager arm_controller_manager(
      arm_controller_state_pub, list_client, switch_client);
  ros::ServiceServer freeze_srv = nh.advertiseService(
      pbd::kFreezeArmService, &pbd::pr2::ArmControllerManager::HandleFreeze,
      &arm_controller_manager);
  ros::ServiceServer relax_srv = nh.advertiseService(
      pbd::kRelaxArmService, &pbd::pr2::ArmControllerManager::HandleRelax,
      &arm_controller_manager);

  left_gripper.Start();
  right_gripper.Start();
  arm_controller_manager.Start();
  ros::spin();
  return 0;
}
