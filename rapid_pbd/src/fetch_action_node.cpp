#include "rapid_pbd/action_names.h"
#include "rapid_pbd/fetch_actions.h"
#include "rapid_pbd_msgs/ArmControllerState.h"
#include "rapid_pbd_msgs/FreezeArm.h"
#include "rapid_pbd_msgs/RelaxArm.h"
#include "robot_controllers_msgs/QueryControllerStatesAction.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;
namespace msgs = rapid_pbd_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fetch_actuator_server");
  ros::NodeHandle nh;

  ros::Publisher arm_controller_state_pub =
      nh.advertise<msgs::ArmControllerState>(pbd::kArmControllerStateTopic, 5,
                                             true);
  actionlib::SimpleActionClient<
      robot_controllers_msgs::QueryControllerStatesAction>
      client(pbd::fetch::kControllerActionName, true);
  while (ros::ok() && !client.waitForServer(ros::Duration(5.0))) {
    ROS_WARN("Waiting for arm controller manager.");
  }
  pbd::fetch::ArmControllerManager arm_controller_manager(
      arm_controller_state_pub, &client);
  ros::ServiceServer freeze_srv = nh.advertiseService(
      pbd::kFreezeArmService, &pbd::fetch::ArmControllerManager::HandleFreeze,
      &arm_controller_manager);
  ros::ServiceServer relax_srv = nh.advertiseService(
      pbd::kRelaxArmService, &pbd::fetch::ArmControllerManager::HandleRelax,
      &arm_controller_manager);

  arm_controller_manager.Start();
  ros::spin();
  return 0;
}
