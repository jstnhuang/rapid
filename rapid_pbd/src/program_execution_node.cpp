#include "actionlib/client/simple_action_client.h"
#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd/action_executor.h"
#include "rapid_pbd/program_executor.h"
#include "std_msgs/Bool.h"
#include "rapid_pbd/action_names.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_executor");
  ros::NodeHandle nh;
  ros::Publisher is_running_pub =
      nh.advertise<std_msgs::Bool>("is_running", 5, true);

  pbd::ActionClients action_clients;
  int num_arms;
  bool success = ros::param::get("num_arms", num_arms);
  if (!success) {
    ROS_ERROR("num_arms param must be set.");
    return 1;
  }
  if (num_arms == 1) {
    while (!action_clients.gripper_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for gripper server.");
    }
    while (!action_clients.arm_joint_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for arm joint server.");
    }
  } else if (num_arms == 2) {
    while (!action_clients.l_gripper_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for left gripper server.");
    }
    while (!action_clients.r_gripper_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for right gripper server.");
    }
    while (!action_clients.l_arm_joint_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for left arm joint server.");
    }
    while (!action_clients.r_arm_joint_client.waitForServer(ros::Duration(5))) {
      ROS_WARN("Waiting for right arm joint server.");
    }
  } else {
    ROS_ERROR("num_arms can only be 1 or 2.");
    return 1;
  }

  rapid::pbd::ProgramExecutionServer server(rapid::pbd::kProgramActionName,
                                            is_running_pub, &action_clients);
  server.Start();
  ROS_INFO("RapidPbD program executor ready.");
  ros::spin();
  return 0;
}
