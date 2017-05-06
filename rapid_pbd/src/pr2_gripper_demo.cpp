#include "actionlib/client/simple_action_client.h"
#include "control_msgs/GripperCommandAction.h"
#include "rapid_pbd/action_names.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_gripper_demo");

  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      left_gripper(rapid::pbd::kLeftGripperActionName, true);
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      right_gripper(rapid::pbd::kRightGripperActionName, true);
  bool success = left_gripper.waitForServer(ros::Duration(10));
  success &= right_gripper.waitForServer(ros::Duration(10));
  if (!success) {
    ROS_ERROR("Failed to connect to server!");
    return 1;
  }

  control_msgs::GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1;

  right_gripper.sendGoalAndWait(open, ros::Duration(5), ros::Duration(5));
  left_gripper.sendGoal(open);
  bool l_success = left_gripper.waitForResult(ros::Duration(5));
  bool r_success = right_gripper.waitForResult(ros::Duration(5));
  if (!l_success) {
    ROS_ERROR("Failed to open left gripper.");
  }
  if (!r_success) {
    ROS_ERROR("Failed to open right gripper.");
  }

  control_msgs::GripperCommandGoal close;
  close.command.position = 0;
  close.command.max_effort = -1;

  left_gripper.sendGoal(close);
  right_gripper.sendGoal(close);

  ros::Rate rate(2);
  rate.sleep();
  right_gripper.cancelAllGoals();

  l_success = left_gripper.waitForResult(ros::Duration(5));
  r_success = right_gripper.waitForResult(ros::Duration(5));
  if (!l_success) {
    ROS_ERROR("Failed to close left gripper.");
  }
  if (!r_success) {
    ROS_ERROR("Failed to close right gripper.");
  }

  if (left_gripper.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Left gripper succeeded");
  } else {
    ROS_ERROR("Left gripper failed");
  }
  if (right_gripper.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Right gripper succeeded");
  } else {
    ROS_ERROR("Right gripper failed");
  }

  return 0;
}
