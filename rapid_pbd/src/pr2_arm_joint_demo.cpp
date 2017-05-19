#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "rapid_pbd/action_names.h"
#include "ros/ros.h"

bool wait_for_server(actionlib::SimpleActionClient<
    control_msgs::FollowJointTrajectoryAction>& arm) {
  bool success = arm.waitForServer(ros::Duration(5));
  if (!success) {
    ROS_ERROR("Error connecting to arm server");
  }
  return success;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_arm_demo");

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      left_arm(rapid::pbd::kLeftArmJointActionName, true);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      right_arm(rapid::pbd::kRightArmJointActionName, true);
  if (!wait_for_server(left_arm)) {
    left_arm.cancelAllGoals();
    return 1;
  }
  if (!wait_for_server(right_arm)) {
    right_arm.cancelAllGoals();
    return 1;
  }

  control_msgs::FollowJointTrajectoryGoal left_goal;
  left_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  left_goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
  left_goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
  left_goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  left_goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
  left_goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
  left_goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
  left_goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  left_goal.trajectory.points.resize(1);
  left_goal.trajectory.points[0].positions.resize(7);
  left_goal.trajectory.points[0].positions[0] = 0;
  left_goal.trajectory.points[0].positions[1] = 0;
  left_goal.trajectory.points[0].positions[2] = 0;
  left_goal.trajectory.points[0].positions[3] = 0;
  left_goal.trajectory.points[0].positions[4] = 0;
  left_goal.trajectory.points[0].positions[5] = 0;
  left_goal.trajectory.points[0].positions[6] = 0;
  left_goal.trajectory.points[0].time_from_start = ros::Duration(4);
  for (int i = 0; i < 7; ++i) {
    left_goal.trajectory.points[0].velocities.push_back(0);
  }

  control_msgs::FollowJointTrajectoryGoal right_goal;
  right_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  right_goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  right_goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  right_goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  right_goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  right_goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  right_goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  right_goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
  right_goal.trajectory.points.resize(1);
  right_goal.trajectory.points[0].positions.resize(7);
  right_goal.trajectory.points[0].positions[0] = 0;
  right_goal.trajectory.points[0].positions[1] = 0;
  right_goal.trajectory.points[0].positions[2] = 0;
  right_goal.trajectory.points[0].positions[3] = 0;
  right_goal.trajectory.points[0].positions[4] = 0;
  right_goal.trajectory.points[0].positions[5] = 0;
  right_goal.trajectory.points[0].positions[6] = 0;
  right_goal.trajectory.points[0].time_from_start = ros::Duration(4);
  for (int i = 0; i < 7; ++i) {
    right_goal.trajectory.points[0].velocities.push_back(0);
  }

  left_arm.sendGoal(left_goal);
  right_arm.sendGoal(right_goal);
  bool l_success = left_arm.waitForResult(ros::Duration(5));
  bool r_success = right_arm.waitForResult(ros::Duration(5));
  if (!l_success) {
    ROS_ERROR("Left arm failed!");
  }
  if (!r_success) {
    ROS_ERROR("Right arm failed!");
  }

  control_msgs::FollowJointTrajectoryGoal left_goal2;
  left_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  left_goal2.trajectory.joint_names.push_back("l_shoulder_pan_joint");
  left_goal2.trajectory.joint_names.push_back("l_shoulder_lift_joint");
  left_goal2.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  left_goal2.trajectory.joint_names.push_back("l_elbow_flex_joint");
  left_goal2.trajectory.joint_names.push_back("l_forearm_roll_joint");
  left_goal2.trajectory.joint_names.push_back("l_wrist_flex_joint");
  left_goal2.trajectory.joint_names.push_back("l_wrist_roll_joint");
  left_goal2.trajectory.points.resize(1);
  left_goal2.trajectory.points[0].positions.resize(7);
  left_goal2.trajectory.points[0].positions[0] = -0.3;
  left_goal2.trajectory.points[0].positions[1] = 0.2;
  left_goal2.trajectory.points[0].positions[2] = -0.1;
  left_goal2.trajectory.points[0].positions[3] = -1.2;
  left_goal2.trajectory.points[0].positions[4] = 1.5;
  left_goal2.trajectory.points[0].positions[5] = -0.3;
  left_goal2.trajectory.points[0].positions[6] = 0.5;
  left_goal2.trajectory.points[0].time_from_start = ros::Duration(4);
  for (int i = 0; i < 7; ++i) {
    left_goal2.trajectory.points[0].velocities.push_back(0);
  }

  control_msgs::FollowJointTrajectoryGoal right_goal2;
  right_goal2.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  right_goal2.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  right_goal2.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  right_goal2.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  right_goal2.trajectory.joint_names.push_back("r_elbow_flex_joint");
  right_goal2.trajectory.joint_names.push_back("r_forearm_roll_joint");
  right_goal2.trajectory.joint_names.push_back("r_wrist_flex_joint");
  right_goal2.trajectory.joint_names.push_back("r_wrist_roll_joint");
  right_goal2.trajectory.points.resize(1);
  right_goal2.trajectory.points[0].positions.resize(7);
  right_goal2.trajectory.points[0].positions[0] = -1.3;
  right_goal2.trajectory.points[0].positions[1] = -0.04;
  right_goal2.trajectory.points[0].positions[2] = -1.4;
  right_goal2.trajectory.points[0].positions[3] = -1.2;
  right_goal2.trajectory.points[0].positions[4] = -1.78;
  right_goal2.trajectory.points[0].positions[5] = -1.64;
  right_goal2.trajectory.points[0].positions[6] = -7.59;
  right_goal2.trajectory.points[0].time_from_start = ros::Duration(4);
  for (int i = 0; i < 7; ++i) {
    right_goal2.trajectory.points[0].velocities.push_back(0);
  }

  left_arm.sendGoal(left_goal2);
  right_arm.sendGoal(right_goal2);
  l_success = left_arm.waitForResult(ros::Duration(5));
  r_success = right_arm.waitForResult(ros::Duration(5));
  if (!l_success) {
    ROS_ERROR("Left arm failed!");
  }
  if (!r_success) {
    ROS_ERROR("Right arm failed!");
  }

  left_arm.cancelAllGoals();
  right_arm.cancelAllGoals();
  ros::spinOnce();
  return 0;
}
