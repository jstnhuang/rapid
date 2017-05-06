#include <string>

#include "actionlib/client/simple_action_client.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/Action.h"
#include "rapid_pbd_msgs/ExecuteProgramAction.h"
#include "rapid_pbd_msgs/Step.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

using rapid_pbd_msgs::ExecuteProgramAction;
using rapid_pbd_msgs::ExecuteProgramGoal;
using rapid_pbd_msgs::Program;
using rapid_pbd_msgs::Action;
using rapid_pbd_msgs::Step;

trajectory_msgs::JointTrajectory BuildTrajectory(
    const std::string& prefix, double shoulder_pan, double shoulder_lift,
    double upper_arm_roll, double elbow_flex, double forearm_roll,
    double wrist_flex, double wrist_roll) {
  trajectory_msgs::JointTrajectory trajectory;
  trajectory.joint_names.push_back(prefix + "_shoulder_pan_joint");
  trajectory.joint_names.push_back(prefix + "_shoulder_lift_joint");
  trajectory.joint_names.push_back(prefix + "_upper_arm_roll_joint");
  trajectory.joint_names.push_back(prefix + "_elbow_flex_joint");
  trajectory.joint_names.push_back(prefix + "_forearm_roll_joint");
  trajectory.joint_names.push_back(prefix + "_wrist_flex_joint");
  trajectory.joint_names.push_back(prefix + "_wrist_roll_joint");
  trajectory.points.resize(1);
  trajectory.points[0].positions.resize(7);
  trajectory.points[0].positions[0] = shoulder_pan;
  trajectory.points[0].positions[1] = shoulder_lift;
  trajectory.points[0].positions[2] = upper_arm_roll;
  trajectory.points[0].positions[3] = elbow_flex;
  trajectory.points[0].positions[4] = forearm_roll;
  trajectory.points[0].positions[5] = wrist_flex;
  trajectory.points[0].positions[6] = wrist_roll;
  trajectory.points[0].time_from_start = ros::Duration(5);
  for (int i = 0; i < 7; ++i) {
    trajectory.points[0].velocities.push_back(0);
  }
  return trajectory;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "program_execution_demo");
  ros::NodeHandle nh;

  Action open_l_gripper;
  open_l_gripper.type = Action::ACTUATE_GRIPPER;
  open_l_gripper.actuator_group = Action::LEFT_GRIPPER;
  open_l_gripper.gripper_command.position = 0.08;
  open_l_gripper.gripper_command.max_effort = -1;

  Action open_r_gripper;
  open_r_gripper.type = Action::ACTUATE_GRIPPER;
  open_r_gripper.actuator_group = Action::RIGHT_GRIPPER;
  open_r_gripper.gripper_command.position = 0.08;
  open_r_gripper.gripper_command.max_effort = -1;

  Action extend_l_arm;
  extend_l_arm.type = Action::MOVE_TO_JOINT_GOAL;
  extend_l_arm.actuator_group = Action::LEFT_ARM;
  extend_l_arm.joint_trajectory = BuildTrajectory("l", 0, 0, 0, 0, 0, 0, 0);

  Action extend_r_arm;
  extend_r_arm.type = Action::MOVE_TO_JOINT_GOAL;
  extend_r_arm.actuator_group = Action::RIGHT_ARM;
  extend_r_arm.joint_trajectory = BuildTrajectory("r", 0, 0, 0, 0, 0, 0, 0);

  Step step1;
  step1.actions.push_back(open_l_gripper);
  step1.actions.push_back(open_r_gripper);
  step1.actions.push_back(extend_l_arm);
  step1.actions.push_back(extend_r_arm);

  Action close_l_gripper;
  close_l_gripper.type = Action::ACTUATE_GRIPPER;
  close_l_gripper.actuator_group = Action::LEFT_GRIPPER;
  close_l_gripper.gripper_command.position = 0;
  close_l_gripper.gripper_command.max_effort = 100;

  Action close_r_gripper;
  close_r_gripper.type = Action::ACTUATE_GRIPPER;
  close_r_gripper.actuator_group = Action::RIGHT_GRIPPER;
  close_r_gripper.gripper_command.position = 0;
  close_r_gripper.gripper_command.max_effort = 100;

  Action move_l_arm;
  move_l_arm.type = Action::MOVE_TO_JOINT_GOAL;
  move_l_arm.actuator_group = Action::LEFT_ARM;
  move_l_arm.joint_trajectory =
      BuildTrajectory("l", -0.3, 0.2, -0.1, -1.2, 1.5, -0.3, 0.5);

  Action move_r_arm;
  move_r_arm.type = Action::MOVE_TO_JOINT_GOAL;
  move_r_arm.actuator_group = Action::RIGHT_ARM;
  move_r_arm.joint_trajectory =
      BuildTrajectory("r", -1.3, -0.04, -1.4, -1.2, -1.78, -1.64, -7.59);

  Step step2;
  step2.actions.push_back(close_l_gripper);
  step2.actions.push_back(close_r_gripper);
  step2.actions.push_back(move_l_arm);
  step2.actions.push_back(move_r_arm);

  Program program;
  program.name = "Test program";
  program.steps.push_back(step1);
  program.steps.push_back(step2);

  actionlib::SimpleActionClient<ExecuteProgramAction> client(
      rapid::pbd::kProgramActionName, true);
  client.waitForServer();
  ExecuteProgramGoal goal;
  goal.program = program;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(8));
  client.cancelAllGoals();

  return 0;
}
