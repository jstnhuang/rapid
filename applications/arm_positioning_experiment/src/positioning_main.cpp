#include "actionlib/client/action_client.h"
#include "pr2_controllers_msgs/JointTrajectoryAction.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "rapid_manipulation/joint_state_reader.h"
#include "rapid_pr2/arm_joints.h"
#include "rapid_pr2/pr2.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <math.h>
#include <string>
#include <vector>
#include "Eigen/Dense"

using std::string;
using std::vector;
using rapid::pr2::Pr2;
using rapid::manipulation::JointStateReader;
using robot_model::RobotModelPtr;
typedef actionlib::SimpleActionClient<
    pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

struct ActionStep {
 public:
  ActionStep();
  string name;
  vector<double> l_arm_joints;
  vector<double> r_arm_joints;
  bool r_open;
  bool r_close;
};

ActionStep::ActionStep()
    : name(), l_arm_joints(), r_open(false), r_close(false) {}

void WaitForJointValues(const JointStateReader& joint_reader) {
  vector<string> r_arm_joints = rapid::pr2::right_arm_joint_names();
  while (true) {
    bool done = true;
    for (size_t i = 0; i < r_arm_joints.size(); ++i) {
      double position = joint_reader.get_position(r_arm_joints[i]);
      if (position == rapid::manipulation::kNoJointValue) {
        done = false;
        break;
      }
    }
    if (done) {
      break;
    } else {
      ros::spinOnce();
    }
  }
}

void MoveToJoints(Pr2* pr2, TrajClient* l_client, TrajClient* r_client,
                  const ActionStep& step, double time_from_start) {
  pr2_controllers_msgs::JointTrajectoryGoal left_goal;
  left_goal.trajectory.joint_names = rapid::pr2::left_arm_joint_names();
  left_goal.trajectory.points.resize(1);
  left_goal.trajectory.points[0].positions = step.l_arm_joints;
  left_goal.trajectory.points[0].time_from_start =
      ros::Duration(time_from_start);

  pr2_controllers_msgs::JointTrajectoryGoal right_goal;
  right_goal.trajectory.joint_names = rapid::pr2::right_arm_joint_names();
  right_goal.trajectory.points.resize(1);
  right_goal.trajectory.points[0].positions = step.r_arm_joints;
  right_goal.trajectory.points[0].time_from_start =
      ros::Duration(time_from_start);

  left_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  right_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);

  l_client->sendGoal(left_goal);
  r_client->sendGoal(right_goal);
  l_client->waitForResult();
  r_client->waitForResult();

  if (step.r_open) {
    pr2->right_gripper()->Open();
  } else if (step.r_close) {
    pr2->right_gripper()->Close();
  }
}

Eigen::Affine3d GetEEPose(const RobotModelPtr& kinematic_model,
                          const vector<double>& joint_values) {
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(kinematic_model));
  //  const robot_state::JointModelGroup* r_arm_model_group =
  //      kinematic_model->getJointModelGroup("right_arm");
  const vector<string>& r_joint_names = rapid::pr2::right_arm_joint_names();
  for (size_t i = 0; i < r_joint_names.size(); ++i) {
    kinematic_state->setJointPositions(r_joint_names[i], &joint_values[i]);
  }
  const Eigen::Affine3d& ee_state =
      kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");
  return ee_state;
}

void EvaluateStep(const JointStateReader& joint_reader,
                  const RobotModelPtr& kinematic_model,
                  const ActionStep& step) {
  ros::spinOnce();
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(kinematic_model));
  const vector<string>& r_joint_names = rapid::pr2::right_arm_joint_names();

  vector<double> actual_positions;
  joint_reader.get_positions(r_joint_names, &actual_positions);

  ROS_INFO("Name \t\t Desired \t Actual \t Error");
  for (size_t i = 0; i < r_joint_names.size(); ++i) {
    double error = step.r_arm_joints[i] - actual_positions[i];
    ROS_INFO("%s \t %f \t %f \t %f", r_joint_names[i].c_str(),
             step.r_arm_joints[i], actual_positions[i], error);
  }

  Eigen::Affine3d desired_ee = GetEEPose(kinematic_model, step.r_arm_joints);
  Eigen::Affine3d actual_ee = GetEEPose(kinematic_model, actual_positions);

  ROS_INFO("Dim \t Desired \t Actual \t Error");
  for (char c = 'X'; c <= 'Z'; ++c) {
    int i = c - 'X';
    double error = desired_ee.translation()[i] - actual_ee.translation()[i];
    ROS_INFO("%c \t %f \t %f \t %f", c, desired_ee.translation()[i],
             actual_ee.translation()[i], error);
  }

  Eigen::Matrix3d difference =
      actual_ee.rotation() * desired_ee.rotation().transpose();
  Eigen::AngleAxisd difference_aa(difference);
  ROS_INFO("Total translation error (Cartesian): %f",
           (desired_ee.translation() - actual_ee.translation()).norm());
  ROS_INFO("Total rotation error (Rotation in degs): %f",
           difference_aa.angle() * 180 / M_PI);
}

void ShowAndSay(Pr2* pr2, const string& message) {
  pr2->display()->ShowMessage(message, "");
  pr2->sound()->Say(message);
}

void WaitForButton(Pr2* pr2) {
  pr2->sound()->Say("Press Okay to continue.");
  vector<string> choices;
  choices.push_back("Okay");
  string choice;
  pr2->display()->AskMultipleChoice("Press Okay to continue.", choices,
                                    &choice);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_positioning_experiment");
  ros::NodeHandle nh;
  rapid::manipulation::JointStateReader joint_reader;
  joint_reader.Start();
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  Pr2* pr2 = rapid::pr2::BuildReal(nh);
  TrajClient l_arm_client("l_arm_controller/joint_trajectory_action", true);
  TrajClient r_arm_client("r_arm_controller/joint_trajectory_action", true);
  l_arm_client.waitForServer();
  r_arm_client.waitForServer();

  ActionStep start;
  start.name = "starting pose";
  start.l_arm_joints.push_back(1.4985608859017916);
  start.l_arm_joints.push_back(0.03155390631796161);
  start.l_arm_joints.push_back(1.370513148654012);
  start.l_arm_joints.push_back(-1.076632987397554);
  start.l_arm_joints.push_back(1.5777792965765376);
  start.l_arm_joints.push_back(-1.4758956344899106);
  start.l_arm_joints.push_back(-2.0523664418123415);
  start.r_arm_joints.push_back(-1.304476494669088);
  start.r_arm_joints.push_back(-0.04297422093706298);
  start.r_arm_joints.push_back(-1.411724339031073);
  start.r_arm_joints.push_back(-1.78354837108332);
  start.r_arm_joints.push_back(-1.639097208744132);
  start.r_arm_joints.push_back(-1.3765792728759028);
  start.r_arm_joints.push_back(-7.593869563531825);

  ActionStep pre_grasp;
  pre_grasp.name = "pre-grasp pose";
  pre_grasp.l_arm_joints = start.l_arm_joints;
  pre_grasp.r_arm_joints.push_back(-0.4967173217599755);
  pre_grasp.r_arm_joints.push_back(0.2076805362214362);
  pre_grasp.r_arm_joints.push_back(-1.6798376087215228);
  pre_grasp.r_arm_joints.push_back(-0.9366397094896827);
  pre_grasp.r_arm_joints.push_back(-0.06449950194987482);
  pre_grasp.r_arm_joints.push_back(-1.162123805705002);
  pre_grasp.r_arm_joints.push_back(-8.05475955414683);
  pre_grasp.r_open = true;

  ActionStep grasp;
  grasp.name = "grasp pose";
  grasp.l_arm_joints = start.l_arm_joints;
  grasp.r_arm_joints.push_back(-0.4031157507639387);
  grasp.r_arm_joints.push_back(0.1871239699070537);
  grasp.r_arm_joints.push_back(-1.723133411841392);
  grasp.r_arm_joints.push_back(-0.9401142065628874);
  grasp.r_arm_joints.push_back(-0.023775152736680242);
  grasp.r_arm_joints.push_back(-1.0979481485581002);
  grasp.r_arm_joints.push_back(-8.097876893965527);
  grasp.r_close = true;

  ActionStep lift;
  lift.name = "lift pose";
  lift.l_arm_joints = start.l_arm_joints;
  lift.r_arm_joints.push_back(-0.38595408097104256);
  lift.r_arm_joints.push_back(-0.027662539855156678);
  lift.r_arm_joints.push_back(-1.6424749341773393);
  lift.r_arm_joints.push_back(-0.9626984375387176);
  lift.r_arm_joints.push_back(-0.14803069550648423);
  lift.r_arm_joints.push_back(-1.1831386141131057);
  lift.r_arm_joints.push_back(-7.82733842878624);

  ActionStep pre_place;
  pre_place.name = "pre-place pose";
  pre_place.l_arm_joints = start.l_arm_joints;
  pre_place.r_arm_joints.push_back(-0.284642194560806);
  pre_place.r_arm_joints.push_back(0.047542346784703524);
  pre_place.r_arm_joints.push_back(-1.66957489983385);
  pre_place.r_arm_joints.push_back(-0.9917973505268067);
  pre_place.r_arm_joints.push_back(-0.06421026651512213);
  pre_place.r_arm_joints.push_back(-0.9325272343394421);
  pre_place.r_arm_joints.push_back(-7.888947059647275);

  ActionStep place;
  place.name = "place pose";
  place.l_arm_joints = start.l_arm_joints;
  place.r_arm_joints.push_back(-0.2820720894227394);
  place.r_arm_joints.push_back(0.06479971109801227);
  place.r_arm_joints.push_back(-1.6772719314996045);
  place.r_arm_joints.push_back(-0.9884676241649856);
  place.r_arm_joints.push_back(-0.0628219364283086);
  place.r_arm_joints.push_back(-0.9446227141271298);
  place.r_arm_joints.push_back(-7.934196336550853);
  place.r_open = true;

  ActionStep post_place;
  post_place.name = "post-place pose";
  post_place.l_arm_joints = start.l_arm_joints;
  post_place.r_arm_joints.push_back(-0.3041252496396976);
  post_place.r_arm_joints.push_back(0.013365997850503838);
  post_place.r_arm_joints.push_back(-1.6182613553954863);
  post_place.r_arm_joints.push_back(-1.0524562785965048);
  post_place.r_arm_joints.push_back(-0.04992203603833377);
  post_place.r_arm_joints.push_back(-0.8628259443398926);
  post_place.r_arm_joints.push_back(-7.921665767562169);

  // ActionStep finish;
  // finish.name = "finishing pose";
  // finish.l_arm_joints = start.l_arm_joints;
  // finish.r_arm_joints.push_back(-1.304476494669088);
  // finish.r_arm_joints.push_back(-0.04297422093706298);
  // finish.r_arm_joints.push_back(-1.411724339031073);
  // finish.r_arm_joints.push_back(-1.78354837108332);
  // finish.r_arm_joints.push_back(-1.639097208744132);
  // finish.r_arm_joints.push_back(-1.3765792728759028);
  // finish.r_arm_joints.push_back(-7.593869563531825);

  vector<ActionStep> action_steps;
  action_steps.push_back(start);
  action_steps.push_back(pre_grasp);
  action_steps.push_back(grasp);
  action_steps.push_back(lift);
  action_steps.push_back(pre_place);
  action_steps.push_back(place);
  action_steps.push_back(post_place);

  WaitForButton(pr2);

  for (size_t i = 0; i < action_steps.size(); ++i) {
    double time_from_start;
    ros::param::param<double>("time_from_start", time_from_start, 10);
    const ActionStep& step = action_steps[i];
    ShowAndSay(pr2, "Moving to " + step.name + ".");
    MoveToJoints(pr2, &l_arm_client, &r_arm_client, step, time_from_start);
    EvaluateStep(joint_reader, kinematic_model, step);
    ShowAndSay(pr2, "Moved to " + step.name + ".");
    WaitForButton(pr2);
  }

  pr2->display()->ShowDefault();

  delete pr2;
  return 0;
}
