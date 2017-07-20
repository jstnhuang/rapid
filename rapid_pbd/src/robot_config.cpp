#include "rapid_pbd/robot_config.h"

#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "rapid_pbd_msgs/Action.h"

using rapid_pbd_msgs::Action;

namespace rapid {
namespace pbd {
Pr2RobotConfig::Pr2RobotConfig(robot_model::RobotModelPtr kinematic_model) {
  kinematic_model_ = kinematic_model;
  l_model_group_ = kinematic_model_->getJointModelGroup("left_arm");
  r_model_group_ = kinematic_model_->getJointModelGroup("right_arm");
}

std::string Pr2RobotConfig::planning_frame() const { return "base_footprint"; }
std::string Pr2RobotConfig::planning_group() const { return "arms"; }
std::string Pr2RobotConfig::base_link() const { return "base_footprint"; }
std::string Pr2RobotConfig::torso_link() const { return "torso_lift_link"; }
std::string Pr2RobotConfig::ee_frame_for_group(
    const std::string& actuator_group) const {
  if (actuator_group == Action::LEFT_ARM) {
    return "l_wrist_roll_link";
  } else if (actuator_group == Action::RIGHT_ARM) {
    return "r_wrist_roll_link";
  } else {
    return "";
  }
}
void Pr2RobotConfig::gripper_joints_for_group(
    const std::string& actuator_group,
    std::vector<std::string>* joint_names) const {
  joint_names->clear();
  if (actuator_group == Action::LEFT_GRIPPER) {
    joint_names->push_back("l_gripper_joint");
    joint_names->push_back("l_gripper_l_finger_joint");
    joint_names->push_back("l_gripper_r_finger_joint");
    joint_names->push_back("l_gripper_l_finger_tip_joint");
    joint_names->push_back("l_gripper_r_finger_tip_joint");
  } else if (actuator_group == Action::RIGHT_GRIPPER) {
    joint_names->push_back("r_gripper_joint");
    joint_names->push_back("r_gripper_l_finger_joint");
    joint_names->push_back("r_gripper_r_finger_joint");
    joint_names->push_back("r_gripper_l_finger_tip_joint");
    joint_names->push_back("r_gripper_r_finger_tip_joint");
  }
}
void Pr2RobotConfig::gripper_open_positions(
    std::vector<double>* positions) const {
  positions->clear();
  positions->push_back(0.088);
  positions->push_back(0.514);
  positions->push_back(0.514);
  positions->push_back(0.514);
  positions->push_back(0.514);
}
void Pr2RobotConfig::gripper_close_positions(
    std::vector<double>* positions) const {
  positions->clear();
  positions->push_back(0.0);
  positions->push_back(0.0069);
  positions->push_back(0.0069);
  positions->push_back(0.0069);
  positions->push_back(0.0069);
}
void Pr2RobotConfig::joints_for_group(
    const std::string& actuator_group,
    std::vector<std::string>* joint_names) const {
  joint_names->clear();
  if (actuator_group == Action::LEFT_ARM) {
    joint_names->push_back("l_shoulder_pan_joint");
    joint_names->push_back("l_shoulder_lift_joint");
    joint_names->push_back("l_upper_arm_roll_joint");
    joint_names->push_back("l_elbow_flex_joint");
    joint_names->push_back("l_forearm_roll_joint");
    joint_names->push_back("l_wrist_flex_joint");
    joint_names->push_back("l_wrist_roll_joint");
  } else if (actuator_group == Action::RIGHT_ARM) {
    joint_names->push_back("r_shoulder_pan_joint");
    joint_names->push_back("r_shoulder_lift_joint");
    joint_names->push_back("r_upper_arm_roll_joint");
    joint_names->push_back("r_elbow_flex_joint");
    joint_names->push_back("r_forearm_roll_joint");
    joint_names->push_back("r_wrist_flex_joint");
    joint_names->push_back("r_wrist_roll_joint");
  }
}
int Pr2RobotConfig::num_arms() const { return 2; }

bool Pr2RobotConfig::ComputeIk(const std::string& actuator_group,
                               const geometry_msgs::Pose& pose,
                               std::vector<std::string>* joint_names,
                               std::vector<double>* joint_values) const {
  robot_state::RobotStatePtr kinematic_state(
      new robot_state::RobotState(kinematic_model_));

  const robot_state::JointModelGroup* model_group;
  if (actuator_group == Action::LEFT_ARM) {
    model_group = l_model_group_;
  } else if (actuator_group == Action::RIGHT_ARM) {
    model_group = r_model_group_;
  } else {
    return false;
  }

  bool found_ik = kinematic_state->setFromIK(model_group, pose, 100, 5);
  if (!found_ik) {
    return false;
  }
  *joint_names = model_group->getVariableNames();
  kinematic_state->copyJointGroupPositions(model_group, *joint_values);
  return true;
}

FetchRobotConfig::FetchRobotConfig(robot_model::RobotModelPtr kinematic_model) {
  kinematic_model_ = kinematic_model;
  arm_group_ = kinematic_model_->getJointModelGroup("arm");
}

std::string FetchRobotConfig::planning_frame() const { return "base_link"; }
std::string FetchRobotConfig::planning_group() const { return "arm"; }
std::string FetchRobotConfig::base_link() const { return "base_link"; }
std::string FetchRobotConfig::torso_link() const { return "torso_lift_link"; }
std::string FetchRobotConfig::ee_frame_for_group(
    const std::string& actuator_group) const {
  if (actuator_group == Action::ARM) {
    return "wrist_roll_link";
  } else {
    return "";
  }
}
void FetchRobotConfig::gripper_joints_for_group(
    const std::string& actuator_group,
    std::vector<std::string>* joint_names) const {
  joint_names->clear();
  if (actuator_group == Action::GRIPPER) {
    joint_names->push_back("l_gripper_finger_joint");
    joint_names->push_back("r_gripper_finger_joint");
  }
}
void FetchRobotConfig::gripper_open_positions(
    std::vector<double>* positions) const {
  positions->clear();
  positions->push_back(0.05);
  positions->push_back(0.05);
}
void FetchRobotConfig::gripper_close_positions(
    std::vector<double>* positions) const {
  positions->clear();
  positions->push_back(0.0);
  positions->push_back(0.0);
}
void FetchRobotConfig::joints_for_group(
    const std::string& actuator_group,
    std::vector<std::string>* joint_names) const {
  joint_names->clear();
  if (actuator_group == Action::ARM) {
    joint_names->push_back("shoulder_pan_joint");
    joint_names->push_back("shoulder_lift_joint");
    joint_names->push_back("upperarm_roll_joint");
    joint_names->push_back("elbow_flex_joint");
    joint_names->push_back("forearm_roll_joint");
    joint_names->push_back("wrist_flex_joint");
    joint_names->push_back("wrist_roll_joint");
  }
}
int FetchRobotConfig::num_arms() const { return 1; }

bool FetchRobotConfig::ComputeIk(const std::string& actuator_group,
                               const geometry_msgs::Pose& pose,
                               std::vector<std::string>* joint_names,
                               std::vector<double>* joint_values) const {
  return false;
}
}  // namespace pbd
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
