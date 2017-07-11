#include "rapid_pbd/robot_config.h"

#include "rapid_pbd_msgs/Action.h"

using rapid_pbd_msgs::Action;

namespace rapid {
namespace pbd {
std::string Pr2RobotConfig::planning_frame() const { return "base_link"; }
std::string Pr2RobotConfig::planning_group() const { return "arms"; }
std::string Pr2RobotConfig::base_link() const { return "base_link"; }
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
void Pr2RobotConfig::joints_for_group(
    const std::string& actuator_group,
    std::vector<std::string>* joint_names) const {
  if (actuator_group == Action::LEFT_ARM) {
    joint_names->clear();
    joint_names->push_back("l_shoulder_pan_joint");
    joint_names->push_back("l_shoulder_lift_joint");
    joint_names->push_back("l_upper_arm_roll_joint");
    joint_names->push_back("l_elbow_flex_joint");
    joint_names->push_back("l_forearm_roll_joint");
    joint_names->push_back("l_wrist_flex_joint");
    joint_names->push_back("l_wrist_roll_joint");
  } else if (actuator_group == Action::RIGHT_ARM) {
    joint_names->clear();
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
double Pr2RobotConfig::gripper_open_position() const { return 0.08; }
}  // namespace pbd
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
