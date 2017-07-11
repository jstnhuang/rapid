#include "rapid_pbd/robot_config.h"

#include "rapid_pbd_msgs/Action.h"

using rapid_pbd_msgs::Action;

namespace rapid {
namespace pbd {
void Pr2RobotConfig::planning_frame(std::string* planning_frame) {
  *planning_frame = "base_link";
}
void Pr2RobotConfig::base_link(std::string* torso_link) {
  *torso_link = "base_link";
}
void Pr2RobotConfig::torso_link(std::string* torso_link) {
  *torso_link = "torso_lift_link";
}
void Pr2RobotConfig::ee_frame_for_group(const std::string& actuator_group,
                                        std::string* ee_frame) {
  if (actuator_group == Action::LEFT_ARM) {
    *ee_frame = "l_wrist_roll_link";
  } else if (actuator_group == Action::RIGHT_ARM) {
    *ee_frame = "r_wrist_roll_link";
  }
}
void Pr2RobotConfig::joints_for_group(const std::string& actuator_group,
                                      std::vector<std::string>* joint_names) {
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
int Pr2RobotConfig::num_arms() { return 2; }
double Pr2RobotConfig::gripper_open_position() { return 0.08; }
}  // namespace pbd
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
