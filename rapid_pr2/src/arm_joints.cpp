#include "rapid_pr2/arm_joints.h"

#include <string>
#include <vector>

namespace rapid {
namespace pr2 {
std::vector<std::string> left_arm_joint_names() {
  std::vector<std::string> names;
  names.push_back("l_shoulder_pan_joint");
  names.push_back("l_shoulder_lift_joint");
  names.push_back("l_upper_arm_roll_joint");
  names.push_back("l_elbow_flex_joint");
  names.push_back("l_forearm_roll_joint");
  names.push_back("l_wrist_flex_joint");
  names.push_back("l_wrist_roll_joint");
  return names;
}

std::vector<std::string> right_arm_joint_names() {
  std::vector<std::string> names;
  names.push_back("r_shoulder_pan_joint");
  names.push_back("r_shoulder_lift_joint");
  names.push_back("r_upper_arm_roll_joint");
  names.push_back("r_elbow_flex_joint");
  names.push_back("r_forearm_roll_joint");
  names.push_back("r_wrist_flex_joint");
  names.push_back("r_wrist_roll_joint");
  return names;
}
}  // namespace pr2
}  // namespace rapid
