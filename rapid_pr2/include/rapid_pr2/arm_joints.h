#ifndef _RAPID_MANIPULATION_ARM_JOINTS_H_
#define _RAPID_MANIPULATION_ARM_JOINTS_H_

#include <string>
#include <vector>

namespace rapid {
namespace pr2 {
std::vector<std::string> left_arm_joint_names();
std::vector<std::string> right_arm_joint_names();
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_ARM_JOINTS_H_
