#ifndef _RAPID_PR2_JOINT_STATES_H_
#define _RAPID_PR2_JOINT_STATES_H_

#include <map>
#include <string>

namespace rapid {
namespace pr2 {
// JointStates holds default joint values for the PR2.
// These can be replaced as needed, and passed to a RobotStatePublisher.
// The default joint values are with the wheels facing forward, the robot
// looking straight ahead, and with both arms outstretched.
class JointStates {
 public:
  JointStates();
  void Set(const std::map<std::string, double>& joint_positions);
  std::map<std::string, double> joint_positions() const;

 private:
  void Initialize();
  std::map<std::string, double> joint_positions_;
};
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_PR2_JOINT_STATES_H_
