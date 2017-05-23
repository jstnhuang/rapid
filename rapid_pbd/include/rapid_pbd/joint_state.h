#ifndef _RAPID_PBD_JOINT_STATE_H_
#define _RAPID_PBD_JOINT_STATE_H_

#include <limits.h>
#include <map>
#include <string>

#include "sensor_msgs/JointState.h"

namespace rapid {
namespace pbd {
// The joint value when a joint name is invalid.
// This assumes that a joint value will never reach this number, although that
// is not necessarily true for continuous joints.
const static double kNoJointValue = std::numeric_limits<double>::min();

// A wrapper around a sensor_msgs::JointState that allows you to edit positions
// like a map.
class JointState {
 public:
  JointState();
  explicit JointState(const sensor_msgs::JointState& joint_state);

  bool HasJoint(const std::string& name) const;

  void SetPosition(const std::string& name, double position);

  double position(const std::string& name) const;

  void ToMsg(sensor_msgs::JointState* msg) const;
  void ToMap(std::map<std::string, double>* joint_positions) const;

 private:
  std::map<std::string, double> positions_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _JOINT_STATE_H_
