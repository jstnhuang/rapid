#include "rapid_pbd/joint_state.h"

#include <map>
#include <string>

#include "sensor_msgs/JointState.h"

namespace rapid {
namespace pbd {
JointState::JointState() : positions_() {}

JointState::JointState(const sensor_msgs::JointState& joint_state)
    : positions_() {
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    positions_[joint_state.name[i]] = joint_state.position[i];
  }
}

bool JointState::HasJoint(const std::string& name) const {
  return positions_.find(name) != positions_.end();
}

void JointState::SetPosition(const std::string& name, double position) {
  positions_[name] = position;
}

double JointState::position(const std::string& name) const {
  if (!HasJoint(name)) {
    return kNoJointValue;
  } else {
    return positions_.at(name);
  }
}

void JointState::ToMsg(sensor_msgs::JointState* msg) const {
  for (std::map<std::string, double>::const_iterator it = positions_.begin();
       it != positions_.end(); ++it) {
    msg->name.push_back(it->first);
    msg->position.push_back(it->second);
  }
}

void JointState::ToMap(std::map<std::string, double>* joint_positions) const {
  *joint_positions = positions_;
}
}  // namespace pbd
}  // namespace rapid
