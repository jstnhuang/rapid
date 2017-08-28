#include "rapid_pbd/joint_state_reader.h"

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "rapid_pbd/joint_state.h"

namespace rapid {
namespace pbd {
JointStateReader::JointStateReader()
    : nh_(), topic_("/joint_states"), joint_state_() {}
JointStateReader::JointStateReader(const std::string& joint_states_topic)
    : nh_(), topic_(joint_states_topic), joint_state_() {}

void JointStateReader::Start() {
  sub_ = nh_.subscribe(topic_, 5, &JointStateReader::Callback, this);
}

double JointStateReader::get_position(const std::string& name) const {
  return joint_state_.position(name);
}

void JointStateReader::get_positions(const std::vector<std::string>& names,
                                     std::vector<double>* positions) const {
  positions->clear();
  for (size_t i = 0; i < names.size(); ++i) {
    positions->push_back(get_position(names[i]));
  }
}

void JointStateReader::ToMsg(sensor_msgs::JointState* msg) {
  joint_state_.ToMsg(msg);
}

void JointStateReader::Callback(const sensor_msgs::JointState& js) {
  if (js.name.size() != js.position.size()) {
    ROS_ERROR_THROTTLE(
        1, "JointState msg had different sized name and position field.");
    return;
  }
  for (size_t i = 0; i < js.name.size(); ++i) {
    joint_state_.SetPosition(js.name[i], js.position[i]);
  }
}
}  // namespace pbd
}  // namespace rapid
