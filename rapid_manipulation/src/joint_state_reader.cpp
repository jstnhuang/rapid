#include "rapid_manipulation/joint_state_reader.h"

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
namespace manipulation {
JointStateReader::JointStateReader()
    : nh_(), topic_("joint_states"), positions_() {}
JointStateReader::JointStateReader(const std::string& joint_states_topic)
    : nh_(), topic_(joint_states_topic), positions_() {}

void JointStateReader::Start() {
  sub_ = nh_.subscribe(topic_, 5, &JointStateReader::callback, this);
}

double JointStateReader::get_position(const std::string& name) const {
  if (positions_.find(name) == positions_.end()) {
    return kNoJointValue;
  }
  return positions_.at(name);
}

void JointStateReader::get_positions(const std::vector<std::string>& names,
                                     std::vector<double>* positions) const {
  positions->clear();
  for (size_t i = 0; i < names.size(); ++i) {
    positions->push_back(get_position(names[i]));
  }
}

void JointStateReader::callback(const sensor_msgs::JointState& js) {
  if (js.name.size() < js.position.size()) {
    ROS_WARN("JointState msg had different sized name and position field.");
    return;
  }
  for (size_t i = 0; i < js.position.size(); ++i) {
    positions_[js.name[i]] = js.position[i];
  }
}
}  // namespace manipulation
}  // namespace rapid
