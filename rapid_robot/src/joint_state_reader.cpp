#include "rapid_robot/joint_state_reader.h"

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
JointStateReader::JointStateReader()
    : nh_(), topic_("joint_states"), positions_() {}
JointStateReader::JointStateReader(const std::string& joint_states_topic)
    : nh_(), topic_(joint_states_topic), positions_() {}

void JointStateReader::Start() {
  sub_ = nh_.subscribe(topic_, 10, &JointStateReader::callback, this);
}

bool JointStateReader::HasJoint(const std::string& name) const {
  return positions_.find(name) != positions_.end();
}

double JointStateReader::position(const std::string& name) const {
  return positions_.at(name);
}

std::map<std::string, double> JointStateReader::positions() const {
  return positions_;
}

void JointStateReader::callback(const sensor_msgs::JointState& js) {
  if (js.name.size() < js.position.size()) {
    ROS_ERROR("JointState msg had different sized name and position field.");
    return;
  }
  for (size_t i = 0; i < js.position.size(); ++i) {
    positions_[js.name[i]] = js.position[i];
  }
}
}  // namespace rapid
