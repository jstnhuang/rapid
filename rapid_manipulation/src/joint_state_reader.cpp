#include "rapid_manipulation/joint_state_reader.h"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
namespace manipulation {
JointStateReader::JointStateReader() : positions_() {}

double JointStateReader::get_position(const std::string& name) {
  if (positions_.find(name) == positions_.end()) {
    throw std::out_of_range("No position value for name: " + name);
  }
  return positions_[name];
}

void JointStateReader::get_positions(const std::vector<std::string>& names,
                                     std::vector<double>* positions) {
  positions->clear();
  for (size_t i = 0; i < names.size(); ++i) {
    positions->push_back(get_position(names[i]));
  }
}

void JointStateReader::Callback(const sensor_msgs::JointState& js) {
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
