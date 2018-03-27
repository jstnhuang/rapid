#include "rapid_robot/joint_state_reader.h"

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
JointStateReader::JointStateReader()
    : nh_(), topic_("joint_states"), positions_(), received_callback_(false) {}
JointStateReader::JointStateReader(const std::string& joint_states_topic)
    : nh_(),
      topic_(joint_states_topic),
      positions_(),
      received_callback_(false) {}

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

bool JointStateReader::WaitForMessages(const ros::Duration& timeout) const {
  ros::Time start = ros::Time::now();
  while (ros::ok() && ros::Time::now() - start < timeout) {
    if (received_callback_) {
      return true;
    }
    ros::spinOnce();
  }
  return false;
}

bool JointStateReader::WaitForJoint(const std::string& name,
                                    const ros::Duration& timeout) const {
  ros::Time start = ros::Time::now();
  while (ros::ok() && ros::Time::now() - start < timeout) {
    if (HasJoint(name)) {
      return true;
    }
    ros::spinOnce();
  }
  return false;
}

void JointStateReader::callback(const sensor_msgs::JointState& js) {
  received_callback_ = true;
  if (js.name.size() < js.position.size()) {
    ROS_ERROR("JointState msg had different sized name and position field.");
    return;
  }
  for (size_t i = 0; i < js.position.size(); ++i) {
    positions_[js.name[i]] = js.position[i];
  }
}
}  // namespace rapid
