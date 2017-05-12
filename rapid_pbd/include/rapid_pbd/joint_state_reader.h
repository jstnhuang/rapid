#ifndef _RAPID_MANIPULATION_JOINT_STATE_READER_H_
#define _RAPID_MANIPULATION_JOINT_STATE_READER_H_

#include <limits.h>
#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace rapid {
namespace manipulation {

// The joint value when a joint name is invalid.
// This assumes that a joint value will never reach this number, although that
// is not necessarily true for continuous joints.
const static double kNoJointValue = std::numeric_limits<double>::min();

// JointStateReader tracks the latest joint states.
class JointStateReader {
 public:
  JointStateReader();
  JointStateReader(const std::string& joint_states_topic);

  void Start();

  // Returns the position of the given joint.
  // If the joint name is valid, returns rapid::manipulation::kNoJointValue.
  double get_position(const std::string& name) const;

  // Returns the positions of the given vector of joints. If a joint name is
  // invalid, then its value will be kNoJointValue.
  void get_positions(const std::vector<std::string>& names,
                     std::vector<double>* positions) const;

 private:
  void callback(const sensor_msgs::JointState& msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_;
  std::map<std::string, double> positions_;
};
}  // namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_JOINT_STATE_READER_H_
