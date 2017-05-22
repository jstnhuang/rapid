#ifndef _RAPID_PBD_JOINT_STATE_READER_H_
#define _RAPID_PBD_JOINT_STATE_READER_H_

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include "rapid_pbd/joint_state.h"

namespace rapid {
namespace pbd {
// JointStateReader tracks the latest joint states.
class JointStateReader {
 public:
  JointStateReader();
  JointStateReader(const std::string& topic);

  void Callback(const sensor_msgs::JointState& msg);
  void Start();

  // Returns the position of the given joint.
  // If the joint name is invalid, returns rapid::manipulation::kNoJointValue.
  double get_position(const std::string& name) const;

  // Returns the positions of the given vector of joints. If a joint name is
  // invalid, then its value will be kNoJointValue.
  void get_positions(const std::vector<std::string>& names,
                     std::vector<double>* positions) const;

  // Puts the latest joint states into a sensor_msgs::JointState msg.
  void ToMsg(sensor_msgs::JointState* msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_;
  JointState joint_state_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_JOINT_STATE_READER_H_
