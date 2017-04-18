#ifndef _JOINT_STATE_READER_H_
#define _JOINT_STATE_READER_H_

#include <map>
#include <string>
#include <vector>

#include "sensor_msgs/JointState.h"

namespace rapid {
namespace manipulation {
// JointStateReader tracks the latest joint states.
class JointStateReader {
 public:
  explicit JointStateReader();
  void Callback(const sensor_msgs::JointState& msg);
  double get_position(const std::string& name);
  void get_positions(const std::vector<std::string>& names,
                     std::vector<double>* positions);

 private:
  std::map<std::string, double> positions_;
};
}  // namespace manipulation
}  // namespace rapid

#endif  // _JOINT_STATE_READER_H_
