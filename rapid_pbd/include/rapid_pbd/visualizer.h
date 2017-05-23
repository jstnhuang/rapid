#ifndef _RAPID_PBD_VIZ_SERVER_H_
#define _RAPID_PBD_VIZ_SERVER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "robot_state_publisher_latched/robot_state_publisher_latched.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"

namespace rapid {
namespace pbd {
// Visualization server for PbD programs.
// The server publishes the robot state for each step, where each step has its
// own TF prefix.
class Visualizer {
 public:
  Visualizer(const robot_state_publisher_latched::RobotStatePublisher&
                 robot_state_pub);

  // Updates the state of the visualization.
  void HandleUpdate(const rapid_pbd_msgs::EditorEvent& event);

 private:
  std::string TfPrefix(const std::string& db_id, const int step_id);

  robot_state_publisher_latched::RobotStatePublisher robot_state_pub_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_VIZ_SERVER_H_
