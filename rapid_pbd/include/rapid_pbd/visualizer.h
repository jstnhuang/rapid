#ifndef _RAPID_PBD_VIZ_SERVER_H_
#define _RAPID_PBD_VIZ_SERVER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "robot_markers/builder.h"
#include "ros/ros.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"

namespace rapid {
namespace pbd {

typedef std::pair<std::string, int> ProgramStep;

struct StepVisualization {
 public:
  visualization_msgs::MarkerArray robot_arr;
};

// Visualization server for PbD programs.
class Visualizer {
 public:
  Visualizer(const robot_markers::Builder& marker_builder);
  void Init();

  // Updates the state of the visualization.
  void HandleUpdate(const rapid_pbd_msgs::EditorEvent& event);

  // Publish the visualization for a particular step.
  void Publish(const std::string& program_id, int step_num);

 private:
  robot_markers::Builder marker_builder_;
  std::map<ProgramStep, StepVisualization> step_vizs_;
  std::map<std::string, ros::Publisher> program_pubs_;

  ros::NodeHandle nh_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_VIZ_SERVER_H_
