#ifndef _RAPID_PBD_VIZ_SERVER_H_
#define _RAPID_PBD_VIZ_SERVER_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "rapid_pbd_msgs/EditorEvent.h"
#include "rapid_pbd_msgs/Program.h"
#include "robot_markers/builder.h"
#include "ros/ros.h"

#include "rapid_pbd/joint_state.h"
#include "rapid_pbd/program_db.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {

typedef std::pair<std::string, int> ProgramStep;

struct StepVisualization {
 public:
  size_t step_id;
  ros::Publisher robot_pub;
  ros::Publisher scene_pub;
  ros::Publisher surface_seg_pub;
};

// Visualization server for PbD programs.
class Visualizer {
 public:
  Visualizer(const SceneDb& scene_db,
             const robot_markers::Builder& marker_builder);
  void Init();

  // Publish the visualization for a particular step.
  void Publish(const std::string& program_id, const World& world);

  void StopPublishing(const std::string& program_id);

 private:
  void GetSegmentationMarker(const World& world,
                             visualization_msgs::MarkerArray* scene_markers);
  void CreateStepVizIfNotExists(const std::string& program_id);

  const SceneDb scene_db_;
  robot_markers::Builder marker_builder_;
  std::map<std::string, StepVisualization> step_vizs_;

  ros::NodeHandle nh_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_VIZ_SERVER_H_
