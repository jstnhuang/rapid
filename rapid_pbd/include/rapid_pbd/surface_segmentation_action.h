#ifndef _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_
#define _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/visualization.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/program_db.h"
#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
class SurfaceSegmentationAction {
 public:
  SurfaceSegmentationAction(const std::string& topic, const SceneDb& scene_db,
                            const RobotConfig& robot_config);
  void Start();
  void Execute(const rapid_pbd_msgs::SegmentSurfacesGoalConstPtr& goal);

 private:
  std::string topic_;
  SceneDb scene_db_;
  const RobotConfig& robot_config_;
  actionlib::SimpleActionServer<rapid_pbd_msgs::SegmentSurfacesAction> as_;
  surface_perception::Segmentation seg_;
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  surface_perception::SurfaceViz viz_;
  tf::TransformListener tf_listener_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_
