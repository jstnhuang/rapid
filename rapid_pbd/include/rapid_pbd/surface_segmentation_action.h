#ifndef _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_
#define _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/visualization.h"

namespace rapid {
namespace pbd {
class SurfaceSegmentationAction {
 public:
  explicit SurfaceSegmentationAction(const std::string& topic);
  void Start();
  void Execute(const rapid_pbd_msgs::SegmentSurfacesGoalConstPtr& goal);

 private:
  std::string topic_;
  actionlib::SimpleActionServer<rapid_pbd_msgs::SegmentSurfacesAction> as_;
  surface_perception::Segmentation seg_;
  ros::NodeHandle nh_;
  ros::Publisher cropped_pub_;
  ros::Publisher marker_pub_;
  surface_perception::SurfaceViz viz_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_SURFACE_SEGMENTATION_ACTION_H_
