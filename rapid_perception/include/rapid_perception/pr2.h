#ifndef _RAPID_PERCEPTION_PR2_H_
#define _RAPID_PERCEPTION_PR2_H_

#include "visualization_msgs/Marker.h"
#include "rapid_perception/rgbd.hpp"

namespace rapid {
namespace perception {
namespace pr2 {
// Gets the parsed scene from the point cloud of the area in front of the
// robot. The area the point cloud comes from is given by
// GetManipulationWorkspace.
bool GetManipulationScene(rapid::perception::Scene* scene);

// Returns a box describing the area in front of the robot that it can reach.
void GetManipulationWorkspace(visualization_msgs::Marker* ws);
}  // namespace pr2
}  // namespace perception
}  // namespace rapid
#endif  // _RAPID_PERCEPTION_PR2_H_
