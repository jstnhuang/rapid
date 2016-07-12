#ifndef _RAPID_PERCEPTION_PR2_H_
#define _RAPID_PERCEPTION_PR2_H_

#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include "rapid_perception/scene.h"

namespace rapid {
namespace perception {
namespace pr2 {
// Gets the parsed scene from the point cloud of the area in front of the
// robot.
bool GetManipulationScene(const tf::TransformListener& tf_listener,
                          rapid::perception::Scene* scene);
}  // namespace pr2
}  // namespace perception
}  // namespace rapid
#endif  // _RAPID_PERCEPTION_PR2_H_
