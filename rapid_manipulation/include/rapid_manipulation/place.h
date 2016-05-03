#ifndef _RAPID_MANIPULATION_PLACE_H_
#define _RAPID_MANIPULATION_PLACE_H_

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace manipulation {
class Placer {
 public:
  Placer(ArmInterface* arm, GripperInterface* gripper,
         rapid::viz::MarkerPub* marker_pub);
  bool Place(const rapid::perception::Object& obj,
             const rapid::perception::HSurface& table);
  void VisualizePlacement(const geometry_msgs::PoseStamped& obj_ps,
                          const geometry_msgs::Vector3& obj_scale);

 private:
  rapid::viz::MarkerPub* const marker_pub_;
  ArmInterface* const arm_;
  GripperInterface* const gripper_;
};

// Samples a random location on the given tabletop, such that the object won't
// intersect with other objects on the table. The pose where the object
// should be placed is given in location. location will be in the same frame as
// the table. The location refers to where the midpoint of the object
// (equivalent to the position of the object) needs to be once the object has
// been placed. For example, if the midpoint of the table is at z=0, the
// table is 4cm tall, and the object is 7cm tall, then the z of the location
// will be 2+3.5 = 5.5cm
//
// Returns false if no placement could be found.
bool SampleRandomPlacement(double obj_height,
                           const rapid::perception::HSurface& table,
                           geometry_msgs::PointStamped* location);
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_PLACE_H_
