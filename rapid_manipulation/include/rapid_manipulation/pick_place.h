#ifndef _RAPID_MANIPULATION_PICK_PLACE_H_
#define _RAPID_MANIPULATION_PICK_PLACE_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/CollisionObject.h"
#include "rapid_perception/scene.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"

namespace rapid {
namespace perception {
// Forward declarations to avoid circular deps.
class Object;
class Scene;
class Tabletop;
}  // namespace perception

namespace manipulation {
class Picker {
 public:
  Picker(const ArmInterface& arm, const GripperInterface& gripper);

  // Updates the MoveIt! planning scene to match the given Scene.
  void UpdatePlanningScene(rapid::perception::Scene& scene);

  // Picks up an object. The object is given by the name of the CollisionObject,
  // which must be published to MoveIt beforehand. Likewise, the support surface
  // of the object is given by name.
  bool Pick(const rapid::perception::Object& obj);

 private:
  void UpdatePlanningSceneTopic(const std::string& id,
                                const moveit_msgs::CollisionObject& obj);
  ros::NodeHandle nh_;
  ros::Publisher co_pub_;
  ros::Publisher ps_pub_;
  tf::TransformListener tf_listener_;
  rapid::perception::Scene scene_;
  const ArmInterface& arm_;
  const GripperInterface& gripper_;
};

class Placer {
 public:
  Placer(const ArmInterface& arm, const GripperInterface& gripper);
  bool Place(const rapid::perception::Object& obj,
             const rapid::perception::Tabletop& table);

 private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  const ArmInterface& arm_;
  const GripperInterface& gripper_;
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
bool SampleRandomPlacement(const geometry_msgs::Vector3& object_scale,
                           const rapid::perception::Tabletop& table,
                           geometry_msgs::PointStamped* location);
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_PICK_PLACE_H_
