#ifndef _RAPID_MANIPULATION_PICK_PLACE_H_
#define _RAPID_MANIPULATION_PICK_PLACE_H_

#include <string>

#include "agile_grasp/FindGrasps.h"
#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_perception/scene.h"
#include "rapid_ros/publisher.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace manipulation {
// TODO(jstn): make this pick with whichever arm is free
class Picker {
 public:
  // Picker does not take ownership of arm or gripper.
  Picker(ArmInterface* arm, GripperInterface* gripper);
  ~Picker();

  // Updates the MoveIt! planning scene to match the given Scene.
  void UpdatePlanningScene(rapid::perception::Scene& scene);

  // Grasps the given object. The gripper will move to a pre-grasp pose, then
  // execute a grasp. It does not do a post-grasp action.
  //
  // If max_effort is negative, the robot will grasp with full strength. If the
  // robot has a pressure sensor, then set max_effort to the max force, in
  // Newtons, to exert. 30N is a good starting point to grasp "gently."
  //
  // Returns false if the grasp failed, true if successful. A grasp is
  // considered successful if the gripper moved to the grasp pose and closed. It
  // does not verify that the object actually firmly in hand.
  bool Pick(const rapid::perception::Object& obj, double max_effort = -1);

 private:
  void UpdatePlanningSceneTopic(const std::string& id,
                                const moveit_msgs::CollisionObject& obj);
  ros::NodeHandle nh_;
  ros::Publisher co_pub_;
  ros::Publisher ps_pub_;
  rapid::viz::MarkerPub* marker_pub_;
  ros::ServiceClient grasp_client_;
  tf::TransformListener tf_listener_;
  rapid::perception::Scene scene_;
  ArmInterface* const arm_;
  GripperInterface* const gripper_;
};

class Placer {
 public:
  Placer(ArmInterface* arm, GripperInterface* gripper);
  ~Placer();
  bool Place(const rapid::perception::Object& obj,
             const rapid::perception::HSurface& table);

 private:
  ros::NodeHandle nh_;
  rapid::viz::MarkerPub* marker_pub_;
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
bool SampleRandomPlacement(const geometry_msgs::Vector3& object_scale,
                           const rapid::perception::HSurface& table,
                           geometry_msgs::PointStamped* location);

// Given the grasp approach and grasp axis (from agile_grasp), compute the
// orientation of the gripper. In general, the approach vector is the vector
// facing the object that represents the +x of the gripper's wrist_roll_link.
// The axis vector is the +z vector of the gripper's wrist_roll_link.
void ComputeGraspOrientation(const geometry_msgs::Vector3& approach,
                             const geometry_msgs::Vector3& axis,
                             geometry_msgs::Quaternion* orientation);
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_PICK_PLACE_H_
