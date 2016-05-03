#ifndef _RAPID_MANIPULATION_PICK_H_
#define _RAPID_MANIPULATION_PICK_H_

#include <string>

#include "agile_grasp/FindGrasps.h"
#include "agile_grasp/Grasp.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
//#include "moveit/move_group_interface/move_group.h"
//#include "moveit_msgs/CollisionObject.h"
#include "ros/ros.h"

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_perception/scene.h"
#include "rapid_ros/service_client.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace manipulation {
class Picker {
 public:
  // Picker does not take ownership of arm or gripper.
  Picker(ArmInterface* arm, GripperInterface* gripper,
         rapid_ros::ServiceClientInterface<agile_grasp::FindGrasps>*
             grasp_gen_client,
         rapid::viz::MarkerPub* marker_pub);

  // Updates the MoveIt! planning scene to match the given Scene.
  // void UpdatePlanningScene(rapid::perception::Scene& scene);

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
  bool GenerateGrasps(const rapid::perception::Object& obj,
                      agile_grasp::Grasps* grasps);
  void VisualizeGrasp(const agile_grasp::Grasp& grasp,
                      const std::string& frame);
  bool MoveToPreGrasp(const geometry_msgs::PoseStamped& grasp_pose,
                      const geometry_msgs::Vector3& grasp_approach);
  bool MoveToGrasp(const geometry_msgs::PoseStamped& grasp_pose,
                   const geometry_msgs::Vector3& grasp_approach);

 private:
  // void UpdatePlanningSceneTopic(const std::string& id,
  //                              const moveit_msgs::CollisionObject& obj);
  // ros::Publisher co_pub_;
  // ros::Publisher ps_pub_;
  rapid_ros::ServiceClientInterface<agile_grasp::FindGrasps>* grasp_gen_client_;
  rapid::viz::MarkerPub* const marker_pub_;
  rapid::perception::Scene scene_;
  ArmInterface* const arm_;
  GripperInterface* const gripper_;
};

// Given the grasp approach and grasp axis (from agile_grasp), compute the
// orientation of the gripper. In general, the approach vector is the vector
// facing the object that represents the +x of the gripper's wrist_roll_link.
// The axis vector is the +z vector of the gripper's wrist_roll_link.
void ComputeGraspOrientation(const geometry_msgs::Vector3& approach,
                             const geometry_msgs::Vector3& axis,
                             geometry_msgs::Quaternion* orientation);
}  // namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_PICK_H_
