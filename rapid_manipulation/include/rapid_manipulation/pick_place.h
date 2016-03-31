#ifndef _RAPID_MANIPULATION_PICK_PLACE_H_
#define _RAPID_MANIPULATION_PICK_PLACE_H_

#include <string>

#include "boost/shared_ptr.hpp"
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
// Error code for picking.
class PickError {
 public:
  explicit PickError(const std::string& error) : error_(error) {}
  const static std::string SUCCESS;  // Pick was successful.
  const static std::string
      OBJ_NOT_FOUND;  // The given object name was not in the planning scene.
  const static std::string PRE_GRASP_FAILED;
  const static std::string GRASP_FAILED;
  const static std::string POST_GRASP_FAILED;
  std::string error() const { return error_; }

 private:
  std::string error_;
};

class Picker {
 public:
  Picker(boost::shared_ptr<ArmInterface> arm,
         boost::shared_ptr<GripperInterface> gripper);

  // Updates the MoveIt! planning scene to match the given Scene.
  void UpdatePlanningScene(rapid::perception::Scene& scene);

  // Picks up an object. The object is given by the name of the CollisionObject,
  // which must be published to MoveIt beforehand. Likewise, the support surface
  // of the object is given by name.
  PickError Pick(const std::string& obj_name, const std::string& support_name);

 private:
  void UpdatePlanningSceneTopic(const std::string& id,
                                const moveit_msgs::CollisionObject& obj);
  // boost::shared_ptr<moveit::planning_interface::MoveGroup> group_;
  ros::NodeHandle nh_;
  ros::Publisher co_pub_;
  ros::Publisher ps_pub_;
  tf::TransformListener tf_listener_;
  rapid::perception::Scene scene_;
  boost::shared_ptr<ArmInterface> arm_;
  boost::shared_ptr<GripperInterface> gripper_;
};
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_PICK_PLACE_H_
