#ifndef _RAPID_MANIPULATION_PICK_PLACE_H_
#define _RAPID_MANIPULATION_PICK_PLACE_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "moveit/move_group_interface/move_group.h"
#include "moveit_msgs/CollisionObject.h"
#include "ros/ros.h"

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
  PickError(const std::string& error) : error_(error) {}
  const static std::string SUCCESS;  // Pick was successful.

 private:
  std::string error_;
};

class Picker {
 public:
  Picker(boost::shared_ptr<moveit::planning_interface::MoveGroup> group);

  // Updates the MoveIt! planning scene to match the given Scene.
  void UpdatePlanningScene(rapid::perception::Scene& scene);

  // Picks up an object. The object is given by the name of the CollisionObject,
  // which must be published to MoveIt beforehand. Likewise, the support surface
  // of the object is given by name.
  PickError Pick(const std::string& obj_name, const std::string& support_name);

 private:
  void UpdatePlanningSceneTopic(const std::string& id,
                                const moveit_msgs::CollisionObject& obj);
  boost::shared_ptr<moveit::planning_interface::MoveGroup> group_;
  ros::NodeHandle nh_;
  ros::Publisher co_pub_;
  ros::Publisher ps_pub_;
};
}  // namespace manipulation
}  // namespace rapid
#endif  // _RAPID_MANIPULATION_PICK_PLACE_H_
