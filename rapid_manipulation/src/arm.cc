#include "rapid_manipulation/arm.h"

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"

using geometry_msgs::PoseStamped;
using moveit::planning_interface::MoveGroup;

namespace rapid {
namespace manipulation {
MoveItArm::MoveItArm(boost::shared_ptr<MoveGroup> group) : group_(group) {}
MoveItArm::MoveItArm(ArmId id) {  // : group_() {
  if (id == LEFT) {
    group_.reset(new MoveGroup("left_arm"));
  } else {
    group_.reset(new MoveGroup("right_arm"));
  }
}

bool MoveItArm::MoveToPoseGoal(const PoseStamped& pose) {
  group_->setPoseTarget(pose);
  return group_->move();
}
}  // namespace manipulation
}  // namespace rapid
