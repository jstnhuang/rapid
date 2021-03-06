#ifndef _RAPID_MANIPULATION_ARM_H_
#define _RAPID_MANIPULATION_ARM_H_

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "gmock/gmock.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"

#include "rapid_manipulation/arm_id.h"

namespace rapid {
namespace manipulation {
// Interface for sending arm navigation goals.
//
// Sample usage:
//   MoveGroup group("right_arm");
//   ArmInterface* right_arm = new MoveItArm(group);
//   right_arm->MoveToPoseGoal(pose, false);
class ArmInterface {
 public:
  virtual ~ArmInterface() {}

  // Sends a goal for the end effector to move to the given pose.
  // refresh_point_cloud tells trajopt to capture a new point cloud before
  // planning. Otherwise, it uses a previously captured point cloud. This arg
  // has not effect on MoveIt.
  virtual bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose) const = 0;
};

// Implements arm navigation using MoveIt.
class MoveItArm : public ArmInterface {
 private:
  boost::shared_ptr<moveit::planning_interface::MoveGroup> group_;

 public:
  MoveItArm(boost::shared_ptr<moveit::planning_interface::MoveGroup> group);
  MoveItArm(ArmId id);
  ~MoveItArm(){};
  bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose) const;
};

class MockArm : public ArmInterface {
 public:
  MockArm() {}
  MOCK_CONST_METHOD1(MoveToPoseGoal,
                     bool(const geometry_msgs::PoseStamped& pose));
};
}  // namespace manipulation
}  // namespace rapid

#endif  // _RAPID_MANIPULATION_ARM_H_
