#ifndef _RAPID_PBD_MOTION_PLANNING_H_
#define _RAPID_PBD_MOTION_PLANNING_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
class MotionPlanning {
 public:
  MotionPlanning(const RobotConfig& robot_config, World* world,
                 const tf::TransformListener& tf_listener);
  // Returns an error message, or empty string if no error.
  std::string AddPoseGoal(const std::string& actuator_group,
                          const geometry_msgs::Pose& pose,
                          const rapid_pbd_msgs::Landmark& landmark);
  void AddJointGoal(const std::string& actuator_group,
                    const geometry_msgs::Pose& pose);
  void ClearGoals();
  void BuildGoal(moveit_msgs::MoveGroupGoal* goal) const;
  int num_goals() const;

 private:
  const RobotConfig& robot_config_;
  World* world_;
  const tf::TransformListener& tf_listener_;
  moveit_goal_builder::Builder builder_;
  int num_goals_;
};

std::string ErrorCodeToString(const moveit_msgs::MoveItErrorCodes& code);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_MOTION_PLANNING_H_
