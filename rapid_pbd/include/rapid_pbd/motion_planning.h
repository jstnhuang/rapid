#ifndef _RAPID_PBD_MOTION_PLANNING_H_
#define _RAPID_PBD_MOTION_PLANNING_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Landmark.h"

#include "rapid_pbd/robot_config.h"

namespace rapid {
namespace pbd {
class MotionPlanning {
 public:
  MotionPlanning(const RobotConfig& robot_config);
  void AddPoseGoal(const std::string& actuator_group,
                   const geometry_msgs::Pose& pose,
                   const rapid_pbd_msgs::Landmark& landmark);
  void AddJointGoal(const std::string& actuator_group,
                    const geometry_msgs::Pose& pose);
  void ClearGoals();
  void BuildGoal(moveit_msgs::MoveGroupGoal* goal);

 private:
  const RobotConfig& robot_config_;
  moveit_goal_builder::Builder builder_;
};
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_MOTION_PLANNING_H_
