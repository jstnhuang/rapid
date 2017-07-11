#include "rapid_pbd/motion_planning.h"

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Landmark.h"

#include "ros/ros.h"

using std::string;

namespace rapid {
namespace pbd {
MotionPlanning::MotionPlanning(const RobotConfig& robot_config)
    : robot_config_(robot_config),
      builder_(robot_config.planning_frame(), robot_config.planning_group()),
      num_goals_(0) {}

void MotionPlanning::AddPoseGoal(const string& actuator_group,
                                 const geometry_msgs::Pose& pose,
                                 const rapid_pbd_msgs::Landmark& landmark) {
  string ee_link = robot_config_.ee_frame_for_group(actuator_group);
  if (ee_link == "") {
    ROS_ERROR("Unable to look up EE link for actuator group \"%s\"",
              actuator_group.c_str());
    return;
  }

  builder_.AddPoseGoal(ee_link, pose);
  ++num_goals_;
}

void MotionPlanning::AddJointGoal(const string& actuator_group,
                                  const geometry_msgs::Pose& pose) {
  // TODO: use moveit instead of joint controllers
}

void MotionPlanning::ClearGoals() {
  std::map<string, geometry_msgs::Pose> goals;
  // Overrides joint goals if any and deletes pose goals.
  builder_.SetPoseGoals(goals);
  num_goals_ = 0;
}

void MotionPlanning::BuildGoal(moveit_msgs::MoveGroupGoal* goal) const {
  builder_.Build(goal);
}

int MotionPlanning::num_goals() const { return num_goals_; }
}  // namespace pbd
}  // namespace rapid
