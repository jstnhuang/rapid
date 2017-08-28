#ifndef _RAPID_PBD_MOTION_PLANNING_H_
#define _RAPID_PBD_MOTION_PLANNING_H_

#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "moveit_msgs/MoveItErrorCodes.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "rapid_pbd/joint_state_reader.h"
#include "rapid_pbd/robot_config.h"
#include "rapid_pbd/world.h"

namespace rapid {
namespace pbd {
class MotionPlanning {
 public:
  MotionPlanning(const RobotConfig& robot_config, World* world,
                 const tf::TransformListener& tf_listener,
                 const ros::Publisher& planning_scene_pub,
                 const JointStateReader& js_reader);
  // Returns an error message, or empty string if no error.
  // Set seed_joint_names and seed_joint_positions to empty vectors if you do
  // not want to specify an IK seed.
  std::string AddPoseGoal(const std::string& actuator_group,
                          const geometry_msgs::Pose& pose,
                          const rapid_pbd_msgs::Landmark& landmark,
                          const std::vector<std::string>& seed_joint_names,
                          const std::vector<double>& seed_joint_positions);
  std::string AddJointGoal(const std::vector<std::string>& joint_names,
                           const std::vector<double>& joint_positions);
  void ClearGoals();
  void BuildGoal(moveit_msgs::MoveGroupGoal* goal) const;
  int num_goals() const;

 private:
  const RobotConfig& robot_config_;
  World* world_;
  const tf::TransformListener& tf_listener_;
  ros::Publisher planning_scene_pub_;
  moveit_goal_builder::Builder builder_;
  int num_goals_;
  // All motions are expressed as joint angle goals.
  std::map<std::string, double> current_joint_goal_;
  const JointStateReader& joint_state_reader_;
};

std::string ErrorCodeToString(const moveit_msgs::MoveItErrorCodes& code);
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_MOTION_PLANNING_H_
