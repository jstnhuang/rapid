#include "rapid_pbd/motion_planning.h"

#include <map>
#include <string>

#include "geometry_msgs/Pose.h"
#include "moveit_goal_builder/builder.h"
#include "moveit_msgs/MoveGroupAction.h"
#include "rapid_pbd_msgs/Landmark.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "transform_graph/graph.h"

#include "rapid_pbd/errors.h"
#include "rapid_pbd/world.h"

using std::string;
namespace msgs = rapid_pbd_msgs;
namespace tg = transform_graph;

namespace rapid {
namespace pbd {
MotionPlanning::MotionPlanning(const RobotConfig& robot_config, World* world,
                               const tf::TransformListener& tf_listener)
    : robot_config_(robot_config),
      world_(world),
      tf_listener_(tf_listener),
      builder_(robot_config.planning_frame(), robot_config.planning_group()),
      num_goals_(0) {}

string MotionPlanning::AddPoseGoal(const string& actuator_group,
                                   const geometry_msgs::Pose& pose,
                                   const rapid_pbd_msgs::Landmark& landmark) {
  string ee_link = robot_config_.ee_frame_for_group(actuator_group);
  if (ee_link == "") {
    ROS_ERROR("Unable to look up EE link for actuator group \"%s\"",
              actuator_group.c_str());
    return "Invalid actuator in the program.";
  }

  tg::Graph graph;
  graph.Add("ee", tg::RefFrame("landmark"), pose);
  if (landmark.type == msgs::Landmark::TF_FRAME) {
    // TODO: get the latest tf
  } else if (landmark.type == msgs::Landmark::SURFACE_BOX) {
    // TODO: Register the landmark
  } else {
    ROS_ERROR("Unsupported landmark type \"%s\"", landmark.type.c_str());
    return "Unsupported landmark type.";
  }

  // Transform pose into base frame

  builder_.AddPoseGoal(ee_link, pose);
  ++num_goals_;

  return "";
}

void MotionPlanning::AddJointGoal(const string& actuator_group,
                                  const geometry_msgs::Pose& pose) {
  // TODO: use moveit instead of joint controllers
  ROS_ERROR("MotionPlanning::AddJointGoal not implemented");
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
