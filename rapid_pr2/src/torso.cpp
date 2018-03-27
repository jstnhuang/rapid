#include "rapid_pr2/torso.h"

#include <cmath>

#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace rapid {
namespace pr2 {
Torso::Torso(const rapid::JointStateReader& js_reader)
    : client_(kTorsoAction), js_reader_(js_reader) {}

bool Torso::StartMoving(double height) {
  if (!client_.waitForServer(ros::Duration(1.0))) {
    ROS_ERROR("Unable to connect to torso server %s.", kTorsoAction);
    return false;
  }
  if (height < kMinHeight) {
    height = kMinHeight;
  } else if (height > kMaxHeight) {
    height = kMaxHeight;
  }

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(height);
  bool success = js_reader_.WaitForJoint(kTorsoJoint, ros::Duration(1));
  if (!success) {
    ROS_ERROR("Failed to read current torso position!");
    return false;
  }
  double current = js_reader_.position(kTorsoJoint);
  double distance = fabs(current - height);
  point.time_from_start = ros::Duration(distance / kMaxVel);

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back(kTorsoJoint);
  goal.trajectory.points.push_back(point);
  client_.sendGoal(goal);

  return true;
}

bool Torso::IsDone() const { return client_.getState().isDone(); }

void Torso::Cancel() { client_.cancelAllGoals(); }
}  // namespace pr2
}  // namespace rapid
