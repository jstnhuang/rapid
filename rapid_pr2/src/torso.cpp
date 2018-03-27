#include "rapid_pr2/torso.h"

#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace rapid {
namespace pr2 {
Torso::Torso() : client_(kTorsoAction) {}

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
  // TODO: add time here.

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
