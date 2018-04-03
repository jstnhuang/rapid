#include "rapid_pr2/head.h"

#include <math.h>

#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace rapid {
namespace pr2 {
Head::Head(const rapid::JointStateReader& js_reader)
    : client_(kPanTiltAction), js_reader_(js_reader) {}

bool Head::StartPanTilt(const Radians& pan, const Radians& tilt) {
  if (!client_.waitForServer(ros::Duration(1.0))) {
    ROS_ERROR("Unable to connect to head server %s.", kPanTiltAction);
    return false;
  }

  double pan_rads = fmod(pan.value() + M_PI, 2 * M_PI) - M_PI;
  if (pan_rads < kMinPanRadians) {
    ROS_WARN("Invalid head pan argument %f, using %f", pan_rads,
             kMinPanRadians);
    pan_rads = kMinPanRadians;
  } else if (pan_rads > kMaxPanRadians) {
    ROS_WARN("Invalid head pan argument %f, using %f", pan_rads,
             kMaxPanRadians);
    pan_rads = kMaxPanRadians;
  }

  double tilt_rads = fmod(tilt.value() + M_PI, 2 * M_PI) - M_PI;
  if (tilt_rads < kMinTiltRadians) {
    ROS_WARN("Invalid head tilt argument %f, using %f", tilt_rads,
             kMinTiltRadians);
    tilt_rads = kMinTiltRadians;
  } else if (tilt_rads > kMaxTiltRadians) {
    ROS_WARN("Invalid head tilt argument %f, using %f", tilt_rads,
             kMaxTiltRadians);
    tilt_rads = kMaxTiltRadians;
  }

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(pan_rads);
  point.positions.push_back(tilt_rads);
  bool success = js_reader_.WaitForJoint(kPanJoint, ros::Duration(1));
  if (!success) {
    ROS_ERROR("Failed to read current head pan position!");
    return false;
  }
  success = js_reader_.WaitForJoint(kTiltJoint, ros::Duration(1));
  if (!success) {
    ROS_ERROR("Failed to read current head tilt position!");
    return false;
  }
  double current_pan = js_reader_.position(kPanJoint);
  double pan_distance = fabs(current_pan - pan_rads);
  double current_tilt = js_reader_.position(kTiltJoint);
  double tilt_distance = fabs(current_tilt - tilt_rads);
  double pan_time = pan_distance / kMaxPanVel;
  double tilt_time = tilt_distance / kMaxTiltVel;
  point.time_from_start = ros::Duration(2 * std::max(pan_time, tilt_time));

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back(kPanJoint);
  goal.trajectory.joint_names.push_back(kTiltJoint);
  goal.trajectory.points.push_back(point);
  client_.sendGoal(goal);

  return true;
}

bool Head::IsDone() const { return client_.getState().isDone(); }

void Head::Cancel() { client_.cancelAllGoals(); }
}  // namespace pr2
}  // namespace rapid
