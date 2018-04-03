#ifndef _RAPID_PR2_HEAD_H_
#define _RAPID_PR2_HEAD_H_

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "rapid_robot/joint_state_reader.h"
#include "rapid_utils/deg_rad.h"

namespace rapid {
namespace pr2 {
/// \brief High-level interface for the PR2's head.
///
/// \b Example:
/// \code
///   #include "rapid_robot/joint_state_reader.h"
///   #include "rapid_pr2/head.h"
///
///   rapid::JointStateReader js_reader;
///   js_reader.Start();
///   rapid::pr2::Head head(js_reader);
///   head.StartPanTilt(rapid::Degrees(0), rapid::Degrees(45));
///   while (ros::ok() && !head.IsDone()) {
///     ros::spinOnce();
///   }
/// \endcode
class Head {
 public:
  static const double kMinPanDegrees;   /// The minimum pan angle, in degrees.
  static const double kMaxPanDegrees;   /// The maximum pan angle, in degrees.
  static const double kMinTiltDegrees;  /// The minimum tilt angle, in degrees.
  static const double kMaxTiltDegrees;  /// The maximum tilt angle, in degrees.
  static const double kMinPanRadians;   /// The minimum pan angle, in radians.
  static const double kMaxPanRadians;   /// The maximum pan angle, in radians.
  static const double kMinTiltRadians;  /// The minimum tilt angle, in radians.
  static const double kMaxTiltRadians;  /// The maximum tilt angle, in radians.

  /// Constructor.
  explicit Head(const rapid::JointStateReader& js_reader);

  /// \brief Starts moving the head to the given pan/tilt angle.
  ///
  /// The head will move at half the maximum velocity towards the goal.
  /// If this method fails to connect to the head action server or read the
  /// current head joint values, it will return false.
  ///
  /// \param[in] pan The pan angle, given as rapid::Degrees or rapid::Radians.
  /// \param[in] tilt The tilt angle, given as rapid::Degrees or rapid::Radians.
  ///
  /// \returns true if head goal was sent successfully, false otherwise.
  bool StartPanTilt(const Radians& pan, const Radians& tilt);

  /// \brief Returns true if the head is done moving.
  bool IsDone() const;

  /// \brief Stops moving the head.
  void Cancel();

 private:
  static const char kPanTiltAction[];
  static const char kPanJoint[];
  static const char kTiltJoint[];
  static const double kMaxPanVel;   // In radians/s
  static const double kMaxTiltVel;  // In radians/s
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      client_;
  const rapid::JointStateReader& js_reader_;
};

const double Head::kMinPanDegrees = -168;
const double Head::kMaxPanDegrees = 168;
const double Head::kMinTiltDegrees = -30;
const double Head::kMaxTiltDegrees = 60;
const double Head::kMinPanRadians = -168 * M_PI / 180;
const double Head::kMaxPanRadians = 168 * M_PI / 180;
const double Head::kMinTiltRadians = -30 * M_PI / 180;
const double Head::kMaxTiltRadians = 60 * M_PI / 180;

const char Head::kPanTiltAction[] =
    "/head_traj_controller/follow_joint_trajectory";
const char Head::kPanJoint[] = "head_pan_joint";
const char Head::kTiltJoint[] = "head_tilt_joint";
const double Head::kMaxPanVel = 6;
const double Head::kMaxTiltVel = 5;
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_PR2_HEAD_H_
