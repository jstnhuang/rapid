#ifndef _RAPID_PR2_TORSO_H_
#define _RAPID_PR2_TORSO_H_

#include "actionlib/client/simple_action_client.h"
#include "boost/shared_ptr.hpp"
#include "control_msgs/FollowJointTrajectoryAction.h"

namespace rapid {
namespace pr2 {

/// \brief High-level interface for the PR2's torso.
///
/// \b Example:
/// \code
///   rapid::pr2::Torso torso;
///   torso.StartMoving(0.31);
///   while (ros::ok() && !torso.IsDone()) {
///     ros::spinOnce();
///   }
/// \endcode
class Torso {
 public:
  static const double kMinHeight;  /// The minimum torso height.
  static const double kMaxHeight;  /// The maximum torso height.

  /// Constructor.
  Torso();

  /// \brief Starts moving the torso.
  ///
  /// The torso will move at its maximum velocity (1.3 cm/s) towards the goal.
  ///
  /// \param[in] height The height move to. The value will be clamped between
  ///   Torso::kMinHeight and Torso::kMaxHeight.
  ///
  /// \returns true if the connection to the torso action server was successful,
  ///   false otherwise.
  bool StartMoving(double height);

  /// \brief Returns true if the torso is done moving.
  bool IsDone() const;

  /// \brief Stops moving the torso.
  void Cancel();

 private:
  static const double kMaxVel;
  static const char kTorsoJoint[];
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
      client_;
};

static const char kTorsoAction[] = "/torso_controller/follow_joint_trajectory";
const double Torso::kMinHeight = 0;     /// 0 meters
const double Torso::kMaxHeight = 0.31;  /// 0.31 meters
const double Torso::kMaxVel = 0.013;
const char Torso::kTorsoJoint[] = "torso_lift_joint";
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_PR2_TORSO_H_
