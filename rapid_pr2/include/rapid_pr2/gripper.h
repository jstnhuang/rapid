#ifndef _RAPID_PR2_GRIPPER_H_
#define _RAPID_PR2_GRIPPER_H_

#include "actionlib/client/simple_action_client.h"
#include "boost/shared_ptr.hpp"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

namespace rapid {
namespace pr2 {
static const char kLeftGripperAction[] = "l_gripper_controller/gripper_action";
static const char kRightGripperAction[] = "r_gripper_controller/gripper_action";

/// \brief High-level interface for a PR2's gripper.
///
/// \b Example:
/// \code
///   rapid::pr2::Gripper left = rapid::pr2::Gripper::Left();
///   left.StartClosing();
///   while (ros::ok() && !left.IsDone()) {
///     ros::spinOnce();
///   }
/// \endcode
class Gripper {
 public:
  /// Returns a Gripper object for the PR2's left gripper.
  static Gripper Left();

  /// Returns a Gripper object for the PR2's right gripper.
  static Gripper Right();

  /// \brief Starts opening the gripper.
  /// \returns true if the goal was sent, false if this couldn't connect to the
  ///   gripper action server within 1 second.
  bool StartOpening();

  /// \brief Starts closing the gripper.
  /// \returns true if the goal was sent, false if this couldn't connect to the
  ///   gripper action server within 1 second.
  bool StartClosing();

  /// \brief Starts closing the gripper with a given maximum effort.
  ///
  /// \param[in] max_effort The maximum amount of force to use, in Newtons.
  /// \returns true if the goal was sent, false if this couldn't connect to the
  ///   gripper action server within 1 second.
  bool StartClosing(double max_effort);

  /// \brief Returns true if the gripper is done opening or closing.
  bool IsDone() const;

  /// \brief Stops the gripper's opening or closing movement.
  void Cancel();

 private:
  explicit Gripper(bool is_left);

  boost::shared_ptr<actionlib::SimpleActionClient<
      pr2_controllers_msgs::Pr2GripperCommandAction> >
      client_;
};
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_PR2_GRIPPER_H_
