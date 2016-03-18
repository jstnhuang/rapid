// A class for opening and closing the grippers.
//
// Sample usage:
//  rapid_pr2::Gripper right_gripper(
//    Gripper::RIGHT_GRIPPER);
//  right_gripper.Open();
//  right_gripper.Close();
//
// Notes:
// 1. The gripper does not work quite right in Gazebo
// 2. Return value is based on whether the gripper reached the target position.
//    So, if the gripper is closing around an object, Close() will typically
//    return false because the object stalls the gripper before it reaches
//    the goal position.

#include "actionlib/client/simple_action_client.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#ifndef _RAPID_PR2_GRIPPER_H_
#define _RAPID_PR2_GRIPPER_H_

namespace rapid {
namespace pr2 {
typedef actionlib::SimpleActionClient<
    pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper {
 private:
  GripperClient* gripper_client_;
  tf::TransformListener transform_listener_;
  const int gripper_id_;

  static const double OPEN_THRESHOLD;  // Gripper open threshold

 public:
  static const double OPEN;    // Canonical "open" position.
  static const double CLOSED;  // Canonical "closed" position.

  // Gripper ids
  static const int LEFT_GRIPPER = 0;
  static const int RIGHT_GRIPPER = 1;

  // Gripper action topics
  static const std::string LEFT_GRIPPER_ACTION;
  static const std::string RIGHT_GRIPPER_ACTION;

  // Constructor that takes the gripper id.
  // gripper_id: Gripper::LEFT_GRIPPER or Gripper::RIGHT_GRIPPER
  Gripper(const int gripper_id);

  ~Gripper();

  // Gets the gripper to the given position. .
  // position - how wide to open or close the gripper
  // effort - now much force to exert, negative is full force
  bool SetPosition(double position, double effort = -1.0);

  // Gets the gripper's current position. Note: may not agree with "SetPosition"
  double GetPosition();

  // Returns whether the gripper is open or not.
  bool IsOpen();

  // Opens the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // effort - defaults to -1.0, to open with unlimited effort
  bool Open(double effort = -1.0);

  // Closes the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // effort - defaults to 50.0 to close gently
  bool Close(double effort = -1.0);
};
}  // namespace pr2
}  // namespace rapid

#endif  // _RAPID_PR2_GRIPPER_H_
