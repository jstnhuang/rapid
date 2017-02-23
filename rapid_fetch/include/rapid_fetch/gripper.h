#ifndef _RAPID_FETCH_GRIPPER_H_
#define _RAPID_FETCH_GRIPPER_H_

#include "actionlib/client/simple_action_client.h"
#include "control_msgs/GripperCommandAction.h"

namespace rapid {
namespace fetch {
// Gripper is a wrapper around the action to open/close the Fetch's gripper.
class Gripper {
 public:
  Gripper();

  bool Close(double max_effort = 0);
  bool Open(double max_effort = 0);

  static const char kGripperAction[];
  static const double kClosedPosition;
  static const double kOpenedPosition;

 private:
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client_;
};
}  // namespace fetch
}  // namespace rapid

#endif  // _RAPID_FETCH_GRIPPER_H_
