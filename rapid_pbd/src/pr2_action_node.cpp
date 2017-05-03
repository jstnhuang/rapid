// Node that runs PR2 action servers using the control_msgs API.

#include "rapid_pbd/action_names.h"
#include "rapid_pbd/pr2_actions.h"
#include "ros/ros.h"

namespace pbd = rapid::pbd;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_actuator_server");

  pbd::pr2::GripperAction left_gripper(pbd::kLeftGripperActionName,
                                       pbd::pr2::kLeftGripperActionName);
  pbd::pr2::GripperAction right_gripper(pbd::kRightGripperActionName,
                                        pbd::pr2::kRightGripperActionName);
  left_gripper.Start();
  right_gripper.Start();
  ros::spin();
  return 0;
}
