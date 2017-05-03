// Node that runs PR2 action servers using the control_msgs API.

#include "rapid_pbd/pr2.h"

namespace pr2 = rapid::pbd::pr2;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pr2_actuator_server");

  pr2::Gripper left_gripper(pr2::kLeftGripperActionName,
                            pr2::kPr2LeftGripperActionName);
  pr2::Gripper right_gripper(pr2::kRightGripperActionName,
                             pr2::kPr2RightGripperActionName);
  left_gripper.Start();
  right_gripper.Start();
  ros::spin();
  return 0;
}
