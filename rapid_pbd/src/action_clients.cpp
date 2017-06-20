#include "rapid_pbd/action_clients.h"

#include "rapid_pbd/action_names.h"

namespace rapid {
namespace pbd {
ActionClients::ActionClients()
    : gripper_client(kGripperActionName, true),
      l_gripper_client(kLeftGripperActionName, true),
      r_gripper_client(kRightGripperActionName, true),
      arm_joint_client(kArmJointActionName, true),
      l_arm_joint_client(kLeftArmJointActionName, true),
      r_arm_joint_client(kRightArmJointActionName, true),
      head_client(kHeadActionName, true),
      surface_segmentation_client(kSurfaceSegmentationActionName, true) {}
}  // namespace pbd
}  // namespace rapid
