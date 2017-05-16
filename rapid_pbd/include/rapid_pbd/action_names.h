// Constants for the names of our actions and the names of the actions in the
// PR2 API.

#ifndef _RAPID_PBD_ACTION_NAMES_H_
#define _RAPID_PBD_ACTION_NAMES_H_

namespace rapid {
namespace pbd {
static const char kProgramActionName[] = "execute_program_action";

static const char kGripperActionName[] = "gripper_action";
static const char kLeftGripperActionName[] = "l_gripper_action";
static const char kRightGripperActionName[] = "r_gripper_action";

static const char kArmJointActionName[] = "arm_joint_action";
static const char kLeftArmJointActionName[] = "l_arm_joint_action";
static const char kRightArmJointActionName[] = "r_arm_joint_action";

namespace pr2 {
static const char kLeftGripperActionName[] =
    "/l_gripper_controller/gripper_action";
static const char kRightGripperActionName[] =
    "/r_gripper_controller/gripper_action";
}  // namespace pr2
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_NAMES_H_
