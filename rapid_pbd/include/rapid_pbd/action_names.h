// Constants for the names of our actions and the names of the actions in the
// robot APIs.

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

static const char kArmControllerStateTopic[] = "arm_controller_state";
static const char kFreezeArmService[] = "freeze_arm";
static const char kRelaxArmService[] = "relax_arm";

static const char kHeadActionName[] = "head_action";

static const char kMoveGroupActionName[] = "move_group";

static const char kSurfaceSegmentationActionName[] = "segment_surfaces_action";

namespace pr2 {
static const char kLeftGripperActionName[] =
    "/l_gripper_controller/gripper_action";
static const char kRightGripperActionName[] =
    "/r_gripper_controller/gripper_action";
static const char kListControllersService[] =
    "/pr2_controller_manager/list_controllers";
static const char kSwitchControllerService[] =
    "/pr2_controller_manager/switch_controller";
static const char kHeadActionName[] =
    "/head_traj_controller/follow_joint_trajectory";
}  // namespace pr2

namespace fetch {
static const char kControllerActionName[] = "/query_controller_states";
}  // namespace fetch
}  // namespace pbd
}  // namespace rapid

#endif  // _RAPID_PBD_ACTION_NAMES_H_
