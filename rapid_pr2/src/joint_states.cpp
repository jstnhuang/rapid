#include "rapid_pr2/joint_states.h"

namespace rapid {
namespace pr2 {
JointStates::JointStates() : joint_positions_() { Initialize(); }

void JointStates::Set(const std::map<std::string, double>& joint_positions) {
  for (std::map<std::string, double>::const_iterator i =
           joint_positions.begin();
       i != joint_positions.end(); ++i) {
    joint_positions_[i->first] = i->second;
  }
}

std::map<std::string, double> JointStates::joint_positions() {
  return joint_positions_;
}

void JointStates::Initialize() {
  joint_positions_["fl_caster_rotation_joint"] = -8.02459178217e-07;
  joint_positions_["fl_caster_l_wheel_joint"] = 0.533794335608;
  joint_positions_["fl_caster_r_wheel_joint"] = 0.533588815886;
  joint_positions_["fr_caster_rotation_joint"] = -8.23756285406e-07;
  joint_positions_["fr_caster_l_wheel_joint"] = 0.534701287511;
  joint_positions_["fr_caster_r_wheel_joint"] = 0.535714054147;
  joint_positions_["bl_caster_rotation_joint"] = 2.28678578296e-06;
  joint_positions_["bl_caster_l_wheel_joint"] = 0.518123172485;
  joint_positions_["bl_caster_r_wheel_joint"] = 0.517648670803;
  joint_positions_["br_caster_rotation_joint"] = -2.28634212007e-06;
  joint_positions_["br_caster_l_wheel_joint"] = 0.512599892408;
  joint_positions_["br_caster_r_wheel_joint"] = 0.513140302713;
  joint_positions_["torso_lift_joint"] = 0.0116032683619;
  joint_positions_["torso_lift_motor_screw_joint"] = 0.0;
  joint_positions_["head_pan_joint"] = 0.000171832254891;
  joint_positions_["head_tilt_joint"] = -0.019527841305;
  joint_positions_["laser_tilt_mount_joint"] = -0.134801738083;
  joint_positions_["r_upper_arm_roll_joint"] = 0.00187607033515;
  joint_positions_["r_shoulder_pan_joint"] = -7.65765703736e-06;
  joint_positions_["r_shoulder_lift_joint"] = 0.00694242650034;
  joint_positions_["r_forearm_roll_joint"] = -2.82108562555e-06;
  joint_positions_["r_elbow_flex_joint"] = -0.307102208425;
  joint_positions_["r_wrist_flex_joint"] = -0.436910912535;
  joint_positions_["r_wrist_roll_joint"] = -0.000240722153036;
  joint_positions_["r_gripper_joint"] = 0.00181575487655;
  joint_positions_["r_gripper_l_finger_joint"] = 0.0120426098916;
  joint_positions_["r_gripper_r_finger_joint"] = 0.0120426098916;
  joint_positions_["r_gripper_r_finger_tip_joint"] = 0.0120426098916;
  joint_positions_["r_gripper_l_finger_tip_joint"] = 0.0120426098916;
  joint_positions_["r_gripper_motor_screw_joint"] = 0.0;
  joint_positions_["r_gripper_motor_slider_joint"] = 0.0;
  joint_positions_["l_upper_arm_roll_joint"] = -0.00420560966146;
  joint_positions_["l_shoulder_pan_joint"] = -0.000130202341205;
  joint_positions_["l_shoulder_lift_joint"] = 0.00691509715501;
  joint_positions_["l_forearm_roll_joint"] = 0.000368846946658;
  joint_positions_["l_elbow_flex_joint"] = -0.306240712687;
  joint_positions_["l_wrist_flex_joint"] = -0.438061830016;
  joint_positions_["l_wrist_roll_joint"] = 0.000332332153258;
  joint_positions_["l_gripper_joint"] = 0.00116500808565;
  joint_positions_["l_gripper_l_finger_joint"] = 0.00847708143776;
  joint_positions_["l_gripper_r_finger_joint"] = 0.00847708143776;
  joint_positions_["l_gripper_r_finger_tip_joint"] = 0.00847708143776;
  joint_positions_["l_gripper_l_finger_tip_joint"] = 0.00847708143776;
  joint_positions_["l_gripper_motor_screw_joint"] = 0.0;
  joint_positions_["l_gripper_motor_slider_joint"] = 0.0;
}
}  // namespace pr2
}  // namespace rapid
