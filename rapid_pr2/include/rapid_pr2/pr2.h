#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_manipulation/tuck_arms.h"
#include "rapid_ros/service_client.h"
#include "rapid_sound/sound.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace pr2 {
// Class for controlling a PR2.
//
// To use a real PR2:
//  Pr2* pr2 = BuildReal(node_handle);
//  pr2->left_arm()->MoveToPoseGoal(...);
//  delete pr2;
class Pr2 {
 public:
  // The PR2 claims ownership of pointers passed in.
  Pr2(rapid::manipulation::ArmInterface* left_arm,
      rapid::manipulation::ArmInterface* right_arm,
      rapid::manipulation::GripperInterface* left_gripper,
      rapid::manipulation::GripperInterface* right_gripper,
      rapid::manipulation::HeadInterface* head,
      rapid::sound::SoundInterface* sound,
      rapid::manipulation::TuckArmsInterface* tuck_arms);
  ~Pr2();

  // Accessors
  rapid::manipulation::ArmInterface* left_arm();
  rapid::manipulation::ArmInterface* right_arm();
  rapid::manipulation::GripperInterface* left_gripper();
  rapid::manipulation::GripperInterface* right_gripper();
  rapid::manipulation::HeadInterface* head();
  rapid::sound::SoundInterface* sound();
  rapid::manipulation::TuckArmsInterface* tuck_arms();

 private:
  rapid::manipulation::ArmInterface* const left_arm_;
  rapid::manipulation::ArmInterface* const right_arm_;
  rapid::manipulation::GripperInterface* const left_gripper_;
  rapid::manipulation::GripperInterface* const right_gripper_;
  rapid::manipulation::HeadInterface* const head_;
  rapid::sound::SoundInterface* const sound_;
  rapid::manipulation::TuckArmsInterface* const tuck_arms_;
};

// Factory function for building a PR2 object for a real PR2.
// Ownership of the Pr2 pointer is transferred to the caller.
Pr2* BuildReal(ros::NodeHandle& nh);
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
