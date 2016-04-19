#include "rapid_pr2/pr2.h"

#include "blinky/FaceAction.h"
#include "pr2_controllers_msgs/PointHeadAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_manipulation/tuck_arms.h"
#include "rapid_ros/action_client.h"
#include "rapid_sound/sound.h"

using blinky::FaceAction;
using pr2_controllers_msgs::PointHeadAction;
using pr2_controllers_msgs::Pr2GripperCommandAction;
using rapid::display::Blinky;
using rapid::display::DisplayInterface;
using rapid::display::MockDisplay;
using rapid::manipulation::ArmInterface;
using rapid::manipulation::Gripper;
using rapid::manipulation::GripperInterface;
using rapid::manipulation::Head;
using rapid::manipulation::HeadInterface;
using rapid::manipulation::MockArm;
using rapid::manipulation::MockGripper;
using rapid::manipulation::MockHead;
using rapid::manipulation::MockTuckArms;
using rapid::manipulation::MoveItArm;
using rapid::manipulation::Picker;
using rapid::manipulation::Placer;
using rapid::manipulation::Pr2TuckArms;
using rapid::manipulation::TuckArmsInterface;
using rapid::sound::MockSound;
using rapid::sound::SoundInterface;
using rapid::sound::SoundPlay;
using rapid_ros::ActionClient;

namespace rapid {
namespace pr2 {
Pr2::Pr2(ArmInterface* left_arm, ArmInterface* right_arm,
         DisplayInterface* display, GripperInterface* left_gripper,
         GripperInterface* right_gripper, HeadInterface* head,
         SoundInterface* sound, TuckArmsInterface* tuck_arms)
    : left_arm_(left_arm),
      right_arm_(right_arm),
      display_(display),
      left_gripper_(left_gripper),
      right_gripper_(right_gripper),
      head_(head),
      sound_(sound),
      tuck_arms_(tuck_arms),
      left_picker_(new Picker(left_arm, left_gripper)),
      right_picker_(new Picker(right_arm, right_gripper)),
      left_placer_(new Placer(left_arm, left_gripper)),
      right_placer_(new Placer(right_arm, right_gripper)) {}

Pr2::~Pr2() {
  if (left_arm_) {
    delete left_arm_;
  }
  if (right_arm_) {
    delete right_arm_;
  }
  if (display_) {
    delete display_;
  }
  if (left_gripper_) {
    delete left_gripper_;
  }
  if (right_gripper_) {
    delete right_gripper_;
  }
  if (head_) {
    delete head_;
  }
  if (sound_) {
    delete sound_;
  }
  if (tuck_arms_) {
    delete tuck_arms_;
  }
}

ArmInterface* Pr2::left_arm() { return left_arm_; }
ArmInterface* Pr2::right_arm() { return right_arm_; }
DisplayInterface* Pr2::display() { return display_; }
GripperInterface* Pr2::left_gripper() { return left_gripper_; }
GripperInterface* Pr2::right_gripper() { return right_gripper_; }
HeadInterface* Pr2::head() { return head_; }
SoundInterface* Pr2::sound() { return sound_; }
TuckArmsInterface* Pr2::tuck_arms() { return tuck_arms_; }
Picker* Pr2::left_picker() { return left_picker_; }
Picker* Pr2::right_picker() { return right_picker_; }
Placer* Pr2::left_placer() { return left_placer_; }
Placer* Pr2::right_placer() { return right_placer_; }

Pr2* BuildReal() {
  ArmInterface* left_arm = new MoveItArm(rapid::manipulation::LEFT);
  ArmInterface* right_arm = new MoveItArm(rapid::manipulation::RIGHT);
  DisplayInterface* display =
      new Blinky(new ActionClient<FaceAction>("blinky"));
  GripperInterface* left_gripper = new Gripper(
      Gripper::LEFT_GRIPPER,
      new ActionClient<Pr2GripperCommandAction>(Gripper::LEFT_GRIPPER_ACTION));
  GripperInterface* right_gripper = new Gripper(
      Gripper::RIGHT_GRIPPER,
      new ActionClient<Pr2GripperCommandAction>(Gripper::RIGHT_GRIPPER_ACTION));
  HeadInterface* head = new Head(new ActionClient<PointHeadAction>(
      "/head_traj_controller/point_head_action"));
  SoundInterface* sound = new SoundPlay();
  TuckArmsInterface* tuck_arms = new Pr2TuckArms();
  return new Pr2(left_arm, right_arm, display, left_gripper, right_gripper,
                 head, sound, tuck_arms);
}
}  // namespace pr2
}  // namespace rapid