#include "rapid_pr2/pr2.h"

#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_sound/sound.h"

#include "rapid_pr2/gripper.h"
#include "rapid_pr2/head.h"

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::manipulation::ArmInterface& left_arm,
         rapid::manipulation::ArmInterface& right_arm,
         rapid::display::DisplayInterface& display,
         GripperInterface& left_gripper, GripperInterface& right_gripper,
         rapid::pr2::HeadInterface& head, rapid::sound::SoundInterface& sound)
    : left_arm(left_arm),
      right_arm(right_arm),
      display(display),
      left_gripper(left_gripper),
      right_gripper(right_gripper),
      head(head),
      sound(sound) {}

Pr2 BuildReal() {
  rapid::manipulation::ArmInterface* left_arm =
      new rapid::manipulation::MoveItArm(rapid::manipulation::LEFT);
  rapid::manipulation::ArmInterface* right_arm =
      new rapid::manipulation::MoveItArm(rapid::manipulation::RIGHT);
  rapid::display::DisplayInterface* display = new rapid::display::Blinky();
  GripperInterface* left_gripper = new Gripper(Gripper::LEFT_GRIPPER);
  GripperInterface* right_gripper = new Gripper(Gripper::RIGHT_GRIPPER);
  HeadInterface* head = new Head();
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  Pr2 pr2(*left_arm, *right_arm, *display, *left_gripper, *right_gripper, *head,
          *sound);
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
