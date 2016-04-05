#include "rapid_pr2/pr2.h"

#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_manipulation/tuck_arms.h"
#include "rapid_sound/sound.h"

using boost::shared_ptr;
using rapid::manipulation::Gripper;

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::manipulation::ArmInterface& left_arm,
         rapid::manipulation::ArmInterface& right_arm,
         rapid::display::DisplayInterface& display,
         rapid::manipulation::GripperInterface& left_gripper,
         rapid::manipulation::GripperInterface& right_gripper,
         rapid::manipulation::HeadInterface& head,
         rapid::sound::SoundInterface& sound,
         rapid::manipulation::TuckArmsInterface& tuck_arms)
    : left_arm(left_arm),
      right_arm(right_arm),
      display(display),
      left_gripper(left_gripper),
      right_gripper(right_gripper),
      head(head),
      sound(sound),
      tuck_arms(tuck_arms) {}

shared_ptr<Pr2> BuildReal() {
  rapid::manipulation::ArmInterface* left_arm =
      new rapid::manipulation::MoveItArm(rapid::manipulation::LEFT);
  rapid::manipulation::ArmInterface* right_arm =
      new rapid::manipulation::MoveItArm(rapid::manipulation::RIGHT);
  rapid::display::DisplayInterface* display = new rapid::display::Blinky();
  rapid::manipulation::GripperInterface* left_gripper =
      new Gripper(Gripper::LEFT_GRIPPER);
  rapid::manipulation::GripperInterface* right_gripper =
      new Gripper(Gripper::RIGHT_GRIPPER);
  rapid::manipulation::HeadInterface* head = new rapid::manipulation::Head();
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  rapid::manipulation::TuckArmsInterface* tuck_arms =
      new rapid::manipulation::Pr2TuckArms();
  shared_ptr<Pr2> pr2(new Pr2(*left_arm, *right_arm, *display, *left_gripper,
                              *right_gripper, *head, *sound, *tuck_arms));
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
