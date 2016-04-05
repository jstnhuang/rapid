#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_manipulation/tuck_arms.h"
#include "rapid_sound/sound.h"

namespace rapid {
namespace pr2 {
class Pr2 {
 public:
  Pr2(rapid::manipulation::ArmInterface& left_arm,
      rapid::manipulation::ArmInterface& right_arm,
      rapid::display::DisplayInterface& display,
      rapid::manipulation::GripperInterface& left_gripper,
      rapid::manipulation::GripperInterface& right_gripper,
      rapid::manipulation::HeadInterface& head,
      rapid::sound::SoundInterface& sound,
      rapid::manipulation::TuckArmsInterface& tuck_arms);
  rapid::manipulation::ArmInterface& left_arm;
  rapid::manipulation::ArmInterface& right_arm;
  rapid::display::DisplayInterface& display;
  rapid::manipulation::GripperInterface& left_gripper;
  rapid::manipulation::GripperInterface& right_gripper;
  rapid::manipulation::HeadInterface& head;
  rapid::sound::SoundInterface& sound;
  rapid::manipulation::TuckArmsInterface& tuck_arms;
};

boost::shared_ptr<Pr2> BuildReal();
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
