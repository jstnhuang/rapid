#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_sound/sound.h"
#include "visualization_msgs/Marker.h"

#include "rapid_pr2/gripper.h"
#include "rapid_pr2/head.h"

namespace rapid {
namespace pr2 {
class Pr2 {
 public:
  Pr2(rapid::manipulation::ArmInterface& left_arm,
      rapid::manipulation::ArmInterface& right_arm,
      rapid::display::DisplayInterface& display, GripperInterface& left_gripper,
      GripperInterface& right_gripper, HeadInterface& head,
      rapid::sound::SoundInterface& sound);
  rapid::manipulation::ArmInterface& left_arm;
  rapid::manipulation::ArmInterface& right_arm;
  rapid::display::DisplayInterface& display;
  GripperInterface& left_gripper;
  GripperInterface& right_gripper;
  HeadInterface& head;
  rapid::sound::SoundInterface& sound;
};

boost::shared_ptr<Pr2> BuildReal();
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
