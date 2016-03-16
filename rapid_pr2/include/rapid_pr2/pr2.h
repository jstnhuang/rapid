#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include <string>

#include "boost/shared_ptr.hpp"
#include "rapid_display/display.h"
#include "rapid_pr2/head.h"
#include "rapid_sound/sound.h"
#include "visualization_msgs/Marker.h"

namespace rapid {
namespace pr2 {
class Pr2 {
 public:
  Pr2(rapid::display::DisplayInterface& display,
      rapid::pr2::HeadInterface& head, rapid::sound::SoundInterface& sound);
  rapid::display::DisplayInterface& display;
  HeadInterface& head;
  rapid::sound::SoundInterface& sound;
};

Pr2 BuildReal();
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
