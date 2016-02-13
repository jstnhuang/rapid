#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include <string>

#include "rapid/display/display.h"
#include "rapid/sound/sound.h"

namespace rapid {
namespace pr2 {
class Pr2 {
 public:
  Pr2(rapid::display::DisplayInterface& display,
      rapid::sound::SoundInterface& sound);
  rapid::display::DisplayInterface& display;
  rapid::sound::SoundInterface& sound;
};

Pr2 BuildReal();
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
