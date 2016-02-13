#include "rapid/pr2/pr2.h"

#include <string>

#include "actionlib/client/simple_action_client.h"
#include "blinky/FaceAction.h"
#include "rapid/display/display.h"
#include "rapid/sound/sound.h"

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::display::DisplayInterface& display,
         rapid::sound::SoundInterface& sound)
    : display(display), sound(sound) {}

Pr2 BuildReal() {
  actionlib::SimpleActionClient<blinky::FaceAction> face_client("blinky", true);
  rapid::display::DisplayInterface* display =
      new rapid::display::Blinky(face_client);
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  Pr2 pr2(*display, *sound);
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
