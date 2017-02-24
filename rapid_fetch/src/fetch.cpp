#include "rapid_fetch/fetch.h"

#include "rapid_display/display.h"
#include "sound_play/sound_play.h"

#include "rapid_fetch/gripper.h"

namespace rapid {
namespace fetch {
Fetch::Fetch(display::DisplayInterface* display, Gripper* gripper,
             sound_play::SoundClient* sound_client)
    : display(display), gripper(gripper), sound_client_(sound_client) {}

void Fetch::Say(const std::string& text, const std::string& voice) {
  sound_client_->say(text, voice);
}

Fetch* BuildReal() {
  display::DisplayInterface* display = new display::Blinky(
      new rapid_ros::ActionClient<blinky::FaceAction>("blinky"));
  Gripper* gripper = new Gripper();
  sound_play::SoundClient* sound_client = new sound_play::SoundClient();
  return new Fetch(display, gripper, sound_client);
}
}  // namespace fetch
}  // namespace rapid
