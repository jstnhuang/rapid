#include "rapid_fetch/fetch.h"

#include "blinky/FaceAction.h"
#include "control_msgs/GripperCommand.h"
#include "rapid_ros/action_client.h"

namespace rapid {
namespace fetch {
Fetch::Fetch(display::DisplayInterface* display,
             sound_play::SoundClient* sound_client)
    : display(display), sound_client_(sound_client) {}

void Fetch::Say(const std::string& text, const std::string& voice) {
  sound_client_->say(text, voice);
}

Fetch* BuildReal() {
  sound_play::SoundClient* sound_client = new sound_play::SoundClient();
  display::DisplayInterface* display = new display::Blinky(
      new rapid_ros::ActionClient<blinky::FaceAction>("blinky"));
  return new Fetch(display, sound_client);
}
}  // namespace fetch
}  // namespace rapid
