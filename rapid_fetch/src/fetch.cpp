#include "rapid_fetch/fetch.h"

#include "sound_play/sound_play.h"

#include "rapid_fetch/gripper.h"

namespace rapid {
namespace fetch {
Fetch::Fetch(Gripper* gripper, sound_play::SoundClient* sound_client)
    : gripper(gripper), sound_client_(sound_client) {}

void Fetch::Say(const std::string& text, const std::string& voice) {
  sound_client_->say(text, voice);
}

Fetch* BuildReal() {
  Gripper* gripper = new Gripper();
  sound_play::SoundClient* sound_client = new sound_play::SoundClient();
  return new Fetch(gripper, sound_client);
}
}  // namespace fetch
}  // namespace rapid
