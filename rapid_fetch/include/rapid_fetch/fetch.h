#ifndef _RAPID_FETCH_FETCH_H_
#define _RAPID_FETCH_FETCH_H_

#include <string>

#include "sound_play/sound_play.h"

#include "rapid_fetch/gripper.h"

namespace rapid {
namespace fetch {
class Fetch {
 public:
  Fetch(Gripper* gripper, sound_play::SoundClient* sound_client);

  // Use text-to-speech to say something aloud.
  void Say(const std::string& text,
           const std::string& voice = "voice_kal_diphone");

  Gripper* gripper;

 private:
  sound_play::SoundClient* sound_client_;
};

Fetch* BuildReal();
}  // namespace fetch
}  // namespace rapid

#endif  // _RAPID_FETCH_FETCH_H_
