#ifndef _RAPID_FETCH_FETCH_H_
#define _RAPID_FETCH_FETCH_H_

#include <string>

#include "rapid_display/display.h"
#include "sound_play/sound_play.h"

namespace rapid {
namespace fetch {
class Fetch {
 public:
  Fetch(display::DisplayInterface* display,
        sound_play::SoundClient* sound_client);

  // Use text-to-speech to say something aloud.
  void Say(const std::string& text,
           const std::string& voice = "kal_voice_diphone");

  // Control a touchscreen display.
  display::DisplayInterface* display;

 private:
  sound_play::SoundClient* sound_client_;
};

Fetch* BuildReal();
}  // namespace fetch
}  // namespace rapid

#endif  // _RAPID_FETCH_FETCH_H_
