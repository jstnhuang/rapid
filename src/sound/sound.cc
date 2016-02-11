#include "rapid/sound/sound.h"

#include "sound_play/sound_play.h"

#include <iostream>
#include <string>

namespace rapid {
namespace sound {
SoundPlay::SoundPlay(const std::string& voice) : client_(), voice_(voice) {}

void SoundPlay::Say(const std::string& text) { client_.say(text, voice_); }
}  // namespace sound
}  // namespace rapid
