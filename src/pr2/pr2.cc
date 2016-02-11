#include "rapid/pr2/pr2.h"

#include <string>

#include "rapid/sound/sound.h"

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::sound::SoundInterface& sound) : sound_(sound) {}

void Pr2::Say(const std::string& text) { sound_.Say(text); }

Pr2 BuildReal() {
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  Pr2 pr2(*sound);
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
