#ifndef _RAPID_PR2_PR2_H_
#define _RAPID_PR2_PR2_H_

#include <string>

#include "rapid/sound/sound.h"

namespace rapid {
namespace pr2 {
class Pr2 {
 public:
  explicit Pr2(rapid::sound::SoundInterface& sound);
  void Say(const std::string& text);

 private:
  rapid::sound::SoundInterface& sound_;
};

Pr2 BuildReal();
}  // namespace pr2
}  // namespace rapid
#endif  // _RAPID_PR2_PR2_H_
