#ifndef _RAPID_SOUND_SOUND_H_
#define _RAPID_SOUND_SOUND_H_

#include "gmock/gmock.h"
#include "sound_play/sound_play.h"
#include <string>

namespace rapid {
namespace sound {
class SoundInterface {
 public:
  virtual ~SoundInterface(){};
  virtual void Say(const std::string& text) = 0;
};

// Implements SoundInterface using sound_play.
class SoundPlay : public SoundInterface {
 public:
  explicit SoundPlay(const std::string& voice = "voice_kal_diphone");
  virtual void Say(const std::string& text);

 private:
  sound_play::SoundClient client_;
  std::string voice_;
};

// Mock version for testing.
class MockSoundPlay : public SoundPlay {
 public:
  explicit MockSoundPlay(const std::string& voice = "voice_kal_diphone") {}
  MOCK_METHOD1(Say, void(const std::string& text));
};
};
};
#endif  // _RAPID_SOUND_SOUND_H_
