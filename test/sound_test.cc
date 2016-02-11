#include "rapid/pr2/pr2.h"
#include "rapid/sound/sound.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "ros/ros.h"

using rapid::sound::SoundPlay;
using rapid::sound::MockSoundPlay;
using rapid::pr2::Pr2;

// Declare a test
TEST(SoundTest, TestCall) {
  MockSoundPlay sound;
  Pr2 pr2(sound);
  EXPECT_CALL(sound, Say("Hello world!"));
  pr2.Say("Hello world!");
}

int main(int argc, char **argv) {
  // sound_play is evil and creates a NodeHandle in the default constructor.
  ros::init(argc, argv, "rapid_sound_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
