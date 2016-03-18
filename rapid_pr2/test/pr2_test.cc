#include "rapid_pr2/pr2.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapid_display/display.h"
#include "rapid_sound/sound.h"

#include "rapid_pr2/gripper.h"
#include "rapid_pr2/head.h"

using rapid::display::MockDisplay;
using rapid::pr2::MockGripper;
using rapid::pr2::MockHead;
using rapid::pr2::Pr2;
using rapid::sound::MockSound;
using rapid::sound::SoundPlay;

TEST(Pr2Test, CallSoundSay) {
  MockDisplay display;
  MockGripper left_gripper;
  MockGripper right_gripper;
  MockHead head;
  MockSound sound;
  Pr2 pr2(display, left_gripper, right_gripper, head, sound);
  EXPECT_CALL(sound, Say("Hello world!"));
  pr2.sound.Say("Hello world!");
}

TEST(Pr2Test, CallDisplayShowDefault) {
  MockDisplay display;
  MockGripper left_gripper;
  MockGripper right_gripper;
  MockHead head;
  MockSound sound;
  Pr2 pr2(display, left_gripper, right_gripper, head, sound);
  EXPECT_CALL(display, ShowDefault());
  pr2.display.ShowDefault();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
