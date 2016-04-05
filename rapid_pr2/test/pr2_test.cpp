#include "rapid_pr2/pr2.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_sound/sound.h"

using rapid::display::MockDisplay;
using rapid::manipulation::MockArm;
using rapid::manipulation::MockGripper;
using rapid::manipulation::MockHead;
using rapid::pr2::Pr2;
using rapid::sound::MockSound;
using rapid::sound::SoundPlay;

// TEST(Pr2Test, CallSoundSay) {
//  MockArm left_arm;
//  MockArm right_arm;
//  MockDisplay display;
//  MockGripper left_gripper;
//  MockGripper right_gripper;
//  MockHead head;
//  MockSound sound;
//  Pr2 pr2(left_arm, right_arm, display, left_gripper, right_gripper, head,
//          sound);
//  EXPECT_CALL(sound, Say("Hello world!"));
//  pr2.sound.Say("Hello world!");
//}
//
// TEST(Pr2Test, CallDisplayShowDefault) {
//  MockArm left_arm;
//  MockArm right_arm;
//  MockDisplay display;
//  MockGripper left_gripper;
//  MockGripper right_gripper;
//  MockHead head;
//  MockSound sound;
//  Pr2 pr2(left_arm, right_arm, display, left_gripper, right_gripper, head,
//          sound);
//  EXPECT_CALL(display, ShowDefault());
//  pr2.display.ShowDefault();
//}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
