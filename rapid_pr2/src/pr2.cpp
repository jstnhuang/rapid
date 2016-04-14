#include "rapid_pr2/pr2.h"

#include "blinky/FaceAction.h"
#include "pr2_controllers_msgs/PointHeadAction.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"

#include "rapid_display/display.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/head.h"
#include "rapid_manipulation/tuck_arms.h"
#include "rapid_ros/action_client.h"
#include "rapid_sound/sound.h"

using boost::shared_ptr;
using blinky::FaceAction;
using pr2_controllers_msgs::PointHeadAction;
using pr2_controllers_msgs::Pr2GripperCommandAction;
using rapid::display::DisplayInterface;
using rapid::display::Blinky;
using rapid::manipulation::HeadInterface;
using rapid::manipulation::Head;
using rapid::manipulation::MoveItArm;
using rapid::sound::SoundInterface;
using rapid::sound::SoundPlay;
using rapid::manipulation::ArmInterface;
using rapid::manipulation::Gripper;
using rapid::manipulation::GripperInterface;
using rapid_ros::ActionClient;
using rapid::manipulation::TuckArmsInterface;
using rapid::manipulation::Pr2TuckArms;

namespace rapid {
namespace pr2 {
Pr2::Pr2(ArmInterface& left_arm, ArmInterface& right_arm,
         DisplayInterface& display, GripperInterface& left_gripper,
         GripperInterface& right_gripper, HeadInterface& head,
         SoundInterface& sound, TuckArmsInterface& tuck_arms)
    : left_arm(left_arm),
      right_arm(right_arm),
      display(display),
      left_gripper(left_gripper),
      right_gripper(right_gripper),
      head(head),
      sound(sound),
      tuck_arms(tuck_arms),
      left_picker(left_arm, left_gripper),
      right_picker(right_arm, right_gripper),
      left_placer(left_arm, left_gripper),
      right_placer(right_arm, right_gripper),
      left_object_(),
      right_object_() {}

shared_ptr<Pr2> BuildReal() {
  ArmInterface* left_arm = new MoveItArm(rapid::manipulation::LEFT);
  ArmInterface* right_arm = new MoveItArm(rapid::manipulation::RIGHT);
  DisplayInterface* display =
      new Blinky(new ActionClient<FaceAction>("blinky"));
  GripperInterface* left_gripper = new Gripper(
      Gripper::LEFT_GRIPPER,
      new ActionClient<Pr2GripperCommandAction>(Gripper::LEFT_GRIPPER_ACTION));
  GripperInterface* right_gripper = new Gripper(
      Gripper::RIGHT_GRIPPER,
      new ActionClient<Pr2GripperCommandAction>(Gripper::RIGHT_GRIPPER_ACTION));
  HeadInterface* head = new Head(new ActionClient<PointHeadAction>(
      "/head_traj_controller/point_head_action"));
  SoundInterface* sound = new SoundPlay();
  TuckArmsInterface* tuck_arms = new Pr2TuckArms();
  shared_ptr<Pr2> pr2(new Pr2(*left_arm, *right_arm, *display, *left_gripper,
                              *right_gripper, *head, *sound, *tuck_arms));
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
