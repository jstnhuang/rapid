#include "rapid_manipulation/gripper.h"

#include "actionlib/server/simple_action_server.h"
#include "gtest/gtest.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

#include "rapid_perception/object.h"
#include "rapid_ros/action_client.h"
#include "rapid_ros/tf_listener.h"

using actionlib::SimpleClientGoalState;
using rapid_ros::MockActionClient;
using rapid_ros::MockTfListener;
using pr2_controllers_msgs::Pr2GripperCommandAction;
using pr2_controllers_msgs::Pr2GripperCommandGoal;
using pr2_controllers_msgs::Pr2GripperCommandResult;

namespace rapid {
namespace manipulation {
class GripperTest : public ::testing::Test {
 public:
  GripperTest()
      : client_(new MockActionClient<Pr2GripperCommandAction>()),
        tf_(new MockTfListener()),
        gripper_(Gripper::RIGHT_GRIPPER, client_, tf_) {}

  void SetUp() {}

  void SetTfDistance(double distance) {
    tf::StampedTransform stamped_transform;
    tf::Transform transform;
    tf::Vector3 origin(0, distance, 0);
    transform.setOrigin(origin);
    stamped_transform.setData(transform);
    tf_->SetTransform("r_gripper_l_finger_tip_link",
                      "r_gripper_r_finger_tip_link", stamped_transform);
  }

 protected:
  MockActionClient<Pr2GripperCommandAction>* client_;
  MockTfListener* tf_;
  Gripper gripper_;
};

TEST_F(GripperTest, Open) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  SetTfDistance(0.118);
  bool success = gripper_.Open();
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0.09, goal.command.position);  // Gripper::OPEN
  EXPECT_EQ(-1, goal.command.max_effort);
}

TEST_F(GripperTest, IsClosed) {
  SetTfDistance(0.031);  // Based on tf_echo of closed gripper.
  EXPECT_EQ(false, gripper_.IsOpen());
}

TEST_F(GripperTest, IsOpen) {
  SetTfDistance(0.118);  // Based on tf_echo of open gripper.
  EXPECT_EQ(true, gripper_.IsOpen());
}

TEST_F(GripperTest, IsHalfOpen) {
  SetTfDistance(0.04);
  EXPECT_EQ(true, gripper_.IsOpen());
}

TEST_F(GripperTest, GetPositionException) {
  tf_->SetThrowException("r_gripper_l_finger_tip_link",
                         "r_gripper_r_finger_tip_link", true);
  EXPECT_EQ(-1, gripper_.GetPosition());
}

TEST_F(GripperTest, Close) {
  SetTfDistance(0.031);
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.Close(10);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0, goal.command.position);  // Gripper::CLOSED
  EXPECT_EQ(10, goal.command.max_effort);
}

TEST_F(GripperTest, ShouldDropObjectOnOpen) {
  EXPECT_EQ(false, gripper_.is_holding_object());

  rapid::perception::Object obj;
  obj.set_name("test");
  gripper_.set_held_object(obj);
  EXPECT_EQ(true, gripper_.is_holding_object());

  // SetPosition measures the start position before sending the goal to open.
  // This will make it measure the starting position as closed.
  SetTfDistance(0.031);

  rapid::perception::Object actual_obj;
  EXPECT_EQ(true, gripper_.HeldObject(&actual_obj));
  EXPECT_EQ("test", actual_obj.name());

  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  gripper_.SetPosition(0.03);  // Slightly open.
  EXPECT_EQ(false, gripper_.is_holding_object());
}

TEST_F(GripperTest, ShouldClampIfPositionTooSmall) {
  SetTfDistance(0.031);
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.SetPosition(-1);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(0, goal.command.position);  // Gripper::CLOSED
  EXPECT_EQ(-1, goal.command.max_effort);
  EXPECT_EQ(true, success);
}

TEST_F(GripperTest, ShouldClampIfPositionTooLarge) {
  SetTfDistance(0.118);
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.SetPosition(1);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(0.09, goal.command.position);  // Gripper::OPEN
  EXPECT_EQ(-1, goal.command.max_effort);
  EXPECT_EQ(true, success);
}

TEST_F(GripperTest, ShouldFailIfNoServer) {
  SetTfDistance(0.118);
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  client_->set_server_delay(ros::DURATION_MAX);
  bool success = gripper_.Open();
  EXPECT_EQ(false, success);
}

TEST_F(GripperTest, ShouldFailIfResultTooSlow) {
  SetTfDistance(0.118);
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  client_->set_result_delay(ros::DURATION_MAX);
  bool success = gripper_.Open();
  EXPECT_EQ(false, success);
}

// If the gripper stalls while trying to open or close all the way, actionlib
// will report the state as ABORTED. Because it still grasped / let go of the
// object, we consider it a success.
TEST_F(GripperTest, ShouldPassIfStateAborted) {
  SetTfDistance(0.031);
  client_->set_state(SimpleClientGoalState::ABORTED);
  bool success = gripper_.Close();
  EXPECT_EQ(true, success);
}

TEST_F(GripperTest, ShouldFailIfOtherState) {
  SetTfDistance(0.031);
  client_->set_state(SimpleClientGoalState::LOST);
  bool success = gripper_.Close();
  EXPECT_EQ(false, success);
}
}  // namespace manipulation
}  //  namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "gripper_test");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
