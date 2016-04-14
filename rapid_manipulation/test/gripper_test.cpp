#include "rapid_manipulation/gripper.h"

#include "actionlib/server/simple_action_server.h"
#include "gtest/gtest.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"

#include "rapid_ros/action_client.h"

using actionlib::SimpleClientGoalState;
using rapid_ros::MockActionClient;
using pr2_controllers_msgs::Pr2GripperCommandAction;
using pr2_controllers_msgs::Pr2GripperCommandGoal;
using pr2_controllers_msgs::Pr2GripperCommandResult;

namespace rapid {
namespace manipulation {
class GripperTest : public ::testing::Test {
 public:
  GripperTest()
      : client_(new MockActionClient<Pr2GripperCommandAction>()),
        gripper_(Gripper::RIGHT_GRIPPER, client_) {}

  void SetUp() {}

 protected:
  MockActionClient<Pr2GripperCommandAction>* client_;
  Gripper gripper_;
};

TEST_F(GripperTest, Open) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.Open();
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0.09, goal.command.position);  // Gripper::OPEN
  EXPECT_EQ(-1, goal.command.max_effort);
}

TEST_F(GripperTest, Close) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.Close(10);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0, goal.command.position);  // Gripper::CLOSED
  EXPECT_EQ(10, goal.command.max_effort);
}

TEST_F(GripperTest, ShouldClampIfPositionTooSmall) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.SetPosition(-1);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(0, goal.command.position);  // Gripper::CLOSED
  EXPECT_EQ(-1, goal.command.max_effort);
  EXPECT_EQ(true, success);
}

TEST_F(GripperTest, ShouldClampIfPositionTooLarge) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  bool success = gripper_.SetPosition(1);
  Pr2GripperCommandGoal goal = client_->last_goal();
  EXPECT_EQ(0.09, goal.command.position);  // Gripper::OPEN
  EXPECT_EQ(-1, goal.command.max_effort);
  EXPECT_EQ(true, success);
}

TEST_F(GripperTest, ShouldFailIfNoServer) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  client_->set_server_delay(ros::DURATION_MAX);
  bool success = gripper_.Open();
  EXPECT_EQ(false, success);
}

TEST_F(GripperTest, ShouldFailIfResultTooSlow) {
  client_->set_state(SimpleClientGoalState::SUCCEEDED);
  client_->set_result_delay(ros::DURATION_MAX);
  bool success = gripper_.Open();
  EXPECT_EQ(false, success);
}

TEST_F(GripperTest, ShouldFailIfStateNotSucceeded) {
  client_->set_state(SimpleClientGoalState::ABORTED);
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
