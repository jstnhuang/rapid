#include "rapid_manipulation/tuck_arms.h"

#include "gtest/gtest.h"
#include "pr2_common_action_msgs/TuckArmsAction.h"

#include "rapid_ros/action_client.h"

using actionlib::SimpleClientGoalState;
using rapid_ros::MockActionClient;
using pr2_common_action_msgs::TuckArmsAction;
using pr2_common_action_msgs::TuckArmsGoal;

namespace rapid {
namespace manipulation {
class TuckArmsTest : public ::testing::Test {
 public:
  TuckArmsTest()
      : client_(new MockActionClient<TuckArmsAction>()), tuck_arms_(client_) {}

  void SetUp() {}

 protected:
  MockActionClient<TuckArmsAction>* client_;
  Pr2TuckArms tuck_arms_;
};

TEST_F(TuckArmsTest, SimpleTuck) {
  bool success = tuck_arms_.TuckArms();
  TuckArmsGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(1, goal.tuck_left);
  EXPECT_EQ(1, goal.tuck_right);
}

TEST_F(TuckArmsTest, DeployLeft) {
  bool success = tuck_arms_.DeployLeft();
  TuckArmsGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0, goal.tuck_left);
  EXPECT_EQ(1, goal.tuck_right);
}

TEST_F(TuckArmsTest, DeployRight) {
  bool success = tuck_arms_.DeployRight();
  TuckArmsGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(1, goal.tuck_left);
  EXPECT_EQ(0, goal.tuck_right);
}

TEST_F(TuckArmsTest, DeployArms) {
  bool success = tuck_arms_.DeployArms();
  TuckArmsGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0, goal.tuck_left);
  EXPECT_EQ(0, goal.tuck_right);
}

TEST_F(TuckArmsTest, ServerConnectionSlow) {
  client_->set_server_delay(ros::DURATION_MAX);
  bool success = tuck_arms_.TuckArms();
  EXPECT_EQ(false, success);
}

TEST_F(TuckArmsTest, ServerNotConnected) {
  client_->set_server_connected(false);
  bool success = tuck_arms_.TuckArms();
  EXPECT_EQ(false, success);
}

TEST_F(TuckArmsTest, ResultTooSlow) {
  client_->set_result_delay(ros::DURATION_MAX);
  bool success = tuck_arms_.TuckArms();
  EXPECT_EQ(false, success);
}

TEST_F(TuckArmsTest, SetServerWaitTime) {
  client_->set_server_delay(ros::Duration(30)); // 30 > default of 20 seconds.
  bool success = tuck_arms_.TuckArms();
  EXPECT_EQ(false, success);

  tuck_arms_.set_server_wait_time(60);
  success = tuck_arms_.TuckArms();
  EXPECT_EQ(true, success);
}
}  // namespace manipulation
}  //  namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tuck_arms_test");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
