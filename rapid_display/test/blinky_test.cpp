// Test for Blinky display.

#include "rapid_display/display.h"

#include <string>
#include <vector>

#include "actionlib/server/simple_action_server.h"
#include "blinky/FaceAction.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

#include "rapid_ros/action_client.h"

using ::testing::Eq;
using blinky::FaceAction;
using blinky::FaceGoal;
using blinky::FaceResult;
using rapid_ros::ActionClientInterface;
using rapid_ros::MockActionClient;

namespace rapid {
namespace display {
class BlinkyTest : public ::testing::Test {
 public:
  // We have a long server wait time to make sure the test isn't flaky.
  BlinkyTest()
      : client_(new MockActionClient<FaceAction>()), blinky_(client_) {}

  void SetUp() { blinky_.set_server_wait_time(5); }

 protected:
  MockActionClient<FaceAction>* client_;
  Blinky blinky_;
};

TEST_F(BlinkyTest, ShowDefault) {
  bool success = blinky_.ShowDefault();
  FaceGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(FaceGoal::DEFAULT, goal.display_type);
}

TEST_F(BlinkyTest, ShouldFailIfServerTimesOut) {
  client_->set_server_delay(::ros::DURATION_MAX);
  bool success = blinky_.ShowDefault();
  EXPECT_EQ(false, success);
}

TEST_F(BlinkyTest, ShouldFailIfResultTimesOut) {
  client_->set_result_delay(::ros::DURATION_MAX);
  bool success = blinky_.ShowDefault();
  EXPECT_EQ(false, success);
}

TEST_F(BlinkyTest, ShowMessage) {
  bool success = blinky_.ShowMessage("Hello world!", "I'm ready to help.");
  FaceGoal goal = client_->last_goal();
  EXPECT_EQ(FaceGoal::DISPLAY_MESSAGE, goal.display_type);
  EXPECT_EQ("Hello world!", goal.h1_text);
  EXPECT_EQ("I'm ready to help.", goal.h2_text);
  EXPECT_EQ(true, success);
}

TEST_F(BlinkyTest, AskMultipleChoice) {
  std::vector<std::string> choices;
  choices.push_back("Indigo");
  choices.push_back("Jade");

  FaceResult result;
  result.choice = "Indigo";
  client_->set_result(result);

  std::string choice;
  bool success = blinky_.AskMultipleChoice("What's your favorite color?",
                                           choices, &choice);
  FaceGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(FaceGoal::ASK_CHOICE, goal.display_type);
  EXPECT_THAT(goal.choices, Eq(choices));
  EXPECT_EQ("Indigo", choice);
}

TEST_F(BlinkyTest, FailIfNullClient) {
  Blinky blinky(NULL);
  bool success = blinky.ShowDefault();
  EXPECT_EQ(false, success);
  success = blinky.ShowMessage("H1", "H2");
  EXPECT_EQ(false, success);
  std::vector<std::string> choices;
  std::string choice;
  success = blinky.AskMultipleChoice("Question", choices, &choice);
  EXPECT_EQ(false, success);
  Blinky* blinky_p(new Blinky(NULL));
  delete blinky_p;
}

TEST_F(BlinkyTest, FailIfServerConnectionSlow) {
  client_->set_server_delay(ros::DURATION_MAX);
  bool success = blinky_.ShowDefault();
  EXPECT_EQ(false, success);
  success = blinky_.ShowMessage("H1", "H2");
  EXPECT_EQ(false, success);
  std::vector<std::string> choices;
  std::string choice;
  success = blinky_.AskMultipleChoice("Question", choices, &choice);
  EXPECT_EQ(false, success);
}

TEST_F(BlinkyTest, FailIfResultTooSlow) {
  client_->set_result_delay(ros::DURATION_MAX);
  bool success = blinky_.ShowDefault();
  EXPECT_EQ(false, success);
  success = blinky_.ShowMessage("H1", "H2");
  EXPECT_EQ(false, success);
}
}  // namespace display
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blinky_test");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
