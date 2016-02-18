// Integration test for Blinky class.
// The test node implements a mock Blinky actionlib server, which we test
// against when using the Blinky class.

#include "rapid/display/display.h"

#include <string>
#include <vector>

#include "actionlib/server/simple_action_server.h"
#include "blinky/FaceAction.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "ros/ros.h"

using ::testing::Pointwise;
using ::testing::Eq;
using actionlib::SimpleActionServer;
using blinky::FaceAction;
using blinky::FaceGoal;
using blinky::FaceGoalConstPtr;
using blinky::FaceResult;

namespace rapid {
namespace display {

class MockBlinkyServer {
 public:
  MockBlinkyServer()
      : server_queue_(),
        nh_(),
        server_(nh_, "blinky",
                boost::bind(&MockBlinkyServer::ExecuteCb, this, _1), false),
        last_goal_() {
    nh_.setCallbackQueue(&server_queue_);
  }
  void Start() { server_.start(); }

  // Processes all available callbacks, and gets the most recent goal that was
  // processed. If you are testing with the MockBlinkyServer, you are guaranteed
  // that the execute callback was processed after calling this method.
  //
  // Inversely, the execute callback will never be processed until someone
  // calls this method. So, all test cases need to call this method.
  //
  // See roscpp documentation on callbacks and spinning.
  FaceGoal WaitForGoal() {
    server_queue_.callAvailable();
    return last_goal_;
  }

 protected:
  void ExecuteCb(const FaceGoalConstPtr& goal) {
    last_goal_ = *goal;
    if (goal->display_type == FaceGoal::ASK_CHOICE) {
      FaceResult result;
      result.choice = goal->choices[0];
      server_.setSucceeded(result);
    }
  }
  ros::CallbackQueue server_queue_;  // Used as a synchronization mechanism.
  ros::NodeHandle nh_;
  SimpleActionServer<FaceAction> server_;
  FaceGoal last_goal_;
};

class BlinkyTest : public ::testing::Test {
 public:
  BlinkyTest() : node_handle_(), server_(), blinky_(10) {}

  void SetUp() {}

 protected:
  ros::NodeHandle node_handle_;
  MockBlinkyServer server_;
  Blinky blinky_;
};

TEST_F(BlinkyTest, ShowDefault) {
  server_.Start();
  bool success = blinky_.ShowDefault();
  FaceGoal goal = server_.WaitForGoal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(FaceGoal::DEFAULT, goal.display_type);
}

TEST_F(BlinkyTest, ShouldFailIfServerNotStarted) {
  // By not calling server_.Start(), the action server should not exist.
  bool success = blinky_.ShowDefault();
  FaceGoal goal = server_.WaitForGoal();
  EXPECT_EQ(false, success);
}

TEST_F(BlinkyTest, ShowMessage) {
  server_.Start();
  bool success = blinky_.ShowMessage("Hello world!", "I'm ready to help.");
  FaceGoal goal = server_.WaitForGoal();
  EXPECT_EQ(FaceGoal::DISPLAY_MESSAGE, goal.display_type);
  EXPECT_EQ("Hello world!", goal.h1_text);
  EXPECT_EQ("I'm ready to help.", goal.h2_text);
  EXPECT_EQ(true, success);
}

TEST_F(BlinkyTest, AskMultipleChoice) {
  server_.Start();
  std::vector<std::string> choices;
  choices.push_back("Indigo");
  choices.push_back("Jade");
  std::string choice;
  bool success = blinky_.AskMultipleChoice("What's your favorite color?",
                                           choices, &choice);
  EXPECT_EQ(true, success);
  EXPECT_EQ("Indigo", choice);
  FaceGoal goal = server_.WaitForGoal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(FaceGoal::ASK_CHOICE, goal.display_type);
  EXPECT_THAT(goal.choices, Eq(choices));
}
}  // namespace display
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blinky_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
