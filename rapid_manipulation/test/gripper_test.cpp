#include "rapid_manipulation/gripper.h"

#include "actionlib/server/simple_action_server.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pr2_controllers_msgs/Pr2GripperCommandAction.h"
#include "ros/ros.h"

using actionlib::SimpleActionServer;
using pr2_controllers_msgs::Pr2GripperCommandAction;
using pr2_controllers_msgs::Pr2GripperCommandGoal;
using pr2_controllers_msgs::Pr2GripperCommandResult;

namespace rapid {
namespace manipulation {
class MockGripperServer {
 public:
  MockGripperServer()
      : nh_(),
        server_(nh_, "r_gripper_controller/gripper_action",
                boost::bind(&MockGripperServer::ExecuteCb, this, _1), false),
        last_goal_() {}
  void Start() { server_.start(); }

  // Processes all available callbacks, and gets the most recent goal that was
  // processed. If you are testing with the MockBlinkyServer, you are guaranteed
  // that the execute callback was processed after calling this method.
  //
  // See roscpp documentation on callbacks and spinning.
  Pr2GripperCommandGoal WaitForGoal() {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    return last_goal_;
  }

 protected:
  void ExecuteCb(const Pr2GripperCommandGoal::ConstPtr& goal) {
    last_goal_ = *goal;
    Pr2GripperCommandResult result;
    server_.setSucceeded(result);
  }
  ros::NodeHandle nh_;
  SimpleActionServer<Pr2GripperCommandAction> server_;
  Pr2GripperCommandGoal last_goal_;
};

class GripperTest : public ::testing::Test {
 public:
  GripperTest() : node_handle_(), server_(), gripper_(Gripper::RIGHT_GRIPPER) {}

  void SetUp() {}

 protected:
  ros::NodeHandle node_handle_;
  MockGripperServer server_;
  Gripper gripper_;
};

TEST_F(GripperTest, Open) {
  server_.Start();
  bool success = gripper_.Open();
  Pr2GripperCommandGoal goal = server_.WaitForGoal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0.09, goal.command.position);  // Gripper::OPEN
  EXPECT_EQ(-1, goal.command.max_effort);
}

TEST_F(GripperTest, Close) {
  server_.Start();
  bool success = gripper_.Close(10);
  Pr2GripperCommandGoal goal = server_.WaitForGoal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(0, goal.command.position);
  EXPECT_EQ(10, goal.command.max_effort);
}

TEST_F(GripperTest, ShouldFailIfPositionTooSmall) {
  server_.Start();
  bool success = gripper_.SetPosition(-1, -1);
  Pr2GripperCommandGoal goal = server_.WaitForGoal();
  EXPECT_EQ(false, success);
}

TEST_F(GripperTest, ShouldFailIfPositionTooLarge) {
  server_.Start();
  bool success = gripper_.SetPosition(1, -1);
  Pr2GripperCommandGoal goal = server_.WaitForGoal();
  EXPECT_EQ(false, success);
}

TEST_F(GripperTest, ShouldFailIfNoServer) {
  // We do not call server_.Start() in this test case.
  bool success = gripper_.Open();
  Pr2GripperCommandGoal goal = server_.WaitForGoal();
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
