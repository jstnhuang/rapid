#include "rapid_manipulation/head.h"

#include "actionlib/server/simple_action_server.h"
#include "geometry_msgs/PointStamped.h"
#include "gtest/gtest.h"
#include "pr2_controllers_msgs/PointHeadAction.h"
#include "ros/ros.h"

#include "rapid_ros/action_client.h"

using actionlib::SimpleClientGoalState;
using geometry_msgs::PointStamped;
using pr2_controllers_msgs::PointHeadAction;
using pr2_controllers_msgs::PointHeadGoal;
using rapid_ros::MockActionClient;

namespace rapid {
namespace manipulation {
class HeadTest : public ::testing::Test {
 public:
  HeadTest()
      : client_(new MockActionClient<PointHeadAction>()), head_(client_) {}

  void SetUp() {}

 protected:
  MockActionClient<PointHeadAction>* client_;
  Head head_;
};

TEST_F(HeadTest, LookAtTarget) {
  geometry_msgs::PointStamped target;
  target.point.x = 1;
  target.point.y = 2;
  target.point.z = 3;
  target.header.frame_id = "base_footprint";
  bool success = head_.LookAt(target);
  PointHeadGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(1, goal.target.point.x);
  EXPECT_EQ(2, goal.target.point.y);
  EXPECT_EQ(3, goal.target.point.z);
  EXPECT_EQ("base_footprint", goal.target.header.frame_id);
  EXPECT_EQ("head_mount_kinect_rgb_link", goal.pointing_frame);
  EXPECT_EQ(1, goal.pointing_axis.x);
  EXPECT_EQ(0, goal.pointing_axis.y);
  EXPECT_EQ(0, goal.pointing_axis.z);
}

// If no frame_id is provided, then use a frame that's located in the center of
// the head with the same orientation as base_footprint. We use torso_lift_link
// because none of the head frames have the right orientation.
TEST_F(HeadTest, LookAtTargetWithNoFrame) {
  geometry_msgs::PointStamped target;
  target.point.x = 1;
  target.point.y = 2;
  target.point.z = 0;
  target.header.frame_id = "";
  bool success = head_.LookAt(target);
  PointHeadGoal goal = client_->last_goal();
  EXPECT_EQ(true, success);
  EXPECT_EQ(1, goal.target.point.x);
  EXPECT_EQ(2, goal.target.point.y);
  EXPECT_EQ(0.65, goal.target.point.z);
  EXPECT_EQ("torso_lift_link", goal.target.header.frame_id);
}

TEST_F(HeadTest, ShouldFailOnServerTimeout) {
  client_->set_server_delay(ros::DURATION_MAX);
  geometry_msgs::PointStamped target;
  bool success = head_.LookAt(target);
  EXPECT_EQ(false, success);
}

TEST_F(HeadTest, ShouldFailOnResultTimeout) {
  client_->set_result_delay(ros::DURATION_MAX);
  geometry_msgs::PointStamped target;
  bool success = head_.LookAt(target);
  EXPECT_EQ(false, success);
}

TEST_F(HeadTest, ShouldFailIfNoServer) {
  client_->set_server_connected(false);
  geometry_msgs::PointStamped target;
  bool success = head_.LookAt(target);
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
