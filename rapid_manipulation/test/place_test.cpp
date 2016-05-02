#include "rapid_manipulation/place.h"

#include "Eigen/Dense"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_ros/publisher.h"
#include "rapid_viz/markers.h"
#include "rapid_utils/math.h"
#include "ros/ros.h"

using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using rapid::perception::HSurface;
using rapid::perception::Object;
using rapid::utils::DegreesToRadians;
using rapid::viz::MarkerPub;
using rapid_ros::MockPublisher;
using visualization_msgs::Marker;
using ::testing::Return;

namespace rapid {
namespace manipulation {
class PlacerTest : public ::testing::Test {
 public:
  PlacerTest();
  void SetUp() {}

  static HSurface Table1();
  static Object Object1();

 protected:
  MockArm* arm_;
  MockGripper* gripper_;
  MockPublisher<Marker>* marker_pub_;
  Placer placer_;
};

PlacerTest::PlacerTest()
    : arm_(new MockArm()),
      gripper_(new MockGripper()),
      marker_pub_(new MockPublisher<Marker>()),
      placer_(arm_, gripper_, marker_pub_) {}

// A simulated table. The +x axis points left and it is longer in the x
// direction than the y direction.
HSurface PlacerTest::Table1() {
  HSurface table;
  Vector3 scale;
  scale.x = 1;
  scale.y = 0.5;
  scale.z = 0.02;
  table.set_scale(scale);
  PoseStamped ps;
  ps.pose.position.x = 1;
  ps.pose.position.z = 1;
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(DegreesToRadians(-90), Eigen::Vector3d::UnitZ());
  geometry_msgs::Quaternion ros_q;
  ros_q.w = q.w();
  ros_q.x = q.x();
  ros_q.y = q.y();
  ros_q.z = q.z();
  ps.pose.orientation = ros_q;
  table.set_pose(ps);
  table.set_name("table");
  return table;
}

Object PlacerTest::Object1() {
  Object object;
  Vector3 scale;
  scale.x = 0.04;
  scale.y = 0.02;
  scale.z = 0.06;
  object.set_scale(scale);
  object.set_name("object_1");
  return object;
}

TEST_F(PlacerTest, SampleRandomPlacement) {
  HSurface table = Table1();
  double obj_height = 0.05;

  PointStamped placement;
  ros::param::set("distance_from_edge", 0.05);
  bool success = SampleRandomPlacement(obj_height, table, &placement);
  EXPECT_TRUE(success);  // No obstacles.
  EXPECT_FLOAT_EQ(1 + 0.01 + 0.025, placement.point.z);
  // Table points left and is 1m long, so in base frame, the y bounds are:
  // [-0.5, 0.5], but to avoid placing object on the edge, the bounds are:
  // [-0.45, 0.45]
  // The x bounds in the base frame are: [1 - 0.25 + 0.05, 1 + 0.25 - 0.05] =
  // [0.8, 1.2]
  EXPECT_GE(placement.point.y, -0.45);
  EXPECT_LE(placement.point.y, 0.45);
  EXPECT_GE(placement.point.x, 0.8);
  EXPECT_LE(placement.point.x, 1.2);
}

// The object will be on the left side of the table. The obstacle distance
// parameter will be set such that the object can't be placed, roughly speaking,
// anywhere on the left side of the table.
TEST_F(PlacerTest, SampleRandomPlacmentAvoidsOtherObjects) {
  HSurface table = Table1();
  Object obj;
  PoseStamped obj_ps;  // Pose of object in base frame.
  // Place the object on the center of the left edge of the table.
  obj_ps.pose.position.x = 1;
  obj_ps.pose.position.y = -0.5;
  obj_ps.pose.position.z = 1 + 0.01 + 0.025;
  obj.set_pose(obj_ps);
  obj.set_name("table_object1");
  table.AddObject(obj);

  PointStamped placement;
  ros::param::set("distance_from_edge", 0.05);
  ros::param::set("obstacle_distance", 0.5);
  ros::param::set("sample_placement_max_tries", 100);
  double obj_height = 0.05;
  bool success = SampleRandomPlacement(obj_height, table, &placement);
  EXPECT_TRUE(success);  // This should be true with high probability.

  double xd = placement.point.x - obj_ps.pose.position.x;
  double yd = placement.point.y - obj_ps.pose.position.y;
  double squared_distance = xd * xd + yd * yd;
  EXPECT_GE(squared_distance, 0.5 * 0.5);
}

TEST_F(PlacerTest, SuccessfulPlace) {
  HSurface table = Table1();
  Object object = Object1();
  EXPECT_CALL(*arm_, MoveToPoseGoal(::testing::_))
      .Times(3)
      .WillRepeatedly(Return(true));
  bool success = placer_.Place(object, table);
  EXPECT_TRUE(success);
}

TEST_F(PlacerTest, VizShouldBePublished) {
  HSurface table = Table1();
  Object object = Object1();
  EXPECT_CALL(*arm_, MoveToPoseGoal(::testing::_))
      .Times(3)
      .WillRepeatedly(Return(true));
  placer_.Place(object, table);
  EXPECT_EQ(2, marker_pub_->sent_messages().size());
}

TEST_F(PlacerTest, GripperStaysClosedOnFailedMotionPlan) {
  HSurface table = Table1();
  Object object = Object1();
  EXPECT_CALL(*arm_, MoveToPoseGoal(::testing::_))
      .Times(3)
      .WillOnce(Return(true))
      .WillRepeatedly(Return(false));
  EXPECT_CALL(*gripper_, Open(::testing::_)).Times(0);
  ros::param::set("place_max_tries", 2);
  bool success = placer_.Place(object, table);
  EXPECT_EQ(false, success);
}

TEST_F(PlacerTest, WillRetryBeforeFailing) {
  HSurface table = Table1();
  Object object = Object1();
  EXPECT_CALL(*arm_, MoveToPoseGoal(::testing::_))
      .Times(2)
      .WillRepeatedly(Return(false));
  EXPECT_CALL(*gripper_, Open(::testing::_)).Times(0);
  ros::param::set("place_max_tries", 2);
  bool success = placer_.Place(object, table);
  EXPECT_EQ(false, success);
}
}  // namespace manipulation
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "place_test");

  ros::AsyncSpinner spinner(2);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
