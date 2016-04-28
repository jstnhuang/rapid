#include "rapid_manipulation/pick.h"

#include "agile_grasp/FindGrasps.h"
#include "agile_grasp/Grasp.h"
#include "agile_grasp/Grasps.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_perception/object.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_ros/publisher.h"
#include "rapid_ros/service_client.h"

using agile_grasp::Grasp;
using agile_grasp::Grasps;
using agile_grasp::FindGrasps;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using rapid::perception::Object;
using rapid_ros::MockServiceClient;
using rapid_ros::ServiceClientInterface;
using visualization_msgs::Marker;
using ::testing::Field;

namespace rapid {
namespace manipulation {
class PickerTest : public ::testing::Test {
 public:
  PickerTest();
  void SetUp() {}

 protected:
  MockArm* arm_;
  MockGripper* gripper_;
  MockServiceClient<FindGrasps>* grasp_gen_client_;
  rapid_ros::MockPublisher<Marker>* marker_pub_;
  Picker picker_;
};

PickerTest::PickerTest()
    : arm_(new MockArm()),
      gripper_(new MockGripper()),
      grasp_gen_client_(new MockServiceClient<FindGrasps>()),
      marker_pub_(new rapid_ros::MockPublisher<Marker>()),
      picker_(arm_, gripper_, grasp_gen_client_, marker_pub_) {}

TEST_F(PickerTest, GenerateGrasps) {
  FindGrasps::Response resp;
  resp.grasps.header.frame_id = "camera_frame";
  grasp_gen_client_->set_response(resp);
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointIndices::Ptr indices(new PointIndices);
  Object obj;
  obj.SetCloud(cloud, indices);

  Grasps grasps_response;
  bool success = picker_.GenerateGrasps(obj, &grasps_response);
  EXPECT_TRUE(success);
  EXPECT_EQ("camera_frame", grasps_response.header.frame_id);

  grasp_gen_client_->set_success(false);
  success = picker_.GenerateGrasps(obj, &grasps_response);
  EXPECT_EQ(false, success);
}

TEST_F(PickerTest, PickFailsIfNoGrasps) {
  FindGrasps::Response resp;
  resp.grasps.header.frame_id = "camera_frame";
  grasp_gen_client_->set_response(resp);
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointIndices::Ptr indices(new PointIndices);
  Object obj;
  obj.SetCloud(cloud, indices);

  bool success = picker_.Pick(obj);
  EXPECT_EQ(false, success);
}

TEST_F(PickerTest, PickFailsIfGraspGenFails) {
  grasp_gen_client_->set_success(false);
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointIndices::Ptr indices(new PointIndices);
  Object obj;
  obj.SetCloud(cloud, indices);

  bool success = picker_.Pick(obj);
  EXPECT_EQ(false, success);
}

TEST_F(PickerTest, VizShouldBePublished) {
  FindGrasps::Response resp;
  Grasp grasp;
  grasp.surface_center.x = 1;
  grasp.surface_center.y = 2;
  grasp.surface_center.z = 3;
  grasp.approach.x = 1;  // Grasp from the front.
  grasp.axis.z = 1;
  resp.grasps.grasps.push_back(grasp);
  grasp_gen_client_->set_response(resp);
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointIndices::Ptr indices(new PointIndices);
  Object obj;
  obj.SetCloud(cloud, indices);

  picker_.Pick(obj);

  // 2 vectors + 2 deletion messages.
  ASSERT_EQ(4, marker_pub_->sent_messages().size());
  // If the grasp frame is the empty frame (which seems to be the case with
  // agile_grasp results), then the grasp frame should be the Kinect optical
  // frame.
  Marker marker = marker_pub_->sent_messages().at(1);
  EXPECT_EQ("/head_mount_kinect_rgb_optical_frame", marker.header.frame_id);
}

// gmock returns false by default, so arm movements with MockArm fail.
// If we say that arm movements succeed, then the grasp should succeed.
TEST_F(PickerTest, SuccessfulGrasp) {
  FindGrasps::Response resp;
  Grasp grasp;
  grasp.surface_center.x = 1;
  grasp.surface_center.y = 2;
  grasp.surface_center.z = 3;
  grasp.approach.x = 1;  // Grasp from the front.
  grasp.axis.z = 1;
  resp.grasps.grasps.push_back(grasp);
  grasp_gen_client_->set_response(resp);
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointIndices::Ptr indices(new PointIndices);
  Object obj;
  obj.SetCloud(cloud, indices);

  EXPECT_CALL(*arm_, MoveToPoseGoal(::testing::_))
      .Times(2)
      .WillRepeatedly(::testing::Return(true));

  bool success = picker_.Pick(obj);
  EXPECT_EQ(true, success);
}

TEST_F(PickerTest, MoveToPreGrasp) {
  PoseStamped grasp_pose;
  grasp_pose.pose.position.x = 0.5;
  grasp_pose.pose.position.z = 0.5;
  Vector3 grasp_approach;
  grasp_approach.x = 2;  // Tests normalization

  ros::param::set("pregrasp_distance", 0.3);

  picker_.MoveToPreGrasp(grasp_pose, grasp_approach);

  EXPECT_CALL(*arm_,
              MoveToPoseGoal(::testing::AllOf(
                  Field(&PoseStamped::pose,
                        Field(&Pose::position,
                              Field(&Point::x, ::testing::DoubleEq(0.2)))),
                  Field(&PoseStamped::pose,
                        Field(&Pose::position,
                              Field(&Point::z, ::testing::DoubleEq(0.5)))))))
      .WillOnce(::testing::Return(true));
}

TEST_F(PickerTest, MoveToGrasp) {
  PoseStamped grasp_pose;
  grasp_pose.pose.position.x = 0.5;
  grasp_pose.pose.position.z = 0.5;
  Vector3 grasp_approach;
  grasp_approach.x = 2;  // Tests normalization

  ros::param::set("grasp_distance", 0.1);

  picker_.MoveToGrasp(grasp_pose, grasp_approach);

  EXPECT_CALL(*arm_,
              MoveToPoseGoal(::testing::AllOf(
                  Field(&PoseStamped::pose,
                        Field(&Pose::position,
                              Field(&Point::x, ::testing::DoubleEq(0.4)))),
                  Field(&PoseStamped::pose,
                        Field(&Pose::position,
                              Field(&Point::z, ::testing::DoubleEq(0.5)))))))
      .WillOnce(::testing::Return(true));
}

TEST(PickTest, ComputeGraspOrientation) {
  // Simple test where the grasp points directly forward, should be identity
  // rotation.
  geometry_msgs::Quaternion q;
  Vector3 approach;
  approach.x = 1;
  Vector3 axis;
  axis.z = 1;
  ComputeGraspOrientation(approach, axis, &q);
  EXPECT_EQ(1, q.w);
  EXPECT_EQ(0, q.x);
  EXPECT_EQ(0, q.y);
  EXPECT_EQ(0, q.z);
}
}  // namespace manipulation
}  // namespace rapid

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
