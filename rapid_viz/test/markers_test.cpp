#include "rapid_viz/markers.h"

#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "gtest/gtest.h"
#include "visualization_msgs/Marker.h"

#include "rapid_ros/publisher.h"

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using rapid_ros::MockPublisher;
using std::vector;

namespace vmsgs = visualization_msgs;

namespace rapid {
namespace viz {
TEST(MarkerTest, Box) {
  MockPublisher<vmsgs::Marker> pub;
  PoseStamped ps;
  ps.header.frame_id = "base_footprint";
  Vector3 scale;
  scale.x = 2;
  Marker m = Marker::Box(&pub, ps, scale);
  m.Publish();
  vmsgs::Marker sent_msg = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::ADD, sent_msg.action);
  EXPECT_EQ(vmsgs::Marker::CUBE, sent_msg.type);
  EXPECT_EQ("base_footprint", sent_msg.header.frame_id);
  EXPECT_EQ(2, sent_msg.scale.x);
}

// Markers delete themselves from rviz after they are out of scope.
// Also tests a text marker.
TEST(MarkerTest, DeleteAfterOutOfScope) {
  MockPublisher<vmsgs::Marker> pub;
  for (int i = 0; i < 1; ++i) {
    PoseStamped ps;
    Vector3 scale;
    Marker m = Marker::Text(&pub, ps, "object", 0.03);
    m.Publish();
    vmsgs::Marker last_msg = pub.last_message();
    EXPECT_EQ(vmsgs::Marker::ADD, last_msg.action);
    EXPECT_EQ(vmsgs::Marker::TEXT_VIEW_FACING, last_msg.type);
    EXPECT_EQ("object", last_msg.text);
    EXPECT_EQ(0.03, last_msg.scale.z);
  }
  vmsgs::Marker last_msg = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::DELETE, last_msg.action);
  EXPECT_EQ(vmsgs::Marker::TEXT_VIEW_FACING, last_msg.type);
}

// Test copying behavior. When a copy is made, a new ID is generated for that
// marker, so that the copy doesn't delete the original marker when it goes out
// of scope.
//
// Also tests vector marker.
TEST(MarkerTest, CopyAssignment) {
  MockPublisher<vmsgs::Marker> pub;
  for (int i = 0; i < 1; ++i) {
    Point origin;
    Vector3 vector;
    Marker m = Marker::Vector(&pub, "base_footprint", origin, vector);
    m.Publish();
    vmsgs::Marker last_msg_1 = pub.last_message();
    EXPECT_EQ(vmsgs::Marker::ADD, last_msg_1.action);
    EXPECT_EQ(vmsgs::Marker::ARROW, last_msg_1.type);
    EXPECT_EQ("base_footprint", last_msg_1.header.frame_id);
    EXPECT_EQ(1, pub.sent_messages().size());

    Marker m2 = m;
    m2.SetFrame("map");
    m2.Publish();
    vmsgs::Marker last_msg_2 = pub.last_message();
    EXPECT_EQ(vmsgs::Marker::ADD, last_msg_2.action);
    EXPECT_EQ(vmsgs::Marker::ARROW, last_msg_2.type);
    EXPECT_EQ("map", last_msg_2.header.frame_id);
    EXPECT_EQ(2, pub.sent_messages().size());

    EXPECT_FALSE(last_msg_1.id == last_msg_2.id);
  }
  vmsgs::Marker last_msg = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::DELETE, last_msg.action);
  EXPECT_EQ(4, pub.sent_messages().size());
}

TEST(MarkerTest, Delete) {
  MockPublisher<vmsgs::Marker> pub;
  Point origin;
  Vector3 vector;
  Marker m = Marker::Vector(&pub, "base_footprint", origin, vector);
  m.Publish();
  vmsgs::Marker last_msg_1 = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::ADD, last_msg_1.action);
  m.Delete();
  vmsgs::Marker last_msg_2 = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::DELETE, last_msg_2.action);
}

TEST(MarkerTest, DeleteOnReassignment) {
  MockPublisher<vmsgs::Marker> pub;
  Point origin;
  Vector3 vector;
  Marker m = Marker::Vector(&pub, "base_footprint", origin, vector);
  int m_id = m.marker().id;
  m.Publish();
  EXPECT_EQ(1, pub.sent_messages().size());
  vmsgs::Marker last_msg_1 = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::ADD, last_msg_1.action);
  EXPECT_EQ(m_id, last_msg_1.id);

  PoseStamped ps;
  Vector3 scale;
  m = Marker::Box(&pub, ps, vector);
  // 2 deletes - one for static Marker::Box, and one for old value of m.
  EXPECT_EQ(3, pub.sent_messages().size());
  vmsgs::Marker last_msg_2 =
      pub.sent_messages()[pub.sent_messages().size() - 2];
  EXPECT_EQ(vmsgs::Marker::DELETE, last_msg_2.action);
  EXPECT_EQ(m_id, last_msg_2.id);

  m.Publish();
  EXPECT_EQ(4, pub.sent_messages().size());
  vmsgs::Marker last_msg_3 = pub.last_message();
  EXPECT_EQ(vmsgs::Marker::ADD, last_msg_3.action);

  int m3_id = m.marker().id;

  m = Marker::Box(&pub, ps, vector);
  EXPECT_EQ(6, pub.sent_messages().size());
  vmsgs::Marker last_msg_4 =
      pub.sent_messages()[pub.sent_messages().size() - 2];
  EXPECT_EQ(vmsgs::Marker::DELETE, last_msg_4.action);
  EXPECT_EQ(m3_id, last_msg_4.id);
}

TEST(MarkerTest, DoesNothingWithInvalidPublisher) {
  MockPublisher<vmsgs::Marker> pub;
  pub.set_valid(false);
  PoseStamped ps;
  Vector3 scale;
  Marker m = Marker::Box(&pub, ps, scale);
  m.Publish();
  EXPECT_EQ(0, pub.sent_messages().size());
}

TEST(MarkerTest, NullMarkerDoesNothing) {
  Marker m = Marker::Null();
  m.Publish();
  // Test passes if no segfault.
}

TEST(MarkerTest, OutlineBox) {
  MockPublisher<vmsgs::Marker> pub;
  PoseStamped ps;
  ps.header.frame_id = "map";
  Vector3 scale;
  scale.x = 2;
  scale.y = 4;
  scale.z = 6;
  Marker m = Marker::OutlineBox(&pub, ps, scale);
  m.Publish();
  EXPECT_EQ(1, pub.sent_messages().size());
  vmsgs::Marker last_msg = pub.last_message();
  EXPECT_EQ("map", last_msg.header.frame_id);
  // 12 edges per box * 2 points per edge.
  EXPECT_EQ(24, last_msg.points.size());
}
}  // namespace viz
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
