#include "rapid_perception/scene_viz.h"

#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "gtest/gtest.h"
#include "rapid_perception/scene.h"
#include "rapid_ros/publisher.h"
#include "visualization_msgs/Marker.h"

using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using rapid_ros::MockPublisher;
using visualization_msgs::Marker;
using rapid::perception::Object;
using std::vector;

namespace rapid {
namespace perception {
class SceneVizTest : public ::testing::Test {
 public:
  SceneVizTest()
      : marker_pub_(new MockPublisher<Marker>), scene_viz(marker_pub_) {}

  void SetUp() {}

  static Object Object1();
  static HSurface Table1();
  static Scene Scene1();

 protected:
  rapid_ros::MockPublisher<Marker>* marker_pub_;
  SceneViz scene_viz;
};

Object SceneVizTest::Object1() {
  Object object;
  object.set_name("object_1");
  PoseStamped obj_ps;
  obj_ps.header.frame_id = "base_footprint";
  obj_ps.pose.position.x = 1;
  obj_ps.pose.position.y = 0;
  obj_ps.pose.position.z = 0.85;
  obj_ps.pose.orientation.w = 1;
  object.set_pose(obj_ps);
  Vector3 obj_scale;
  obj_scale.x = 0.03;
  obj_scale.y = 0.02;
  obj_scale.z = 0.1;
  object.set_scale(obj_scale);
  return object;
}

HSurface SceneVizTest::Table1() {
  HSurface table;
  table.set_name("table");
  PoseStamped table_ps;
  table_ps.header.frame_id = "base_footprint";
  table_ps.pose.position.x = 1;
  table_ps.pose.position.y = 0;
  table_ps.pose.position.z = 0.75;
  table_ps.pose.orientation.w = 1;
  table.set_pose(table_ps);
  Vector3 table_scale;
  table_scale.x = 1;
  table_scale.y = 0.5;
  table_scale.z = 0.1;
  table.set_scale(table_scale);
  return table;
}

Scene SceneVizTest::Scene1() {
  Scene scene;
  PoseStamped scene_ps;
  scene_ps.header.frame_id = "base_footprint";
  scene_ps.pose.position.x = 0.5;
  scene_ps.pose.position.y = 0;
  scene_ps.pose.position.z = 1;
  scene.set_pose(scene_ps);
  Vector3 scene_scale;
  scene_scale.x = 1;
  scene_scale.y = 1;
  scene_scale.z = 1;
  scene.set_scale(scene_scale);
  return scene;
}

TEST_F(SceneVizTest, ObjectViz) {
  Object object = Object1();
  ObjectViz viz(marker_pub_);
  viz.set_object(object);
  viz.Visualize();
  const vector<Marker>& messages = marker_pub_->sent_messages();
  bool box = false;
  bool text = false;
  for (size_t i = 0; i < messages.size(); ++i) {
    const Marker& marker = messages[i];
    if (marker.action == marker.ADD) {
      if (marker.type == marker.CUBE) {
        box = true;
        EXPECT_EQ("object_1", marker.ns);
        EXPECT_EQ(0.85, marker.pose.position.z);
        EXPECT_EQ(0.1, marker.scale.z);
      } else if (marker.type == marker.TEXT_VIEW_FACING) {
        text = true;
        EXPECT_EQ("object_1", marker.text);
        EXPECT_EQ("object_1", marker.ns);
      }
    }
  }
  EXPECT_TRUE(box);
  EXPECT_TRUE(text);
}

TEST_F(SceneVizTest, HSurfaceViz) {
  Object object = Object1();
  HSurface table = Table1();
  table.AddObject(object);
  HSurfaceViz viz(marker_pub_);
  viz.set_hsurface(table);
  viz.Visualize();
  const vector<Marker>& messages = marker_pub_->sent_messages();
  bool object_was_published = false;
  bool table_was_published = false;
  bool text_was_published = false;
  for (size_t i = 0; i < messages.size(); ++i) {
    const Marker& marker = messages[i];
    if (marker.action == marker.ADD) {
      if (marker.type == marker.CUBE) {
        if (marker.ns == "table") {
          table_was_published = true;
          EXPECT_EQ(1, marker.scale.x);
        } else if (marker.ns == "object_1") {
          object_was_published = true;
          EXPECT_EQ("object_1", marker.ns);
          EXPECT_EQ(0.85, marker.pose.position.z);
          EXPECT_EQ(0.1, marker.scale.z);
        }
      } else if (marker.type == marker.TEXT_VIEW_FACING) {
        text_was_published = true;
        EXPECT_EQ("object_1", marker.text);
        EXPECT_EQ("object_1", marker.ns);
      }
    }
  }
  EXPECT_TRUE(table_was_published);
  EXPECT_TRUE(object_was_published);
  EXPECT_TRUE(text_was_published);
}

TEST_F(SceneVizTest, SceneVizViz) {
  Object object = Object1();
  HSurface table = Table1();
  Scene scene = Scene1();
  table.AddObject(object);
  scene.set_primary_surface(table);
  SceneViz viz(marker_pub_);
  viz.set_scene(scene);
  viz.Visualize();
  const vector<Marker>& messages = marker_pub_->sent_messages();
  bool scene_was_published = false;
  bool object_was_published = false;
  bool table_was_published = false;
  bool text_was_published = false;
  for (size_t i = 0; i < messages.size(); ++i) {
    const Marker& marker = messages[i];
    if (marker.action == marker.ADD) {
      if (marker.type == marker.CUBE) {
        if (marker.ns == "table") {
          table_was_published = true;
          EXPECT_EQ(1, marker.scale.x);
        } else if (marker.ns == "object_1") {
          object_was_published = true;
          EXPECT_EQ("object_1", marker.ns);
          EXPECT_EQ(0.85, marker.pose.position.z);
          EXPECT_EQ(0.1, marker.scale.z);
        }
      } else if (marker.type == marker.TEXT_VIEW_FACING) {
        text_was_published = true;
        EXPECT_EQ("object_1", marker.text);
        EXPECT_EQ("object_1", marker.ns);
      } else if (marker.type == marker.LINE_LIST) {
        scene_was_published = true;
        EXPECT_EQ(24, marker.points.size());
      }
    }
  }
  EXPECT_TRUE(scene_was_published);
  EXPECT_TRUE(table_was_published);
  EXPECT_TRUE(object_was_published);
  EXPECT_TRUE(text_was_published);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
