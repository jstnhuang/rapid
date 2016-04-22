// Simple tests of the Scene model classes (Object, HSurface, and Scene).
#include "rapid_perception/object.h"
#include "rapid_perception/hsurface.h"
#include "rapid_perception/scene.h"

#include <stdlib.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "gtest/gtest.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointIndices;

namespace rapid {
namespace perception {
TEST(ObjectTest, SetCloud) {
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointXYZRGB p;
  p.x = 1;
  cloud->push_back(p);
  p.x = 2;
  cloud->push_back(p);
  p.x = 3;
  cloud->push_back(p);
  PointIndices::Ptr indices(new PointIndices);
  indices->indices.push_back(0);
  indices->indices.push_back(2);

  Object object;
  object.SetCloud(cloud, indices);

  PointCloud<PointXYZRGB>::Ptr output = object.GetCloud();
  EXPECT_EQ(2, output->size());
  EXPECT_EQ(1, output->points[0].x);
  EXPECT_EQ(3, output->points[1].x);
}

TEST(ObjectTest, SetMembers) {
  Object object;
  object.set_name("object_0");
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "base_footprint";
  geometry_msgs::Vector3 scale;
  scale.x = 2;
  object.set_pose(ps);
  object.set_scale(scale);
  EXPECT_EQ("object_0", object.name());
  EXPECT_EQ("base_footprint", object.pose().header.frame_id);
  EXPECT_EQ(2, object.scale().x);
}

TEST(HSurfaceTest, SetCloud) {
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointXYZRGB p;
  p.x = 1;
  cloud->push_back(p);
  p.x = 2;
  cloud->push_back(p);
  p.x = 3;
  cloud->push_back(p);
  PointIndices::Ptr indices(new PointIndices);
  indices->indices.push_back(0);
  indices->indices.push_back(2);

  HSurface surface;
  surface.SetCloud(cloud, indices);

  PointCloud<PointXYZRGB>::Ptr output = surface.GetCloud();
  EXPECT_EQ(2, output->size());
  EXPECT_EQ(1, output->points[0].x);
  EXPECT_EQ(3, output->points[1].x);
}

TEST(HSurfaceTest, SetMembers) {
  HSurface surface;
  surface.set_name("primary_surface");
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "base_footprint";
  geometry_msgs::Vector3 scale;
  scale.x = 2;
  surface.set_pose(ps);
  surface.set_scale(scale);
  EXPECT_EQ("primary_surface", surface.name());
  EXPECT_EQ("base_footprint", surface.pose().header.frame_id);
  EXPECT_EQ(2, surface.scale().x);
}

TEST(HSurfaceTest, FindObject) {
  HSurface surface;
  Object o1;
  o1.set_name("object_1");
  Object o2;
  o2.set_name("object_2");
  EXPECT_EQ(0, surface.objects().size());
  surface.AddObject(o1);
  surface.AddObject(o2);
  EXPECT_EQ(2, surface.objects().size());

  Object obj;
  bool found = surface.GetObject("object_1", &obj);
  EXPECT_TRUE(found);
  EXPECT_EQ("object_1", obj.name());

  Object obj2;
  found = surface.GetObject("asdf", &obj2);
  EXPECT_FALSE(found);
  EXPECT_EQ("", obj2.name());
}

TEST(SceneTest, FindObject) {
  Object o1;
  o1.set_name("object_1");
  Object o2;
  o2.set_name("object_2");
  HSurface surface;
  surface.set_name("surface");
  surface.AddObject(o1);
  surface.AddObject(o2);
  Scene scene;
  scene.set_primary_surface(surface);

  EXPECT_EQ("surface", scene.primary_surface().name());
  EXPECT_EQ(2, scene.primary_surface().objects().size());

  Object obj;
  bool found = scene.GetObject("object_1", &obj);
  EXPECT_TRUE(found);
  EXPECT_EQ("object_1", obj.name());

  Object obj2;
  found = scene.GetObject("asdf", &obj2);
  EXPECT_FALSE(found);
  EXPECT_EQ("", obj2.name());
}

TEST(SceneTest, StoresPointCloud) {
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  PointXYZRGB p;
  p.x = 1;
  cloud->push_back(p);
  p.x = 2;
  cloud->push_back(p);
  p.x = 3;
  cloud->push_back(p);

  Scene scene;
  scene.set_cloud(cloud);

  PointCloud<PointXYZRGB>::ConstPtr output = scene.cloud();
  EXPECT_EQ(3, output->size());
  EXPECT_EQ(1, output->points[0].x);
  EXPECT_EQ(2, output->points[1].x);
  EXPECT_EQ(3, output->points[2].x);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
