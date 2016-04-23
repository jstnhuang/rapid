#include "rapid_perception/rgbd.hpp"

#include <stdlib.h>

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "gtest/gtest.h"
#include "pcl/common/angles.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_datatypes.h"

using pcl::PointCloud;
using pcl::PointXYZ;
using pcl::PointXYZRGB;
using pcl::PointIndices;

namespace rapid {
namespace perception {
TEST(RgbdTest, IndicesToCloud) {
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  PointXYZ p;
  p.x = 1;
  cloud->push_back(p);
  p.x = 2;
  cloud->push_back(p);
  p.x = 3;
  cloud->push_back(p);
  PointIndices::Ptr indices(new PointIndices);
  indices->indices.push_back(0);
  indices->indices.push_back(2);

  PointCloud<PointXYZ>::Ptr output = IndicesToCloud<PointXYZ>(cloud, indices);
  EXPECT_EQ(2, output->size());
  EXPECT_EQ(1, output->points[0].x);
  EXPECT_EQ(3, output->points[1].x);
}

TEST(RgbdTest, FindHorizontalPlane) {
  // Simulate a plane with some noise.
  PointCloud<PointXYZ>::Ptr plane(new PointCloud<PointXYZ>);
  for (double x = -0.3; x < 0.3; x += 0.1) {
    for (double y = -0.4; y < 0.4; y += 0.1) {
      PointXYZ p;
      p.x = x;
      p.y = y;
      p.z = 0.7;
      plane->push_back(p);
    }
  }

  srand(0);
  PointCloud<PointXYZ>::Ptr outliers(new PointCloud<PointXYZ>);
  for (double x = -0.1; x < 0.1; x += 0.1) {
    for (double y = -0.1; y < 0.1; y += 0.1) {
      PointXYZ p;
      p.x = x;
      p.y = y;
      p.z = rand() / RAND_MAX;
      outliers->push_back(p);
    }
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  *cloud = *plane + *outliers;
  PointIndices::Ptr inliers(new PointIndices);
  bool success =
      FindHorizontalPlane<PointXYZ>(cloud, 0.1, pcl::deg2rad(5.0), inliers);

  EXPECT_TRUE(success);
  EXPECT_GT(inliers->indices.size(), 0);

  PointCloud<PointXYZ>::Ptr output = IndicesToCloud<PointXYZ>(cloud, inliers);
  for (size_t i = 0; i < output->size(); ++i) {
    const PointXYZ& p = output->points[i];
    EXPECT_FLOAT_EQ(0.7, p.z);
  }
}

TEST(RgbdTest, GetPlanarBoundingBox) {
  // Simulate a box.
  PointCloud<PointXYZRGB>::Ptr box(new PointCloud<PointXYZRGB>);
  for (double x = -0.1; x < 0.11; x += 0.2) {
    for (double y = -0.2; y < 0.22; y += 0.4) {
      for (double z = 0; z < 0.31; z += 0.3) {
        PointXYZRGB p;
        p.x = x;
        p.y = y;
        p.z = z;
        box->push_back(p);
      }
    }
  }

  // Rotate the box by 30 degrees around z axis, translate it.
  tf::Vector3 origin(1, 2, 3);
  tf::Quaternion tf_q(tf::Vector3(0, 0, 1), pcl::deg2rad(30.0));
  tf::Transform transform(tf_q, origin);

  PointCloud<PointXYZRGB>::Ptr transformed_box(new PointCloud<PointXYZRGB>);
  pcl_ros::transformPointCloud(*box, *transformed_box, transform);

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 scale;
  GetPlanarBoundingBox(transformed_box, &pose, &scale);

  EXPECT_EQ(1, pose.position.x);
  EXPECT_EQ(2, pose.position.y);
  EXPECT_FLOAT_EQ(3.15, pose.position.z);

  // Because the y direction is the longest side of this box, we expect its
  // orientation to be in the +y direction.
  // We also expect +z to point up.
  Eigen::Quaternion<double> expected_q;
  expected_q =
      Eigen::AngleAxis<double>(pcl::deg2rad(300.0), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond actual_q;
  actual_q.w() = pose.orientation.w;
  actual_q.x() = pose.orientation.x;
  actual_q.y() = pose.orientation.y;
  actual_q.z() = pose.orientation.z;

  EXPECT_NEAR(expected_q.w(), pose.orientation.w, 0.00001);
  EXPECT_NEAR(expected_q.x(), pose.orientation.x, 0.00001);
  EXPECT_NEAR(expected_q.y(), pose.orientation.y, 0.00001);
  EXPECT_NEAR(expected_q.z(), pose.orientation.z, 0.00001);

  // The x scale is the length of the longest side, which is actually the y
  // direction in this example.
  EXPECT_NEAR(0.4, scale.x, 0.00001);
  EXPECT_NEAR(0.2, scale.y, 0.00001);
  EXPECT_NEAR(0.3, scale.z, 0.00001);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
