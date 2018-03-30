#include "rapid_utils/vector3.hpp"

#include <gtest/gtest.h>

#include "Eigen/Dense"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_types.h"

namespace rapid {
TEST(Vector3Test, TestEigenVector3f) {
  Eigen::Vector3f input;
  input << 1.5, 2, 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestEigenVector3d) {
  Eigen::Vector3d input;
  input << 1.5, 2, 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestGeometryMsgsPoint) {
  geometry_msgs::Point input;
  input.x = 1.5;
  input.y = 2;
  input.z = 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestGeometryMsgsVector3) {
  geometry_msgs::Vector3 input;
  input.x = 1.5;
  input.y = 2;
  input.z = 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestPclPointXyz) {
  pcl::PointXYZ input;
  input.x = 1.5;
  input.y = 2;
  input.z = 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestPclPointXyzRgb) {
  pcl::PointXYZRGB input;
  input.x = 1.5;
  input.y = 2;
  input.z = 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

TEST(Vector3Test, TestPclNormal) {
  pcl::Normal input;
  input.normal_x = 1.5;
  input.normal_y = 2;
  input.normal_z = 3;
  Eigen::Vector3d output = AsVector3d(input);
  EXPECT_FLOAT_EQ(1.5, output.x());
  EXPECT_FLOAT_EQ(2, output.y());
  EXPECT_FLOAT_EQ(3, output.z());
}

struct UnsupportedType {
  float x;
  float y;
  float z;
};
TEST(Vector3Test, TestUnsupportedType) {
  UnsupportedType unsupported;
  unsupported.x = 0;
  unsupported.y = 0;
  unsupported.z = 0;
  EXPECT_THROW(AsVector3d(unsupported), std::invalid_argument);
}
}  // namespace rapid

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
