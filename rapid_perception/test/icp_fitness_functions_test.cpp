#include "rapid_perception/icp_fitness_functions.h"

#include "gtest/gtest.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_msgs/Roi3D.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace rapid {
namespace perception {
TEST(IcpFitnessFunctionTest, Identical) {
  PointCloudC::Ptr object(new PointCloudC);
  for (double x = 0; x < 0.06; x += 0.05) {
    for (double y = 0; y < 0.11; y += 0.1) {
      for (double z = 0; z < 0.06; z += 0.05) {
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        object->push_back(pt);
      }
    }
  }
  rapid_msgs::Roi3D roi;
  roi.transform.translation.x = 0.025;
  roi.transform.translation.y = 0.05;
  roi.transform.translation.z = 0.025;
  roi.transform.rotation.w = 1;
  roi.dimensions.x = 0.05;
  roi.dimensions.y = 0.1;
  roi.dimensions.z = 0.05;
  double fitness = ComputeIcpFitness(object, object, roi);
  EXPECT_EQ(0, fitness);
}

TEST(IcpFitnessFunctionTest, ShiftedRoiCutoff) {
  PointCloudC::Ptr object(new PointCloudC);
  for (double x = 0; x < 0.06; x += 0.05) {
    for (double y = 0; y < 0.11; y += 0.1) {
      for (double z = 0; z < 0.06; z += 0.05) {
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        object->push_back(pt);
      }
    }
  }
  // ROI will cut off the back points of the scene
  rapid_msgs::Roi3D roi;
  roi.transform.translation.x = 0.025;
  roi.transform.translation.y = 0.05;
  roi.transform.translation.z = 0.025;
  roi.transform.rotation.w = 1;
  roi.dimensions.x = 0.05;
  roi.dimensions.y = 0.1;
  roi.dimensions.z = 0.05;

  // Scene is same as object, but shifted by 1 cm in the +x direction
  PointCloudC::Ptr scene(new PointCloudC);
  for (double x = 0.01; x < 0.07; x += 0.05) {
    for (double y = 0; y < 0.11; y += 0.1) {
      for (double z = 0; z < 0.06; z += 0.05) {
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        scene->push_back(pt);
      }
    }
  }

  // All four of the scene points will have an error of 0.01. However, only half
  // of the object will have been visited. For the four unvisited points, the
  // error is 0.04. So the average is 0.025.
  double fitness = ComputeIcpFitness(scene, object, roi);
  EXPECT_FLOAT_EQ(0.025, fitness);
}

TEST(IcpFitnessFunctionTest, Shifted) {
  PointCloudC::Ptr object(new PointCloudC);
  for (double x = 0; x < 0.06; x += 0.05) {
    for (double y = 0; y < 0.11; y += 0.1) {
      for (double z = 0; z < 0.06; z += 0.05) {
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        object->push_back(pt);
      }
    }
  }
  rapid_msgs::Roi3D roi;
  roi.transform.translation.x = 0.025;
  roi.transform.translation.y = 0.05;
  roi.transform.translation.z = 0.025;
  roi.transform.rotation.w = 1;
  roi.dimensions.x = 0.07;
  roi.dimensions.y = 0.1;
  roi.dimensions.z = 0.05;

  // Scene is same as object, but shifted by 1 cm in the +x direction
  PointCloudC::Ptr scene(new PointCloudC);
  for (double x = 0.01; x < 0.07; x += 0.05) {
    for (double y = 0; y < 0.11; y += 0.1) {
      for (double z = 0; z < 0.06; z += 0.05) {
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        scene->push_back(pt);
      }
    }
  }

  double fitness = ComputeIcpFitness(scene, object, roi);
  EXPECT_FLOAT_EQ(0.01, fitness);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
