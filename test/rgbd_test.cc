#include "rapid/perception/rgbd.hpp"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"

namespace rapid {
namespace perception {
class TableTest : public ::testing::Test {
 public:
  TableTest()
      : data_path_(ros::package::getPath("rapid") + "/test_data/surfaces.bag"),
        cloud_() {}

  void SetUp() {
    rosbag::Bag bag;
    bag.open(data_path_, rosbag::bagmode::Read);
    std::vector<std::string> types;
    types.push_back(std::string("sensor_msgs/PointCloud2"));

    rosbag::View view(bag, rosbag::TypeQuery(types));
    rosbag::View::iterator it = view.begin();
    for (; it != view.end(); ++it) {
      const rosbag::MessageInstance& mi = *it;
      sensor_msgs::PointCloud2::ConstPtr msg =
          mi.instantiate<sensor_msgs::PointCloud2>();
      pcl::fromROSMsg(*msg, cloud_);
    }
  }

  void Visualize(const std::string& name,
                 const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
    pcl::visualization::CloudViewer viewer(name);
    viewer.showCloud(cloud.makeShared());
    while (!viewer.wasStopped()) {
    }
  }

 protected:
  std::string data_path_;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

TEST_F(TableTest, FindHorizontalPlane) {
  Visualize("Input", cloud_);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  bool found = FindHorizontalPlane(cloud_, 0.01, inliers);
  pcl::PointCloud<pcl::PointXYZRGB> plane;
  IndicesToCloud(cloud_, inliers, &plane);
  Visualize("Detected plane", plane);
  EXPECT_EQ(true, found);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
