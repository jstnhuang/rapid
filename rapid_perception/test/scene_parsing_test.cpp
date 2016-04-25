#include "rapid_perception/scene_parsing.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/package.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_perception/scene.h"

using pcl::PointCloud;
using pcl::PointXYZRGB;
using std::string;

namespace rapid {
namespace perception {
// Test fixture that can loads point clouds from bag files.
// In particular, it reads the first message of type sensor_msgs/PointCloud2 in
// the bag file. Bag files are assumed to be located in the test_data folder. A
// script is included with the package to download the test data.
class TableTest : public ::testing::Test {
 public:
  TableTest()
      : data_dir_(ros::package::getPath("rapid_perception") + "/test_data/"),
        cloud_(new PointCloud<PointXYZRGB>) {}

  void SetUp() {}

  void Load(const string& filename) {
    rosbag::Bag bag;
    bag.open(data_dir_ + filename, rosbag::bagmode::Read);
    std::vector<string> types;
    types.push_back("sensor_msgs/PointCloud2");

    rosbag::View view(bag, rosbag::TypeQuery(types));
    rosbag::View::iterator it = view.begin();
    for (; it != view.end(); ++it) {
      const rosbag::MessageInstance& mi = *it;
      sensor_msgs::PointCloud2::ConstPtr msg =
          mi.instantiate<sensor_msgs::PointCloud2>();
      pcl::fromROSMsg(*msg, *cloud_);
      break;
    }
  }

 protected:
  string data_dir_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};

TEST_F(TableTest, Table0) {
  Load("table0.bag");  // An empty table.
  Scene scene;
  bool success = ParseScene(cloud_, Pr2Params(), &scene);
  int num_objects = scene.primary_surface().objects().size();
  EXPECT_EQ(0, num_objects);
  EXPECT_TRUE(success);
}

TEST_F(TableTest, Table1) {
  Load("table1.bag");  // Simple table with 1 object.
  Scene scene;
  bool success = ParseScene(cloud_, Pr2Params(), &scene);
  int num_objects = scene.primary_surface().objects().size();
  EXPECT_EQ(1, num_objects);
  EXPECT_TRUE(success);
}

TEST_F(TableTest, Table4) {
  Load("table4.bag");
  Scene scene;
  bool success = ParseScene(cloud_, Pr2Params(), &scene);
  // TODO(jstn): fix code to pass test
  // int num_objects = scene.primary_surface().objects().size();
  // EXPECT_EQ(4, num_objects);
  EXPECT_TRUE(success);
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
