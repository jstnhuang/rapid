#include "rapid_perception/conversions.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/PointCloud2.h"

namespace rapid {
namespace perception {
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PclFromRos(
    const sensor_msgs::PointCloud2& msg);
sensor_msgs::PointCloud2::Ptr RosFromPcl(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}  // namespace perception
}  // namespace rapid
