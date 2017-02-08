#ifndef _RAPID_PERCEPTION_CONVERSIONS_H_
#define _RAPID_PERCEPTION_CONVERSIONS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"

using sensor_msgs::PointCloud2;
using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace rapid {
namespace perception {
PointCloud<PointXYZRGB>::Ptr PclFromRos(const PointCloud2& msg) {
  PointCloud<PointXYZRGB>::Ptr out(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(msg, *out);
  return out;
}

PointCloud2::Ptr RosFromPcl(PointCloud<PointXYZRGB>::Ptr cloud) {
  PointCloud2::Ptr out(new PointCloud2);
  pcl::toROSMsg(*cloud, *out);
  return out;
}
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_CONVERSIONS_H_
