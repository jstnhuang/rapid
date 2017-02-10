#ifndef _RAPID_VIZ_POINT_CLOUD_H_
#define _RAPID_VIZ_POINT_CLOUD_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
namespace viz {
// Set the color of a point cloud for visualization.
void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b);
// Set the color of a point cloud for visualization randomly.
void ColorizeRandom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_POINT_CLOUD_H_
