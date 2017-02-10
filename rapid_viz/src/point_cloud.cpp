#include "rapid_viz/point_cloud.h"

#include <cstdlib>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
namespace viz {
void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->points[i].r = static_cast<int>(round(r * 255));
    cloud->points[i].g = static_cast<int>(round(g * 255));
    cloud->points[i].b = static_cast<int>(round(b * 255));
  }
}

void ColorizeRandom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  double r = static_cast<double>(rand()) / RAND_MAX;
  double g = static_cast<double>(rand()) / RAND_MAX;
  double b = static_cast<double>(rand()) / RAND_MAX;
  Colorize(cloud, r, g, b);
}
}  // namespace viz
}  // namespace rapid
