#ifndef _RAPID_UTILS_PCL_TYPEDEFS_H_
#define _RAPID_UTILS_PCL_TYPEDEFS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

// Typedefs for PCL types.
// Do not include in a header file!

namespace rapid {
typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudP;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
}  // namespace rapid

#endif  // _RAPID_PCL_TYPEDEFS_H_
