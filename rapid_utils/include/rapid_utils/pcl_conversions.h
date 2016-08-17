#ifndef _RAPID_UTILS_PCL_CONVERSIONS_H_
#define _RAPID_UTILS_PCL_CONVERSIONS_H_

#include "pcl/point_types.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
namespace utils {
void GeometryMsgToPcl(const geometry_msgs::Point& in, pcl::PointXYZ* out);
void GeometryMsgToPcl(const geometry_msgs::Vector3& in, pcl::PointXYZ* out);
void PclToGeometryMsg(const pcl::PointXYZ& in, geometry_msgs::Point* out);
void PclToGeometryMsg(const pcl::PointXYZ& in, geometry_msgs::Vector3* out);
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_PCL_CONVERSIONS_H_
