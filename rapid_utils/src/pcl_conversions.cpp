#include "rapid_utils/pcl_conversions.h"

#include "pcl/point_types.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
namespace utils {
void GeometryMsgToPcl(const geometry_msgs::Point& in, pcl::PointXYZ* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

void GeometryMsgToPcl(const geometry_msgs::Vector3& in, pcl::PointXYZ* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

void PclToGeometryMsg(const pcl::PointXYZ& in, geometry_msgs::Point* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

void PclToGeometryMsg(const pcl::PointXYZ& in, geometry_msgs::Vector3* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}
}  // namespace utils
}  // namespace rapid
