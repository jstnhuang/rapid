#include "rapid_utils/eigen_conversions.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/Quaternion.h"

namespace rapid {
namespace utils {
void GeometryMsgToEigen(const geometry_msgs::Quaternion& in,
                        Eigen::Quaternionf* out) {
  out->w() = in.w;
  out->x() = in.x;
  out->y() = in.y;
  out->z() = in.z;
}

void EigenToGeometryMsg(const Eigen::Quaternionf& in,
                        geometry_msgs::Quaternion* out) {
  out->w = in.w();
  out->x = in.x();
  out->y = in.y();
  out->z = in.z();
}
}  // namespace utils
}  // namespace rapid
