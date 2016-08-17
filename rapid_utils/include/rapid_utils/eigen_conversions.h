#ifndef _RAPID_UTILS_EIGEN_CONVERSIONS_H_
#define _RAPID_UTILS_EIGEN_CONVERSIONS_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/Quaternion.h"

namespace rapid {
namespace utils {
void GeometryMsgToEigen(const geometry_msgs::Quaternion& in,
                        Eigen::Quaternionf* out);
void EigenToGeometryMsg(const Eigen::Quaternionf& in,
                        geometry_msgs::Quaternion* out);
}  // namespace utils
}  // namespace rapid

#endif  // _RAPID_UTILS_EIGEN_CONVERSIONS_H_
