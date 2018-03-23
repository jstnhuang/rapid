#include "rapid_collision/obb.h"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "eigen_conversions/eigen_msg.h"
#include "fcl/math/vec_3f.h"

using geometry_msgs::Pose;
using geometry_msgs::Vector3;

namespace rapid {
fcl::OBB BoxToOBB(const Pose& pose, const Vector3& scale) {
  fcl::OBB obb;
  obb.To = fcl::Vec3f(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(pose.orientation, q);
  Eigen::Matrix3d rot = q.toRotationMatrix();
  obb.axis[0] = fcl::Vec3f(rot(0, 0), rot(1, 0), rot(2, 0));
  obb.axis[1] = fcl::Vec3f(rot(0, 1), rot(1, 1), rot(2, 1));
  obb.axis[2] = fcl::Vec3f(rot(0, 2), rot(1, 2), rot(2, 2));

  obb.extent = fcl::Vec3f(scale.x / 2, scale.y / 2, scale.z / 2);
  return obb;
}
}  // namespace rapid
