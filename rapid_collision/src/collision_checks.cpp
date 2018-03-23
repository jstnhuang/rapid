#include "rapid_collision/collision_checks.h"

#include "fcl/BV/OBB.h"
#include "rapid_collision/obb.h"

namespace rapid {
bool AreObbsInCollision(const geometry_msgs::Pose& pose1,
                        const geometry_msgs::Vector3& scale1,
                        const geometry_msgs::Pose& pose2,
                        const geometry_msgs::Vector3& scale2) {
  fcl::OBB obb1 = BoxToOBB(pose1, scale1);
  fcl::OBB obb2 = BoxToOBB(pose2, scale2);
  return obb1.overlap(obb2);
}
}  // namespace rapid
