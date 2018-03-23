#ifndef _RAPID_COLLISION_OBB_H_
#define _RAPID_COLLISION_OBB_H_

#include "fcl/BV/OBB.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
// Returns an fcl::OBB from a Pose and Vector3 message.
//
// NOTE: The OBB documentation says that the X axis of the rotation matrix
// "is assumed" to point in the direction of the longest dimension (and Z points
// towards the shortest). This function does not do so, but collision checking
// still seems to work.
fcl::OBB BoxToOBB(const geometry_msgs::Pose& pose,
                  const geometry_msgs::Vector3& scale);
}  // namespace rapid

#endif  // _RAPID_COLLISION_OBB_H_
