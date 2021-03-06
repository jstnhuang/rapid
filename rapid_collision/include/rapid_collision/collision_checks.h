#ifndef _RAPID_COLLISION_COLLISION_CHECKS_H_
#define _RAPID_COLLISION_COLLISION_CHECKS_H_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace rapid {
/// \brief Returns true if the given oriented bounding boxes are colliding.
///
/// The poses must be in the same frame and the scales must be in the same
/// units.
///
/// \param[in] pose1: The pose of the first box.
/// \param[in] scale1: The x/y/z dimensions of the first box.
/// \param[in] pose2: The pose of the second box.
/// \param[in] scale2: The x/y/z dimensions of the second box.
//
/// \returns true if the given oriented bounding boxes are colliding, false
///   otherwise.
bool AreObbsInCollision(const geometry_msgs::Pose& pose1,
                        const geometry_msgs::Vector3& scale1,
                        const geometry_msgs::Pose& pose2,
                        const geometry_msgs::Vector3& scale2);
}  // namespace rapid

#endif  // _RAPID_COLLISION_COLLISION_CHECKS_H_
