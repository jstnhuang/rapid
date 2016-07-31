#ifndef _RAPID_PERCEPTION_ICP_FITNESS_FUNCTIONS_H_
#define _RAPID_PERCEPTION_ICP_FITNESS_FUNCTIONS_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_msgs/Roi3D.h"

namespace rapid {
namespace perception {
// Computes an alternative ICP fitness score between the scene and an object
// with a bounding box around it. This metric measures the distance between
// the
// scene points inside the bounding box and the nearest point on the object.
// This allows us to assert that there should be free space in the given
// bounding box.
//
// We assume that the scene, object, and ROI are all given in the same
// reference
// frame.
//
// We do not take into account the difference between free space that is
// visibly
// free or free space with unknown occupancy (in practice it may not make a
// difference?).
//
// Args:
//  scene: The scene the object is in.
//  object: The object to compute the score of.
//  roi: The region of interest to get scene points from.
double ComputeIcpFitness(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scene,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object,
                         const rapid_msgs::Roi3D& roi);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_ICP_FITNESS_FUNCTIONS_H_
