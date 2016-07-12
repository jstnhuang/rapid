// Contains general purpose code for working with point clouds.
// In the future, we may want to split this into more meaningfully named files.
#ifndef _RAPID_PERCEPTION_RGBD_H_
#define _RAPID_PERCEPTION_RGBD_H_

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

namespace rapid {
namespace perception {
// Given a 3x3 matrix and a 3x3 vector, find which column of the matrix is
// closest (in cosine similarity) or most diametrically opposite to the vector.
// E.g., a cosine similarity of -1 is considered closer than 0.5.
void ClosestAxis(const Eigen::Matrix3d& mat, const Eigen::Vector3d& vec,
                 int* index, double* cosine_sim);

// Same as FindHorizontalPlane below, except using the entire cloud as given.
bool FindHorizontalPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    double distance_threshold, double eps_angle,
    pcl::PointIndices::Ptr inliers);

// Given a point cloud, finds the dominant horizontal plane (as determined by
// RANSAC). Assumes that the point cloud is transformed such that the z axis
// corresponds to "up."
//
// Implementation notes: after finding a plane, there can sometimes be points in
// the background which are part of the plane. These should be filtered using
// clustering.
//
// Returns true if a plane was found, false otherwise.
bool FindHorizontalPlane(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices, double distance_threshold,
    double eps_angle, pcl::PointIndices::Ptr inliers);

// Computes the bounding box for the given point cloud using PCA.
// The midpoint is given in the same frame as input cloud. The +z direction
// points in the direction closest to +z in the input cloud, and the same with
// +x.
//
// Args:
//   cloud: The input cloud. Assumed to not have any NaN points.
//   midpoint: The geometric center of the cloud, i.e., the midpoint between the
//     minimum and maximum points in each of the x, y, and z directions. The
//     orientation is such that the x direction points along the principal
//     component and the z direction points up.
//   dimensions: A vector containing the length of the cloud in the x, y, and z
//   directions.
void GetBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                    const pcl::PointIndices::ConstPtr& indices,
                    geometry_msgs::PoseStamped* midpoint,
                    geometry_msgs::Vector3* dimensions);

// Computes the bounding box for the given point cloud on the XY plane.
//
// Args:
//   cloud: The input cloud. Assumed to not have any NaN points.
//   midpoint: The geometric center of the cloud, i.e., the midpoint between the
//     minimum and maximum points in each of the x, y, and z directions. The
//     orientation is such that the x direction points along the principal
//     component and the z direction points up.
//   dimensions: A vector containing the length of the cloud in the x, y, and z
//   directions.
void GetPlanarBoundingBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices, geometry_msgs::Pose* midpoint,
    geometry_msgs::Vector3* dimensions);

// Like GetPlanarBoundingBox, but with the entire cloud as given.
void GetPlanarBoundingBox(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    geometry_msgs::Pose* midpoint, geometry_msgs::Vector3* dimensions);

// Given PointIndices to a PointCloud, returns the point cloud referred to by
// the PointIndices.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr IndicesToCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices);

}  // namespace perception
}  // namespace rapid
#endif  // _RAPID_PERCEPTION_RGBD_H_
