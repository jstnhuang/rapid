// Contains general purpose code for working with point clouds.
// In the future, we may want to split this into more meaningfully named files.
#ifndef _RAPID_PERCEPTION_RGBD_H_
#define _RAPID_PERCEPTION_RGBD_H_

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
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
// Declarations ----------------------------------------------------------
// Given PointIndices to a PointCloud, returns the point cloud referred to by
// the PointIndices.
template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr IndicesToCloud(
    const typename pcl::PointCloud<PointType>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices);

// Same as FindHorizontalPlane below, except using the entire cloud as given.
template <typename PointType>
bool FindHorizontalPlane(const typename pcl::PointCloud<PointType>::Ptr& cloud,
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
template <typename PointType>
bool FindHorizontalPlane(const typename pcl::PointCloud<PointType>::Ptr& cloud,
                         const pcl::PointIndices::ConstPtr& indices,
                         double distance_threshold, double eps_angle,
                         pcl::PointIndices::Ptr inliers);

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

// Definitions --------------------------------------------------------------
template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr IndicesToCloud(
    const typename pcl::PointCloud<PointType>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices) {
  typename pcl::PointCloud<PointType>::Ptr result(
      new pcl::PointCloud<PointType>());
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*result);
  return result;
}

template <typename PointType>
bool FindHorizontalPlane(const typename pcl::PointCloud<PointType>::Ptr& cloud,
                         double distance_threshold, double eps_angle,
                         pcl::PointIndices::Ptr inliers) {
  pcl::PointIndices::Ptr pi;
  FindHorizontalPlane<PointType>(cloud, pi, distance_threshold, eps_angle,
                                 inliers);
}

template <typename PointType>
bool FindHorizontalPlane(const typename pcl::PointCloud<PointType>::Ptr& cloud,
                         const pcl::PointIndices::ConstPtr& indices,
                         double distance_threshold, double eps_angle,
                         pcl::PointIndices::Ptr inliers) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(eps_angle);

  seg.setInputCloud(cloud);
  if (indices) {
    seg.setIndices(indices);
  }
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    return false;
  }
  return true;
}
}  // namespace perception
}  // namespace rapid
#endif  // _RAPID_PERCEPTION_RGBD_H_
