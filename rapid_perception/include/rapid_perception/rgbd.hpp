#ifndef _RAPID_PERCEPTION_RGBD_H_
#define _RAPID_PERCEPTION_RGBD_H_

#include <iostream>
#include <limits>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "Eigen/Dense"

namespace rapid {
namespace perception {
// Declarations ----------------------------------------------------------
// Given PointIndices to a PointCloud, returns the point cloud referred to by
// the PointIndices.
template <typename PointType>
typename pcl::PointCloud<PointType>::Ptr IndicesToCloud(
    const typename pcl::PointCloud<PointType>::ConstPtr& cloud,
    const pcl::PointIndices::ConstPtr& indices);

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

template <typename PointType>
void CropWorkspace(const pcl::PointCloud<PointType>& cloud,
                   const visualization_msgs::Marker& ws,
                   pcl::PointCloud<PointType>* cloud_out);

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
  pcl::PointIndices::Ptr pi(new pcl::PointIndices);
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

// template <typename PointType>
// void FindHorizontalPlanes(const pcl::PointCloud<PointType>& cloud,
//                          double distance_threshold,
//                          std::vector<pcl::PointCloud<PointType> >* planes) {
//  typename pcl::PointCloud<PointType>::Ptr working(
//      new pcl::PointCloud<PointType>(cloud));
//
//  bool found = true;
//  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//  while (found) {
//    found = FindHorizontalPlane(*working, distance_threshold, inliers);
//    if (!found) {
//      break;
//    }
//
//    pcl::PointCloud<PointType> surface;
//    IndicesToCloud(*working, inliers, &surface);
//    planes->push_back(surface);
//    pcl::PointCloud<PointType> filtered;
//    IndicesToNegativeCloud(*working, inliers, &filtered);
//    *working = filtered;
//  }
//}

template <typename PointType>
void CropWorkspace(const pcl::PointCloud<PointType>& cloud,
                   const visualization_msgs::Marker& ws,
                   pcl::PointCloud<PointType>* cloud_out) {
  Eigen::Vector4f min;
  min[0] = ws.pose.position.x - ws.scale.x / 2;
  min[1] = ws.pose.position.y - ws.scale.y / 2;
  min[2] = ws.pose.position.z - ws.scale.z / 2;
  Eigen::Vector4f max;
  max[0] = ws.pose.position.x + ws.scale.x / 2;
  max[1] = ws.pose.position.y + ws.scale.y / 2;
  max[2] = ws.pose.position.z + ws.scale.z / 2;
  pcl::CropBox<PointType> filter;
  filter.setInputCloud(cloud.makeShared());
  filter.setMin(min);
  filter.setMax(max);
  filter.filter(*cloud_out);
}

}  // namespace perception
}  // namespace rapid
#endif  // _RAPID_PERCEPTION_RGBD_H_
