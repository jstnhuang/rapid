#ifndef _RAPID_PERCEPTION_RGBD_H_
#define _RAPID_PERCEPTION_RGBD_H_

#include <iostream>
#include <vector>

#include "geometry_msgs/Point.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

namespace rapid {
namespace perception {
// Declarations ----------------------------------------------------------
// Given PointIndices to a PointCloud, returns the point cloud referred to by
// the PointIndices.
template <typename PointType>
void IndicesToCloud(const pcl::PointCloud<PointType>& cloud,
                    const pcl::PointIndices::Ptr& indices,
                    pcl::PointCloud<PointType>* result);

// Removes the indices from the given cloud. The cloud is modified directly.
template <typename PointType>
void IndicesToNegativeCloud(const pcl::PointCloud<PointType>& cloud,
                            const pcl::PointIndices::Ptr& indices,
                            pcl::PointCloud<PointType>* result);

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
bool FindHorizontalPlane(const pcl::PointCloud<PointType>& cloud,
                         double distance_threshold,
                         pcl::PointIndices::Ptr inliers);

// Given a point cloud, find the horizontal planes.
template <typename PointType>
void FindHorizontalPlanes(const pcl::PointCloud<PointType>& cloud,
                          double distance_threshold,
                          std::vector<pcl::PointCloud<PointType> >* planes);

template <typename PointType>
void CropWorkspace(const pcl::PointCloud<PointType>& cloud,
                   const visualization_msgs::Marker& ws,
                   pcl::PointCloud<PointType>* cloud_out);

// Given a point cloud and a horizontal plane, segments the objects resting on
// the plane.
// void FindPlaneObjects();

// Definitions --------------------------------------------------------------
template <typename PointType>
void IndicesToCloud(const pcl::PointCloud<PointType>& cloud,
                    const pcl::PointIndices::Ptr& indices,
                    pcl::PointCloud<PointType>* result) {
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud.makeShared());
  pcl::PointIndices::Ptr indices_p(indices);
  extract.setIndices(indices_p);
  extract.setNegative(false);
  extract.filter(*result);
}

template <typename PointType>
void IndicesToNegativeCloud(pcl::PointCloud<PointType>& cloud,
                            const pcl::PointIndices::Ptr& indices,
                            pcl::PointCloud<PointType>* result) {
  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloud.makeShared());
  pcl::PointIndices::Ptr indices_p(indices);
  extract.setIndices(indices_p);
  extract.setNegative(true);
  extract.filter(*result);
}

template <typename PointType>
bool FindHorizontalPlane(const pcl::PointCloud<PointType>& cloud,
                         double distance_threshold,
                         pcl::PointIndices::Ptr inliers) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(pcl::deg2rad(5.0));

  seg.setInputCloud(cloud.makeShared());
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    return false;
  }
  return true;
}

template <typename PointType>
void FindHorizontalPlanes(const pcl::PointCloud<PointType>& cloud,
                          double distance_threshold,
                          std::vector<pcl::PointCloud<PointType> >* planes) {
  typename pcl::PointCloud<PointType>::Ptr working(
      new pcl::PointCloud<PointType>(cloud));

  bool found = true;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  while (found) {
    found = FindHorizontalPlane(*working, distance_threshold, inliers);
    if (!found) {
      break;
    }

    pcl::PointCloud<PointType> surface;
    IndicesToCloud(*working, inliers, &surface);
    planes->push_back(surface);
    pcl::PointCloud<PointType> filtered;
    IndicesToNegativeCloud(*working, inliers, &filtered);
    *working = filtered;
  }
}

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
}  // perception
}  // rapid
#endif  // _RAPID_PERCEPTION_RGBD_H_
