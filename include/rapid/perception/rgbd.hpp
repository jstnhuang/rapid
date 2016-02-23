#ifndef _RAPID_PERCEPTION_RGBD_H_
#define _RAPID_PERCEPTION_RGBD_H_

#include <iostream>
#include <limits>
#include <vector>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Point.h"
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

// A horizontal plane may cut across the entire scene, so we cluster the plane
// and call the largest cluster the tabletop.
template <typename PointType>
bool FindTabletop(const pcl::PointCloud<PointType>& cloud,
                  double distance_threshold, pcl::PointIndices::Ptr inliers);

template <typename PointType>
void CropWorkspace(const pcl::PointCloud<PointType>& cloud,
                   const visualization_msgs::Marker& ws,
                   pcl::PointCloud<PointType>* cloud_out);

class Object {
 public:
  Object(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();

 private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

// A tabletop is any horizontal surface that objects can rest on. It is not
// necessarily part of a table.
// Like an rviz marker, the position of a tabletop refers to its center, and its
// scale gives the dimensions in the x, y, and z directions.
class Tabletop {
 public:
  Tabletop(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  void AddObject(const Object& object);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();
  std::vector<Object> objects();
  geometry_msgs::Pose pose();

 private:
  geometry_msgs::Pose pose_;
  geometry_msgs::Vector3 scale_;
  std::vector<Object> objects_;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

// A Scene is a semantic representation of a point cloud. Given a point cloud,
// it will extract surfaces and objects.
// This class works best when the given point cloud is cropped to a small area
// of interest, such as the robot's current workspace.
class Scene {
 public:
  Scene(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  void Parse();
  boost::shared_ptr<Tabletop> GetPrimarySurface();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();

 private:
  boost::shared_ptr<Tabletop> primary_surface_;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
};

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
bool FindTabletop(const pcl::PointCloud<PointType>& cloud,
                  double distance_threshold, pcl::PointIndices::Ptr inliers) {
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
  bool found = FindHorizontalPlane(cloud, distance_threshold, plane_inliers);
  if (!found) {
    return found;
  }

  typename pcl::search::KdTree<PointType>::Ptr tree(
      new pcl::search::KdTree<PointType>);
  typename pcl::search::KdTree<PointType>::IndicesConstPtr plane_iptr(
      new std::vector<int>(plane_inliers->indices));
  tree->setInputCloud(cloud.makeShared(), plane_iptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(distance_threshold);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.setIndices(plane_inliers);
  ec.extract(cluster_indices);

  if (cluster_indices.size() == 0) {
    return false;
  }

  size_t max_index = -1;
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    size_t size = cluster_indices.size();
    if (size > max_size) {
      max_size = size;
      max_index = i;
    }
  }

  inliers->indices = cluster_indices[max_index].indices;
  return true;
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
