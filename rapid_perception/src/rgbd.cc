#include "rapid_perception/rgbd.hpp"

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/pca.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"

#include <iostream>
#include <limits>
#include <set>
#include <vector>

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using pcl::search::KdTree;
using std::cout;
using std::endl;
using std::vector;

namespace rapid {
namespace perception {
Object::Object(const PointCloud<PointXYZRGB>& cloud)
    : cloud_(cloud), pose_(), scale_() {
  geometry_msgs::Pose pose;
  GetPlanarBoundingBox(cloud, &pose, &scale_);
  pose_.header.frame_id = cloud.header.frame_id;
  pose_.pose = pose;
}

geometry_msgs::PoseStamped Object::pose() const { return pose_; }

geometry_msgs::Vector3 Object::scale() const { return scale_; }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Object::GetCloud() {
  return cloud_.makeShared();
}

Tabletop::Tabletop(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    : pose_(), scale_(), objects_(), cloud_(cloud) {}

void Tabletop::AddObject(const Object& obj) { objects_.push_back(obj); }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tabletop::GetCloud() {
  return cloud_.makeShared();
}

geometry_msgs::Pose Tabletop::pose() { return pose_; }

vector<Object> Tabletop::objects() { return objects_; }

Scene::Scene() : cloud_(), primary_surface_() {}

void Scene::set_cloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
  cloud_ = cloud;
}

void Scene::Parse() {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  bool found = FindTabletop(cloud_, 0.01, inliers);
  if (!found) {
    ROS_ERROR("Failed to find tabletop while parsing scene.");
    return;
  }
  PointCloud<PointXYZRGB> table_cloud;
  IndicesToCloud(cloud_, inliers, &table_cloud);
  primary_surface_.reset(new Tabletop(table_cloud));

  // Crop the scene to just the tabletop area.
  // Do a Euclidean clustering of the scene. We assume all points clustered with
  // the table are either part of the table or an object on the table.
  // So, the objects are clustered with the table, but not part of the table
  // itself.
  KdTree<PointXYZRGB>::Ptr tree(new KdTree<PointXYZRGB>);
  tree->setInputCloud(cloud_.makeShared());

  vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(25);
  ec.setMaxClusterSize(100000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_.makeShared());
  ec.extract(cluster_indices);

  // Intersect the cluster indices with the tabletop indices.
  std::set<int> table_indices;  // Indices of table points.
  for (size_t i = 0; i < inliers->indices.size(); ++i) {
    int table_index = inliers->indices[i];
    table_indices.insert(table_index);
  }

  std::set<int> table_cluster_indices;  // The set of clusters that table points
                                        // can be found in.
  int cluster_index = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    for (vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      int point_index = *pit;
      if (table_indices.find(point_index) != table_indices.end()) {
        // This is a tabletop point.
        table_cluster_indices.insert(cluster_index);
        break;
      }
    }
    ++cluster_index;
  }

  PointCloud<PointXYZRGB> objects_cloud;
  cluster_index = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    if (table_cluster_indices.find(cluster_index) ==
        table_cluster_indices.end()) {
      ++cluster_index;
      continue;
    }
    for (vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      int point_index = *pit;
      if (table_indices.find(point_index) == table_indices.end()) {
        // This is an object point, or below the table.
        if (cloud_[point_index].z > primary_surface_->pose().position.z) {
          objects_cloud.push_back(cloud_[point_index]);
        }
      }
    }
    ++cluster_index;
  }

  vector<Object> objects;
  SegmentObjects<PointXYZRGB>(objects_cloud.makeShared(), 0.01, &objects);
  for (vector<Object>::const_iterator it = objects.begin(); it != objects.end();
       ++it) {
    primary_surface_->AddObject(*it);
  }
}

boost::shared_ptr<Tabletop> Scene::GetPrimarySurface() {
  return primary_surface_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Scene::GetCloud() {
  return cloud_.makeShared();
}

void GetPlanarBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // Project points onto XY plane.
  PointCloud<PointXYZRGB>::Ptr projected(new PointCloud<PointXYZRGB>(cloud));
  for (size_t i = 0; i < projected->points.size(); ++i) {
    PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }

  // Compute PCA.
  pcl::PCA<PointXYZRGB> pca(true);
  pca.setInputCloud(projected);

  // Get eigenvectors.
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  // Because we projected points on the XY plane, we add in the Z vector as the
  // 3rd eigenvector.
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
  Eigen::Quaternionf q1(eigenvectors);

  // Find min/max x and y, based on the points in eigenspace.
  PointCloud<PointXYZRGB>::Ptr eigen_projected(
      new PointCloud<PointXYZRGB>(cloud));
  pca.project(cloud, *eigen_projected);

  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;

  // The points in eigenspace all have z values of 0. Get min/max z from the
  // original point cloud data.
  pcl::PointXYZRGB cloud_min;
  pcl::PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  double z_length = cloud_max.z - cloud_min.z;

  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = 0;
  PointXYZRGB center;
  pca.reconstruct(eigen_center, center);
  center.z = z_length / 2 + cloud_min.z;

  // Output midpoint.
  midpoint->position.x = center.x;
  midpoint->position.y = center.y;
  midpoint->position.z = center.z;
  midpoint->orientation.w = q1.w();
  midpoint->orientation.x = q1.x();
  midpoint->orientation.y = q1.y();
  midpoint->orientation.z = q1.z();

  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}

}  // namespace perception
}  // namespace rapid
