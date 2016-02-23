#include "rapid/perception/rgbd.hpp"

#include "boost/shared_ptr.hpp"
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
Object::Object(const PointCloud<PointXYZRGB>& cloud) : cloud_(cloud) {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Object::GetCloud() {
  return cloud_.makeShared();
}

Tabletop::Tabletop(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    : cloud_(cloud) {}

void Tabletop::AddObject(const Object& obj) { objects_.push_back(obj); }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Tabletop::GetCloud() {
  return cloud_.makeShared();
}

vector<Object> Tabletop::GetObjects() { return objects_; }

Scene::Scene(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) : cloud_(cloud) {}

void Scene::Parse() {
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  bool found = FindHorizontalPlane(cloud_, 0.01, inliers);
  if (!found) {
    return;
  }

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

  cout << "Found " << cluster_indices.size() << " clusters." << endl;

  // Intersect the cluster indices with the tabletop indices.
  std::set<int> table_indices;  // Indices of table points.
  for (size_t i = 0; i < inliers->indices.size(); ++i) {
    int table_index = inliers->indices[i];
    table_indices.insert(table_index);
  }

  cout << table_indices.size() << " points on the table." << endl;

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

  cout << "Table points were found in clusters: ";
  for (std::set<int>::const_iterator it = table_cluster_indices.begin();
       it != table_cluster_indices.end(); ++it) {
    cout << *it << " ";
  }
  cout << endl;

  PointCloud<PointXYZRGB> table_cloud;
  PointCloud<PointXYZRGB> objects_cloud;
  cluster_index = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    if (table_cluster_indices.find(cluster_index) ==
        table_cluster_indices.end()) {
      ++cluster_index;
      continue;
    }
    cout << "Looking at table/object cluster: " << cluster_index
         << ", size=" << it->indices.size() << endl;
    for (vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      int point_index = *pit;
      if (table_indices.find(point_index) == table_indices.end()) {
        // This is an object point.
        objects_cloud.push_back(cloud_[point_index]);
      } else {
        // This is a table point.
        table_cloud.push_back(cloud_[point_index]);
      }
    }
    ++cluster_index;
  }

  cout << "Extracted " << table_cloud.size() << " table points." << endl;
  cout << "Extracted " << objects_cloud.size() << " object points." << endl;

  primary_surface_.reset(new Tabletop(table_cloud));
  Object obj(objects_cloud);
  primary_surface_->AddObject(obj);
}

boost::shared_ptr<Tabletop> Scene::GetPrimarySurface() {
  return primary_surface_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Scene::GetCloud() {
  return cloud_.makeShared();
}

}  // namespace perception
}  // namespace rapid
