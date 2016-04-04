#include "rapid_perception/scene.h"

#include <iostream>
#include <sstream>

#include "rapid_perception/rgbd.hpp"
#include "rapid_viz/markers.h"
#include "visualization_msgs/Marker.h"

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using pcl::search::KdTree;
using std::cout;
using std::endl;
using std::vector;

namespace rapid {
namespace perception {
Object::Object(Scene* scene, const PointIndices::Ptr& indices)
    : scene_(scene),
      indices_(indices),
      pose_(),
      scale_(),
      name_(""),
      obj_marker_(rapid::viz::Marker::Box(scene->viz_pub(), pose_, scale_)),
      text_marker_(
          rapid::viz::Marker::Text(scene->viz_pub(), pose_, name_, 0.03)) {
  geometry_msgs::Pose pose;
  PointCloud<PointXYZRGB>::Ptr cloud = GetCloud();
  GetPlanarBoundingBox(*cloud, &pose, &scale_);
  pose_.header.frame_id = cloud->header.frame_id;
  pose_.pose = pose;
}

void Object::Visualize() {
  obj_marker_ = rapid::viz::Marker::Box(scene_->viz_pub(), pose_, scale_);
  obj_marker_.SetNamespace(name_);
  obj_marker_.SetColor(0, 0, 1, 0.9);
  obj_marker_.Publish();

  text_marker_ =
      rapid::viz::Marker::Text(scene_->viz_pub(), pose_, name_, 0.03);
  text_marker_.SetNamespace(name_);
  text_marker_.SetColor(1, 1, 1, 1);
  text_marker_.Publish();
}

PointCloud<PointXYZRGB>::Ptr Object::GetCloud() const {
  return IndicesToCloud(scene_->GetCloud(), indices_);
}

void SegmentObjects(Scene* scene, PointIndices::Ptr indices,
                    double distance_threshold, vector<Object>* objects) {
  KdTree<PointXYZRGB>::Ptr tree(new KdTree<PointXYZRGB>);
  boost::shared_ptr<vector<int> > indices_v(new vector<int>(indices->indices));
  tree->setInputCloud(scene->GetCloud(), indices_v);

  vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(distance_threshold);
  ec.setSearchMethod(tree);
  ec.setInputCloud(scene->GetCloud());
  ec.setIndices(indices);
  ec.setMinClusterSize(20);
  ec.extract(cluster_indices);

  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    boost::shared_ptr<PointIndices> cluster_indices_p(new PointIndices);
    *cluster_indices_p = *it;
    Object obj(scene, cluster_indices_p);
    objects->push_back(obj);
  }
}

Tabletop::Tabletop(Scene* scene, const PointIndices::Ptr& indices)
    : scene_(scene),
      indices_(indices),
      pose_(),
      scale_(),
      objects_(),
      name_("table"),
      marker_(rapid::viz::Marker::Box(scene_->viz_pub(), pose_, scale_)) {
  geometry_msgs::Pose pose;
  PointCloud<PointXYZRGB>::Ptr cloud = GetCloud();
  GetPlanarBoundingBox(*cloud, &pose, &scale_);
  pose_.header.frame_id = cloud->header.frame_id;
  pose_.pose = pose;
}

void Tabletop::AddObject(const Object& obj) { objects_.push_back(obj); }

PointCloud<PointXYZRGB>::Ptr Tabletop::GetCloud() {
  return IndicesToCloud(scene_->GetCloud(), indices_);
}

void Tabletop::Visualize() {
  marker_ = rapid::viz::Marker::Box(scene_->viz_pub(), pose_, scale_);
  marker_.SetNamespace(name_);
  marker_.SetColor(0, 1, 0, 0.5);
  marker_.Publish();

  for (size_t i = 0; i < objects_.size(); ++i) {
    objects_[i].Visualize();
  }
}

bool FindTabletop(const PointCloud<PointXYZRGB>& cloud,
                  double distance_threshold, PointIndices::Ptr inliers) {
  PointIndices::Ptr plane_inliers(new PointIndices);
  bool found = FindHorizontalPlane(cloud, distance_threshold, plane_inliers);
  if (!found) {
    return found;
  }

  KdTree<PointXYZRGB>::Ptr tree(new KdTree<PointXYZRGB>);
  KdTree<PointXYZRGB>::IndicesConstPtr plane_iptr(
      new vector<int>(plane_inliers->indices));
  tree->setInputCloud(cloud.makeShared(), plane_iptr);

  vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
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

Scene::Scene()
    : cloud_(),
      primary_surface_(),
      nh_(),
      viz_pub_(nh_.advertise<visualization_msgs::Marker>("visualization_marker",
                                                         10)) {}

void Scene::set_cloud(const PointCloud<PointXYZRGB>& cloud) { cloud_ = cloud; }

void Scene::Parse() {
  PointIndices::Ptr inliers(new PointIndices);
  bool found = FindTabletop(cloud_, 0.01, inliers);
  if (!found) {
    ROS_ERROR("Failed to find tabletop while parsing scene.");
    return;
  }
  primary_surface_.reset(new Tabletop(this, inliers));

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

  PointIndices::Ptr objects_indices(new PointIndices);
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
        if (cloud_[point_index].z > primary_surface_->pose().pose.position.z) {
          objects_indices->indices.push_back(point_index);
        }
      }
    }
    ++cluster_index;
  }

  vector<Object> objects;
  SegmentObjects(this, objects_indices, 0.01, &objects);
  for (size_t i = 0; i < objects.size(); ++i) {
    std::stringstream name;
    name << "object_" << i;
    objects[i].set_name(name.str());
    primary_surface_->AddObject(objects[i]);
  }
}

boost::shared_ptr<Tabletop> Scene::GetPrimarySurface() {
  return primary_surface_;
}

PointCloud<PointXYZRGB>::Ptr Scene::GetCloud() { return cloud_.makeShared(); }

void Scene::Visualize() { primary_surface_->Visualize(); }
}  // namespace rapid
}  // namespace perception
