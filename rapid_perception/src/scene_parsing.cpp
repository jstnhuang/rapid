#include "rapid_perception/scene_parsing.h"

#include <sstream>
#include <vector>

#include "Eigen/Dense"
#include "pcl/PointIndices.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"

#include "rapid_perception/rgbd.hpp"
#include "rapid_perception/scene.h"
#include "rapid_utils/math.h"

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using std::vector;

namespace rapid {
namespace perception {
ParseParams Pr2Params() {
  ParseParams p;
  p.scene.min_x = 0.2;
  p.scene.max_x = 1.2;
  p.scene.min_y = -1;
  p.scene.max_y = 1;
  p.scene.min_z = 0.3;
  p.scene.max_z = 1.7;
  p.hsurface.distance_threshold = 0;
  p.hsurface.eps_angle = rapid::utils::DegreesToRadians(5);
  p.objects.cluster_tolerance = 0.01;
  p.objects.min_cluster_size = 20;
  return p;
}

bool ParseScene(const PointCloud<PointXYZRGB>::Ptr& cloud,
                const ParseParams& params, Scene* scene) {
  // This function just crops the scene to a box given by the params.
  scene->set_cloud(cloud);
  Eigen::Vector4f min;
  min << params.scene.min_x, params.scene.min_y, params.scene.min_z, 1;
  Eigen::Vector4f max;
  max << params.scene.max_x, params.scene.max_y, params.scene.max_z, 1;
  pcl::CropBox<PointXYZRGB> cb;
  cb.setInputCloud(cloud);
  cb.setMin(min);
  cb.setMax(max);

  vector<int> box_indices;
  cb.filter(box_indices);

  PointIndices::Ptr pcl_indices(new PointIndices);
  pcl_indices->indices = box_indices;
  HSurface primary_surface;
  bool success = ParseHSurface(cloud, pcl_indices, params, &primary_surface);
  if (!success) {
    return false;
  }
  scene->set_primary_surface(primary_surface);
  return true;
}

bool ParseHSurface(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                   const PointIndices::ConstPtr& indices,
                   const ParseParams& params, HSurface* surface) {
  // Find the dominant plane, which should be mostly horizontal. This becomes
  // the HSurface.
  // Then, subtract the HSurface, and pass the points above the HSurface to
  // ParseObjects.
  PointIndices::Ptr plane_inliers(new PointIndices);
  bool found = FindHorizontalPlane<PointXYZRGB>(
      cloud, indices, params.hsurface.distance_threshold,
      params.hsurface.eps_angle, plane_inliers);
  if (!found) {
    return false;
  }

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = cloud->header.frame_id;
  geometry_msgs::Vector3 scale;
  GetPlanarBoundingBox(cloud, plane_inliers, &ps.pose, &scale);

  surface->SetCloud(cloud, plane_inliers);
  surface->set_name("primary_surface");
  surface->set_pose(ps);
  surface->set_scale(scale);

  // Find points above the surface.
  PointIndices::Ptr obj_indices(new PointIndices);
  tf::Vector3 surface_translation(ps.pose.position.x, ps.pose.position.y,
                                  ps.pose.position.z);
  tf::Quaternion surface_orientation;
  tf::quaternionMsgToTF(ps.pose.orientation, surface_orientation);
  tf::Transform base_to_surface(surface_orientation, surface_translation);
  for (size_t i = 0; i < indices->indices.size(); ++i) {
    int index = indices->indices[i];
    PointXYZRGB& point = cloud->points[index];
    tf::Vector3 point_v(point.x, point.y, point.z);
    // Project point into table space
    tf::Vector3 table_p = base_to_surface.inverse() * point_v;
    if (table_p.x() >= -scale.x / 2 && table_p.x() <= scale.x / 2 &&
        table_p.y() >= -scale.y / 2 && table_p.y() <= scale.y / 2 &&
        table_p.z() >= scale.z / 2) {
      obj_indices->indices.push_back(index);
    }
  }

  vector<Object> objects;
  ParseObjects(cloud, obj_indices, params, &objects);
  for (size_t i = 0; i < objects.size(); ++i) {
    surface->AddObject(objects[i]);
  }
  return true;
}

bool ParseObjects(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                  const pcl::PointIndices::ConstPtr& indices,
                  const ParseParams& params, std::vector<Object>* objects) {
  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>);
  boost::shared_ptr<vector<int> > indices_v(new vector<int>(indices->indices));
  tree->setInputCloud(cloud, indices_v);

  vector<PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(params.objects.cluster_tolerance);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.setIndices(indices);
  ec.setMinClusterSize(params.objects.min_cluster_size);
  ec.extract(cluster_indices);

  int obj_num = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    PointIndices::Ptr cluster_indices_p(new PointIndices);
    *cluster_indices_p = *it;
    geometry_msgs::PoseStamped ps;
    geometry_msgs::Vector3 scale;
    GetPlanarBoundingBox(cloud, cluster_indices_p, &ps.pose, &scale);
    std::stringstream ss;
    ss << "object_" << obj_num;

    Object obj;
    obj.set_name(ss.str());
    obj.set_pose(ps);
    obj.set_scale(scale);
    objects->push_back(obj);
  }
  return true;
}

// Crop the scene to just the tabletop area.
// Do a Euclidean clustering of the scene. We assume all points clustered with
// the table are either part of the table or an object on the table.
// So, the objects are clustered with the table, but not part of the table
// itself.
//  KdTree<PointXYZRGB>::Ptr tree(new KdTree<PointXYZRGB>);
//  tree->setInputCloud(cloud_.makeShared());
//
//  vector<PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
//  ec.setClusterTolerance(0.01);
//  ec.setMinClusterSize(25);
//  ec.setSearchMethod(tree);
//  ec.setInputCloud(cloud_.makeShared());
//  ec.extract(cluster_indices);
//
//  // Intersect the cluster indices with the tabletop indices.
//  std::set<int> table_indices;  // Indices of table points.
//  for (size_t i = 0; i < inliers->indices.size(); ++i) {
//    int table_index = inliers->indices[i];
//    table_indices.insert(table_index);
//  }
//
//  std::set<int> table_cluster_indices;  // The set of clusters that table
//  points
//                                        // can be found in.
//  int cluster_index = 0;
//  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
//       it != cluster_indices.end(); ++it) {
//    for (vector<int>::const_iterator pit = it->indices.begin();
//         pit != it->indices.end(); ++pit) {
//      int point_index = *pit;
//      if (table_indices.find(point_index) != table_indices.end()) {
//        // This is a tabletop point.
//        table_cluster_indices.insert(cluster_index);
//        break;
//      }
//    }
//    ++cluster_index;
//  }
//
//  PointIndices::Ptr objects_indices(new PointIndices);
//  cluster_index = 0;
//  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
//       it != cluster_indices.end(); ++it) {
//    if (table_cluster_indices.find(cluster_index) ==
//        table_cluster_indices.end()) {
//      ++cluster_index;
//      continue;
//    }
//    for (vector<int>::const_iterator pit = it->indices.begin();
//         pit != it->indices.end(); ++pit) {
//      int point_index = *pit;
//      if (table_indices.find(point_index) == table_indices.end()) {
//        // This is an object point, or below the table.
//        if (cloud_[point_index].z > primary_surface_->pose().pose.position.z)
//        {
//          objects_indices->indices.push_back(point_index);
//        }
//      }
//    }
//    ++cluster_index;
//  }
//
//  vector<Object> objects;
//  SegmentObjects(this, objects_indices, 0.01, &objects);
//  for (size_t i = 0; i < objects.size(); ++i) {
//    std::stringstream name;
//    name << "object_" << i;
//    objects[i].set_name(name.str());
//    primary_surface_->AddObject(objects[i]);
//  }
//}
//
// bool FindTabletop(const PointCloud<PointXYZRGB>& cloud,
//                  double distance_threshold, PointIndices::Ptr inliers) {
//  PointIndices::Ptr plane_inliers(new PointIndices);
//  bool found = FindHorizontalPlane(cloud, distance_threshold, plane_inliers);
//  if (!found) {
//    return found;
//  }
//
//  KdTree<PointXYZRGB>::Ptr tree(new KdTree<PointXYZRGB>);
//  KdTree<PointXYZRGB>::IndicesConstPtr plane_iptr(
//      new vector<int>(plane_inliers->indices));
//  tree->setInputCloud(cloud.makeShared(), plane_iptr);
//
//  vector<PointIndices> cluster_indices;
//  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
//  ec.setClusterTolerance(distance_threshold);
//  ec.setSearchMethod(tree);
//  ec.setInputCloud(cloud.makeShared());
//  ec.setIndices(plane_inliers);
//  ec.extract(cluster_indices);
//
//  if (cluster_indices.size() == 0) {
//    return false;
//  }
//
//  size_t max_index = -1;
//  size_t max_size = std::numeric_limits<size_t>::min();
//  for (size_t i = 0; i < cluster_indices.size(); ++i) {
//    size_t size = cluster_indices.size();
//    if (size > max_size) {
//      max_size = size;
//      max_index = i;
//    }
//  }
//
//  inliers->indices = cluster_indices[max_index].indices;
//  return true;
//}

}  // namespace perception
}  // namespace rapid