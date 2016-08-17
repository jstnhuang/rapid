#include "rapid_perception/scene_parsing.h"

#include <sstream>
#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/region_growing_rgb.h"

#include "rapid_perception/rgbd.h"
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
  p.hsurface.distance_threshold = 0.015;
  p.hsurface.eps_angle = rapid::utils::DegreesToRadians(5);
  p.objects.distance_threshold = 0.05;
  p.objects.point_color_threshold = 35;
  p.objects.region_color_threshold = 20;
  p.objects.min_cluster_size = 38;
  return p;
}

bool ParseScene(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
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
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = cloud->header.frame_id;
  ps.pose.position.x = (params.scene.max_x + params.scene.min_x) / 2;
  ps.pose.position.y = (params.scene.max_y + params.scene.min_y) / 2;
  ps.pose.position.z = (params.scene.max_z + params.scene.min_z) / 2;
  ps.pose.orientation.w = 1;
  scene->set_pose(ps);
  geometry_msgs::Vector3 scale;
  scale.x = params.scene.max_x - params.scene.min_x;
  scale.y = params.scene.max_y - params.scene.min_y;
  scale.z = params.scene.max_z - params.scene.min_z;
  scene->set_scale(scale);
  return true;
}

bool ParseHSurface(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                   const PointIndices::ConstPtr& indices,
                   const ParseParams& params, HSurface* surface) {
  // Find the dominant plane, which should be mostly horizontal. This becomes
  // the HSurface.
  PointIndices::Ptr plane_inliers(new PointIndices);
  bool found =
      FindHorizontalPlane(cloud, indices, params.hsurface.distance_threshold,
                          params.hsurface.eps_angle, plane_inliers);
  if (!found) {
    return false;
  }

  geometry_msgs::PoseStamped ps;
  geometry_msgs::Vector3 scale;
  GetBoundingBox(cloud, plane_inliers, &ps, &scale);

  surface->SetCloud(cloud, plane_inliers);
  surface->set_name("primary_surface");
  surface->set_pose(ps);
  surface->set_scale(scale);

  // Find points above the HSurface
  PointIndices::Ptr object_indices(new PointIndices);
  tf::Vector3 surface_translation(ps.pose.position.x, ps.pose.position.y,
                                  ps.pose.position.z);
  tf::Quaternion surface_orientation;
  tf::quaternionMsgToTF(ps.pose.orientation, surface_orientation);
  tf::Transform base_to_surface(surface_orientation, surface_translation);
  for (size_t i = 0; i < indices->indices.size(); ++i) {
    int index = indices->indices[i];
    const PointXYZRGB& point = cloud->points[index];
    tf::Vector3 point_v(point.x, point.y, point.z);
    // Project point into table space
    tf::Vector3 table_p = base_to_surface.inverse() * point_v;
    if (table_p.x() >= -scale.x / 2 && table_p.x() <= scale.x / 2 &&
        table_p.y() >= -scale.y / 2 && table_p.y() <= scale.y / 2 &&
        table_p.z() > scale.z / 2) {
      object_indices->indices.push_back(index);
    }
  }

  // We must reify the object cloud now because the outlier filters search for
  // neighbors in the entire input cloud, not just in the indices subset.
  PointCloud<PointXYZRGB>::Ptr object_cloud =
      IndicesToCloud(cloud, object_indices);

  pcl::RadiusOutlierRemoval<PointXYZRGB> radius;
  radius.setInputCloud(object_cloud);
  radius.setRadiusSearch(0.05);
  radius.setMinNeighborsInRadius(10);
  radius.filter(*object_cloud);

  vector<Object> objects;
  PointIndices::Ptr all_indices(new PointIndices);
  ParseObjects(object_cloud, all_indices, params, &objects);
  for (size_t i = 0; i < objects.size(); ++i) {
    surface->AddObject(objects[i]);
  }
  return true;
}

bool ParseObjects(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                  const pcl::PointIndices::ConstPtr& indices,
                  const ParseParams& params, std::vector<Object>* objects) {
  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>);
  tree->setInputCloud(cloud);

  vector<PointIndices> cluster_indices;
  pcl::RegionGrowingRGB<PointXYZRGB> ec;
  ec.setInputCloud(cloud);
  if (indices->indices.size() != 0) {
    ec.setIndices(indices);
  }
  ec.setSearchMethod(tree);
  ec.setDistanceThreshold(params.objects.distance_threshold);
  ec.setPointColorThreshold(params.objects.point_color_threshold);
  ec.setRegionColorThreshold(params.objects.region_color_threshold);
  ec.setMinClusterSize(params.objects.min_cluster_size);
  ec.extract(cluster_indices);

  int obj_num = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    PointIndices::Ptr cluster_indices_p(new PointIndices);
    *cluster_indices_p = *it;
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = cloud->header.frame_id;
    geometry_msgs::Vector3 scale;
    GetPlanarBoundingBox(cloud, cluster_indices_p, &ps.pose, &scale);
    std::stringstream ss;
    ss << "object_" << obj_num++;

    Object obj;
    obj.SetCloud(cloud, cluster_indices_p);
    obj.set_name(ss.str());
    obj.set_pose(ps);
    obj.set_scale(scale);
    objects->push_back(obj);
  }
  return true;
}
}  // namespace perception
}  // namespace rapid
