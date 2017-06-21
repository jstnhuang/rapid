#include "rapid_pbd/surface_segmentation_action.h"

#include <string>

#include "actionlib/server/simple_action_server.h"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rapid_pbd/action_names.h"
#include "rapid_pbd_msgs/SegmentSurfacesAction.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"
#include "surface_perception/visualization.h"
#include "visualization_msgs/Marker.h"

#include "rapid_pbd/scene_db.h"

using surface_perception::SurfaceObjects;
using surface_perception::Object;

namespace rapid {
namespace pbd {
SurfaceSegmentationAction::SurfaceSegmentationAction(const std::string& topic,
                                                     const SceneDb& scene_db)
    : topic_(topic),
      scene_db_(scene_db),
      as_(kSurfaceSegmentationActionName,
          boost::bind(&SurfaceSegmentationAction::Execute, this, _1), false),
      seg_(),
      nh_(),
      cropped_pub_(nh_.advertise<sensor_msgs::PointCloud2>(
          "surface_seg/cropped_scene", 1, true)),
      marker_pub_(nh_.advertise<visualization_msgs::Marker>(
          "surface_seg/visualization", 100)),
      viz_(marker_pub_) {}

void SurfaceSegmentationAction::Start() { as_.start(); }

void SurfaceSegmentationAction::Execute(
    const rapid_pbd_msgs::SegmentSurfacesGoalConstPtr& goal) {
  boost::shared_ptr<const sensor_msgs::PointCloud2> cloud_msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_,
                                                           ros::Duration(10.0));
  // TODO: transform into base_link if it's not already in it.
  rapid_pbd_msgs::SegmentSurfacesResult result;
  if (!cloud_msg) {
    ROS_ERROR("Failed to get point cloud on topic: %s.", topic_.c_str());
    as_.setSucceeded(result);
    return;
  }
  if (goal->save_cloud) {
    result.cloud_db_id = scene_db_.Insert(*cloud_msg);
  }
  PointCloudC::Ptr cloud(new PointCloudC);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  double min_x = 0, min_y = 0, min_z = 0;
  double max_x = 0, max_y = 0, max_z = 0;
  ros::param::param<double>("crop_min_x", min_x, -1);
  ros::param::param<double>("crop_min_y", min_y, -1);
  ros::param::param<double>("crop_min_z", min_z, 0.1);
  ros::param::param<double>("crop_max_x", max_x, 1);
  ros::param::param<double>("crop_max_y", max_y, 1);
  ros::param::param<double>("crop_max_z", max_z, 1.5);
  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  crop.setMin(min);
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMax(max);
  crop.filter(point_indices->indices);

  sensor_msgs::PointCloud2 viz_msg;
  PointCloudC::Ptr viz_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(point_indices);
  extract.filter(*viz_cloud);
  ROS_INFO("Extracted %ld indices to %ld points", point_indices->indices.size(),
           viz_cloud->size());
  pcl::toROSMsg(*viz_cloud, viz_msg);
  cropped_pub_.publish(viz_msg);

  double horizontal_tolerance_degrees;
  ros::param::param("horizontal_tolerance_degrees",
                    horizontal_tolerance_degrees, 10.0);
  double margin_above_surface;
  ros::param::param("margin_above_surface", margin_above_surface, 0.01);
  double cluster_distance;
  ros::param::param("cluster_distance", cluster_distance, 0.01);
  int min_cluster_size;
  ros::param::param("min_cluster_size", min_cluster_size, 10);
  int max_cluster_size;
  ros::param::param("max_cluster_size", max_cluster_size, 10000);

  surface_perception::Segmentation seg;
  seg.set_input_cloud(cloud);
  seg.set_indices(point_indices);
  seg.set_horizontal_tolerance_degrees(horizontal_tolerance_degrees);
  seg.set_margin_above_surface(margin_above_surface);
  seg.set_cluster_distance(cluster_distance);
  seg.set_min_cluster_size(min_cluster_size);
  seg.set_max_cluster_size(max_cluster_size);

  std::vector<surface_perception::SurfaceObjects> surface_objects;
  bool success = seg.Segment(&surface_objects);

  if (!success) {
    ROS_ERROR("Failed to perceive surface objects.");
    as_.setSucceeded(result);
    return;
  }

  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  size_t num_objects = 0;

  for (size_t i = 0; i < surface_objects.size(); ++i) {
    const SurfaceObjects& surface_scene = surface_objects[i];
    num_objects += surface_scene.objects.size();
    for (size_t j = 0; j < surface_scene.objects.size(); ++j) {
      const Object& object = surface_scene.objects[j];
      size_t cloud_size = object.indices->indices.size();
      if (cloud_size < min_size) {
        min_size = cloud_size;
      }
      if (cloud_size > max_size) {
        max_size = cloud_size;
      }
      rapid_pbd_msgs::Landmark landmark;
      landmark.type = rapid_pbd_msgs::Landmark::SURFACE_BOX;
      landmark.frame_id = cloud->header.frame_id;
      landmark.pose_stamped = object.pose_stamped;
      landmark.surface_box_dims = object.dimensions;
      result.landmarks.push_back(landmark);
    }
  }
  ROS_INFO("Detected %ld objects, smallest: %ld points, largest: %ld points",
           num_objects, min_size, max_size);
  viz_.set_surface_objects(surface_objects);
  viz_.Show();
  as_.setSucceeded(result);
}
}  // namespace pbd
}  // namespace rapid
