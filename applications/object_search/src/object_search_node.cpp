#include "object_search/object_search_node.h"

#include <string>
#include <vector>

#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

#include "object_search_msgs/Match.h"
#include "object_search_msgs/Search.h"
#include "object_search/capture_roi.h"
#include "object_search/commands.h"
#include "object_search/cloud_database.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace object_search {
ObjectSearchNode::ObjectSearchNode(
    const rapid::perception::PoseEstimator& estimator,
    const RecordObjectCommand& record_object)
    : estimator_(estimator),
      record_object_(record_object),
      leaf_size_(0.005),
      min_x_(0.2),
      min_y_(-1),
      min_z_(0.3),
      max_x_(1.2),
      max_y_(1),
      max_z_(1.7),
      sample_ratio_(0.02),
      max_samples_(500),
      fitness_threshold_(0.0045),
      sigma_threshold_(8),
      nms_radius_(0.02) {}

bool ObjectSearchNode::ServeRecordObject(
    object_search_msgs::RecordObjectRequest& req,
    object_search_msgs::RecordObjectResponse& resp) {
  std::vector<std::string> args(1);
  args.push_back(req.name);
  record_object_.Execute(args);
  resp.success = record_object_.last_id() != "";
  resp.db_id = record_object_.last_id();
  return true;
}

bool ObjectSearchNode::ServeSearch(object_search_msgs::SearchRequest& req,
                                   object_search_msgs::SearchResponse& resp) {
  UpdateParams();

  PointCloudC::Ptr scene_in(new PointCloudC);
  PointCloudC::Ptr object_in(new PointCloudC);
  pcl::fromROSMsg(req.scene.cloud, *scene_in);
  pcl::fromROSMsg(req.object.cloud, *object_in);

  PointCloudC::Ptr scene_sampled(new PointCloudC);
  PointCloudC::Ptr object_sampled(new PointCloudC);
  Downsample(scene_in, scene_sampled);
  ROS_INFO("Downsampled scene to %ld points", scene_sampled->size());
  Downsample(object_in, object_sampled);

  PointCloudC::Ptr scene_transformed(new PointCloudC);
  PointCloudC::Ptr object_transformed(new PointCloudC);
  TransformToBase(scene_sampled, req.scene.parent_frame_id,
                  req.scene.base_to_camera, scene_transformed);
  TransformToBase(scene_sampled, req.object.parent_frame_id,
                  req.object.base_to_camera, scene_transformed);

  CropScene(scene_transformed);

  rapid::perception::RandomHeatMapper* heat_mapper =
      static_cast<rapid::perception::RandomHeatMapper*>(
          estimator_.heat_mapper());
  heat_mapper->set_sample_ratio(sample_ratio_);
  heat_mapper->set_max_samples(max_samples_);

  estimator_.set_fitness_threshold(fitness_threshold_);
  estimator_.set_sigma_threshold(sigma_threshold_);
  estimator_.set_nms_radius(nms_radius_);
  estimator_.set_num_candidates(max_samples_);

  estimator_.set_scene(scene_transformed);
  estimator_.set_object(object_transformed, req.object.roi);
  estimator_.set_fitness_threshold(req.max_error);
  estimator_.set_min_results(req.min_results);

  std::vector<rapid::perception::PoseEstimationMatch> matches;
  estimator_.Find(&matches);

  for (size_t i = 0; i < matches.size(); ++i) {
    const rapid::perception::PoseEstimationMatch& match = matches[i];
    object_search_msgs::Match msg;
    msg.transform.translation.x = match.translation().x();
    msg.transform.translation.y = match.translation().y();
    msg.transform.translation.z = match.translation().z();
    msg.transform.rotation.w = match.rotation().w();
    msg.transform.rotation.x = match.rotation().x();
    msg.transform.rotation.y = match.rotation().y();
    msg.transform.rotation.z = match.rotation().z();
    pcl::toROSMsg(*match.cloud(), msg.cloud);
    msg.error = match.fitness();
  }

  return true;
}

void ObjectSearchNode::UpdateParams() {
  ros::param::param<double>("leaf_size_", leaf_size_, 0.005);
  ros::param::param<double>("min_x", min_x_, 0.2);
  ros::param::param<double>("min_y", min_y_, -1);
  ros::param::param<double>("min_z", min_z_, 0.3);
  ros::param::param<double>("max_x", max_x_, 1.2);
  ros::param::param<double>("max_y", max_y_, 1);
  ros::param::param<double>("max_z", max_z_, 1.7);
  ros::param::param<double>("sample_ratio", sample_ratio_, 0.02);
  ros::param::param<int>("max_samples", max_samples_, 500);
  ros::param::param<double>("fitness_threshold", fitness_threshold_, 0.0045);
  ros::param::param<double>("sigma_threshold", sigma_threshold_, 8);
  ros::param::param<double>("nms_radius", nms_radius_, 0.02);
}

void ObjectSearchNode::Downsample(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out) {
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(in);
  vox.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  vox.filter(*out);
}

void ObjectSearchNode::TransformToBase(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr in,
    const std::string& parent_frame_id,
    const geometry_msgs::Transform& base_to_camera,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out) {
  tf::Transform base_to_camera_tf;
  tf::transformMsgToTF(base_to_camera, base_to_camera_tf);
  pcl_ros::transformPointCloud(*in, *out, base_to_camera_tf.inverse());
  out->header.frame_id = parent_frame_id;
}

void ObjectSearchNode::CropScene(PointCloudC::Ptr scene) {
  ROS_INFO(
      "Cropping:\n"
      "  min_x: %f\n"
      "  min_y: %f\n"
      "  min_z: %f\n"
      "  max_x: %f\n"
      "  max_y: %f\n"
      "  max_z: %f\n",
      min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);
}
}  // namespace object_search

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search_service");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Visualization publishers
  ros::Publisher object_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/landmark", 1, true);
  ros::Publisher scene_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/scene", 1, true);
  ros::Publisher heatmap_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/heatmap", 1, true);
  ros::Publisher candidates_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/candidate_samples", 1, true);
  ros::Publisher alignment_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/alignment", 1, true);
  ros::Publisher output_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/output", 1, true);
  rapid_ros::Publisher<visualization_msgs::Marker> marker_pub(
      nh.advertise<visualization_msgs::Marker>("/find_object_markers", 1,
                                               true));

  // Build heat mapper
  rapid::perception::RandomHeatMapper* heat_mapper =
      new rapid::perception::RandomHeatMapper();
  heat_mapper->set_name("random");
  heat_mapper->set_heatmap_publisher(heatmap_pub);

  // Build pose estimator
  rapid::perception::PoseEstimator pose_estimator(heat_mapper);
  pose_estimator.set_scene_publisher(scene_pub);
  pose_estimator.set_object_publisher(object_pub);
  pose_estimator.set_candidates_publisher(candidates_pub);
  pose_estimator.set_alignment_publisher(alignment_pub);
  pose_estimator.set_output_publisher(output_pub);
  pose_estimator.set_marker_publisher(&marker_pub);

  // Build databases
  ros::ServiceClient get_cloud =
      nh.serviceClient<rapid_msgs::GetStaticCloud>("get_static_cloud");
  ros::ServiceClient list_clouds =
      nh.serviceClient<rapid_msgs::ListStaticClouds>("list_static_clouds");
  ros::ServiceClient remove_cloud =
      nh.serviceClient<rapid_msgs::RemoveStaticCloud>("remove_static_cloud");
  ros::ServiceClient save_cloud =
      nh.serviceClient<rapid_msgs::SaveStaticCloud>("save_static_cloud");
  object_search::Database object_db("object_search", "objects", get_cloud,
                                    list_clouds, remove_cloud, save_cloud);

  rapid::perception::Box3DRoiServer roi_server("roi");
  roi_server.set_base_frame("base_link");
  object_search::CaptureRoi capture(&roi_server);
  capture.set_base_frame("base_link");
  object_search::RecordObjectCommand record_object(&object_db, &capture);

  object_search::ObjectSearchNode node(pose_estimator, record_object);
  ros::ServiceServer search_service = nh.advertiseService(
      "find_object", &object_search::ObjectSearchNode::ServeSearch, &node);
  ros::ServiceServer record_object_service = nh.advertiseService(
      "record_object", &object_search::ObjectSearchNode::ServeRecordObject,
      &node);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}
