#include "object_search/object_search_node.h"

#include <string>
#include <vector>

#include "Eigen/Core"
#include "pcl/filters/crop_box.h"
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
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_parsing.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

#include "object_search_msgs/GetObjectInfo.h"
#include "object_search_msgs/Match.h"
#include "object_search_msgs/Search.h"
#include "object_search/capture_roi.h"
#include "object_search/commands.h"
#include "object_search/cloud_database.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using sensor_msgs::PointCloud2;

namespace object_search {
ObjectSearchNode::ObjectSearchNode(
    const rapid::perception::PoseEstimator& estimator,
    const RecordObjectCommand& record_object, const Database& object_db)
    : tf_listener_(),
      estimator_(estimator),
      record_object_(record_object),
      object_db_(object_db),
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

bool ObjectSearchNode::ServeGetObjectInfo(
    object_search_msgs::GetObjectInfoRequest& req,
    object_search_msgs::GetObjectInfoResponse& resp) {
  rapid_msgs::StaticCloud cloud;
  if (req.db_id != "") {
    object_db_.GetById(req.db_id, &cloud);
  } else {
    object_db_.Get(req.name, &cloud);
  }
  resp.name = cloud.name;
  resp.dimensions = cloud.roi.dimensions;
  return true;
}

bool ObjectSearchNode::ServeRecordObject(
    object_search_msgs::RecordObjectRequest& req,
    object_search_msgs::RecordObjectResponse& resp) {
  std::vector<std::string> args(1);
  args[0] = req.name;
  record_object_.Execute(args);
  resp.success = record_object_.last_id() != "";
  resp.db_id = record_object_.last_id();
  resp.name = record_object_.last_name();
  resp.roi = record_object_.last_roi();
  return true;
}

void ObjectSearchNode::Search(const rapid_msgs::StaticCloud& scene,
                              const rapid_msgs::StaticCloud& object,
                              const bool is_tabletop, const double max_error,
                              const int min_results,
                              std::vector<object_search_msgs::Match>* matches) {
  matches->clear();
  UpdateParams();

  PointCloudC::Ptr scene_in(new PointCloudC);
  PointCloudC::Ptr object_in(new PointCloudC);
  pcl::fromROSMsg(scene.cloud, *scene_in);
  pcl::fromROSMsg(object.cloud, *object_in);
  ROS_INFO("Scene (frame %s) has %ld points", scene_in->header.frame_id.c_str(),
           scene_in->size());
  ROS_INFO("Object (frame %s) has %ld points",
           object_in->header.frame_id.c_str(), object_in->size());

  PointCloudC::Ptr scene_transformed(new PointCloudC);
  PointCloudC::Ptr object_transformed(new PointCloudC);
  TransformToBase(scene_in, scene.parent_frame_id, scene.base_to_camera,
                  scene_transformed);
  TransformToBase(object_in, object.parent_frame_id, object.base_to_camera,
                  object_transformed);
  ROS_INFO("Scene transformed to frame %s",
           scene_transformed->header.frame_id.c_str());
  ROS_INFO("Object transformed to frame %s",
           object_transformed->header.frame_id.c_str());

  PointCloudC::Ptr scene_cropped(new PointCloudC);
  if (is_tabletop) {
    ExtractTabletop(scene_transformed, scene_cropped);
    ROS_INFO("Extracted %ld points from tabletop", scene_cropped->size());
  } else {
    CropScene(scene_transformed, scene_cropped);
    ROS_INFO("Cropped scene to %ld points", scene_cropped->size());
  }

  PointCloudC::Ptr scene_sampled(new PointCloudC);
  PointCloudC::Ptr object_sampled(new PointCloudC);
  Downsample(scene_cropped, scene_sampled);
  ROS_INFO("Downsampled scene to %ld points", scene_sampled->size());
  Downsample(object_transformed, object_sampled);

  rapid::perception::RandomHeatMapper* heat_mapper =
      static_cast<rapid::perception::RandomHeatMapper*>(
          estimator_.heat_mapper());
  heat_mapper->set_sample_ratio(sample_ratio_);
  heat_mapper->set_max_samples(max_samples_);

  estimator_.set_sigma_threshold(sigma_threshold_);
  estimator_.set_nms_radius(nms_radius_);
  estimator_.set_num_candidates(max_samples_);

  estimator_.set_scene(scene_sampled);
  estimator_.set_object(object_sampled, object.roi);
  if (max_error == 0) {
    estimator_.set_fitness_threshold(fitness_threshold_);
  } else {
    estimator_.set_fitness_threshold(max_error);
  }
  estimator_.set_min_results(min_results);

  std::vector<rapid::perception::PoseEstimationMatch> pe_matches;
  estimator_.Find(&pe_matches);

  for (size_t i = 0; i < pe_matches.size(); ++i) {
    const rapid::perception::PoseEstimationMatch& match = pe_matches[i];
    object_search_msgs::Match msg;
    msg.pose = match.pose();
    pcl::toROSMsg(*match.cloud(), msg.cloud);
    msg.error = match.fitness();
    matches->push_back(msg);
  }
}

bool ObjectSearchNode::ServeSearch(object_search_msgs::SearchRequest& req,
                                   object_search_msgs::SearchResponse& resp) {
  Search(req.scene, req.object, req.is_tabletop, req.max_error, req.min_results,
         &resp.matches);
  return true;
}

bool ObjectSearchNode::ServeSearchFromDb(
    object_search_msgs::SearchFromDbRequest& req,
    object_search_msgs::SearchFromDbResponse& resp) {
  // Read scene from cloud_in
  PointCloud2::ConstPtr cloud_in =
      ros::topic::waitForMessage<PointCloud2>("cloud_in", ros::Duration(10));
  rapid_msgs::StaticCloud scene;
  scene.cloud = *cloud_in;

  // Get transform
  scene.parent_frame_id = "base_link";
  try {
    tf::StampedTransform base_to_camera_tf;
    tf_listener_.lookupTransform(scene.cloud.header.frame_id, "base_link",
                                 scene.cloud.header.stamp, base_to_camera_tf);
    tf::transformTFToMsg(base_to_camera_tf, scene.base_to_camera);
  } catch (tf::TransformException e) {
    ROS_WARN("%s", e.what());
  }

  rapid_msgs::StaticCloud object;
  ROS_INFO("object_id: %s, name: %s", req.object_id.c_str(), req.name.c_str());
  if (req.object_id != "") {
    bool success = object_db_.GetById(req.object_id, &object);
    if (!success) {
      ROS_ERROR("Invalid ID: %s", req.object_id.c_str());
      return false;
    }
  } else {
    bool success = object_db_.Get(req.name, &object);
    if (!success) {
      ROS_ERROR("Invalid name: %s", req.name.c_str());
      return false;
    }
  }

  Search(scene, object, req.is_tabletop, req.max_error, req.min_results,
         &resp.matches);
  return true;
}

void ObjectSearchNode::UpdateParams() {
  ros::param::param<double>("leaf_size", leaf_size_, 0.005);
  ros::param::param<double>("min_x", min_x_, 0.3);
  ros::param::param<double>("min_y", min_y_, -0.75);
  ros::param::param<double>("min_z", min_z_, 0.3);
  ros::param::param<double>("max_x", max_x_, 1.2);
  ros::param::param<double>("max_y", max_y_, 0.75);
  ros::param::param<double>("max_z", max_z_, 1.7);
  ros::param::param<double>("sample_ratio", sample_ratio_, 0.02);
  ros::param::param<int>("max_samples", max_samples_, 500);
  ros::param::param<double>("fitness_threshold", fitness_threshold_, 0.0055);
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

void ObjectSearchNode::CropScene(PointCloudC::Ptr in, PointCloudC::Ptr out) {
  ROS_INFO(
      "Cropping:\n"
      "  min_x: %f\n"
      "  min_y: %f\n"
      "  min_z: %f\n"
      "  max_x: %f\n"
      "  max_y: %f\n"
      "  max_z: %f\n",
      min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(in);
  Eigen::Vector4f min;
  min << min_x_, min_y_, min_z_, 1;
  Eigen::Vector4f max;
  max << max_x_, max_y_, max_z_, 1;
  crop.setMin(min);
  crop.setMax(max);
  crop.filter(*out);
}

void ObjectSearchNode::ExtractTabletop(PointCloudC::Ptr in,
                                       PointCloudC::Ptr out) {
  rapid::perception::Scene scene;
  rapid::perception::ParseScene(in, rapid::perception::Pr2Params(), &scene);
  std::vector<rapid::perception::Object> objects =
      scene.primary_surface().objects();
  for (size_t i = 0; i < objects.size(); ++i) {
    const rapid::perception::Object& object = objects[i];
    *out += *object.GetCloud();
  }
  out->header.frame_id = in->header.frame_id;
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

  object_search::ObjectSearchNode node(pose_estimator, record_object,
                                       object_db);
  ros::ServiceServer get_info_service = nh.advertiseService(
      "get_object_info", &object_search::ObjectSearchNode::ServeGetObjectInfo,
      &node);
  ros::ServiceServer search_service = nh.advertiseService(
      "find_object", &object_search::ObjectSearchNode::ServeSearch, &node);
  ros::ServiceServer search_from_db_service = nh.advertiseService(
      "find_object_from_db",
      &object_search::ObjectSearchNode::ServeSearchFromDb, &node);
  ros::ServiceServer record_object_service = nh.advertiseService(
      "record_object", &object_search::ObjectSearchNode::ServeRecordObject,
      &node);

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}
