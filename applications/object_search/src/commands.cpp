#include "object_search/commands.h"

#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "pcl/common/time.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_msgs/StaticCloud.h"
#include "rapid_msgs/StaticCloudInfo.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/pose_estimation_match.h"
#include "rapid_perception/random_heat_mapper.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "object_search/capture_roi.h"
#include "object_search/cloud_database.h"
#include "object_search/estimators.h"

using pcl::FPFHSignature33;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointXYZRGBNormal;
using sensor_msgs::PointCloud2;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using rapid_msgs::StaticCloud;
using rapid_msgs::StaticCloudInfo;
using rapid::perception::PoseEstimationMatch;
using rapid::perception::GroupingPoseEstimator;

namespace object_search {
ListCommand::ListCommand(Database* db, const string& type)
    : db_(db), type_(type) {}

void ListCommand::Execute(vector<string>& args) {
  vector<StaticCloudInfo> clouds;
  db_->List(&clouds);
  if (type_ == "object") {
    cout << "Objects:" << endl;
  } else {
    cout << "Scenes:" << endl;
  }
  if (clouds.size() == 0) {
    cout << "  None." << endl;
  }
  for (size_t i = 0; i < clouds.size(); ++i) {
    cout << "  " << clouds[i].id << "\t" << clouds[i].name << endl;
  }
}

RecordObjectCommand::RecordObjectCommand(Database* db, CaptureRoi* capture)
    : db_(db), capture_(capture), last_id_(""), last_name_("") {}

void RecordObjectCommand::Execute(std::vector<std::string>& args) {
  last_id_ = "";    // Reset ID
  last_name_ = "";  // Reset ID
  rapid_msgs::Roi3D roi;
  last_roi_ = roi;

  // Start server and wait for input
  capture_->ShowMarker();
  cout << "Adjust the ROI marker in rviz." << endl;

  string name("");
  string input("");
  if (args.size() > 0 && args[0] != "") {
    name = args[0];
    cout << "Type \"save\" to save or \"cancel\" to cancel: ";
    std::getline(std::cin, input);
  } else {
    cout << "Give this landmark a name, or type \"cancel\" to cancel: ";
    std::getline(std::cin, input);
    name = input;
  }
  capture_->HideMarker();
  if (input == "cancel") {
    return;
  }

  // Read cloud and saved region
  PointCloud2::ConstPtr cloud_in =
      ros::topic::waitForMessage<PointCloud2>("cloud_in", ros::Duration(10));
  capture_->set_cloud(cloud_in);
  StaticCloud static_cloud;
  capture_->Capture(&static_cloud.cloud);

  // Get transform
  static_cloud.parent_frame_id = capture_->base_frame();
  tf::transformTFToMsg(capture_->cloud_to_base().inverse(),
                       static_cloud.base_to_camera);
  static_cloud.roi = capture_->roi();
  last_roi_ = static_cloud.roi;

  // Save static cloud
  static_cloud.name = name;
  last_id_ = db_->Save(static_cloud);
  last_name_ = name;
  std::cout << "Saved " << static_cloud.name << " with ID " << last_id_
            << std::endl;
}

std::string RecordObjectCommand::last_id() { return last_id_; }

std::string RecordObjectCommand::last_name() { return last_name_; }

rapid_msgs::Roi3D RecordObjectCommand::last_roi() { return last_roi_; }

RecordSceneCommand::RecordSceneCommand(Database* db)
    : db_(db), tf_listener_() {}

void RecordSceneCommand::Execute(std::vector<std::string>& args) {
  // Read cloud and saved region
  PointCloud2::ConstPtr cloud_in =
      ros::topic::waitForMessage<PointCloud2>("cloud_in", ros::Duration(10));
  StaticCloud static_cloud;
  static_cloud.cloud = *cloud_in;

  // Get transform
  static_cloud.parent_frame_id = "base_footprint";
  try {
    tf::StampedTransform base_to_camera_tf;
    tf_listener_.lookupTransform(
        static_cloud.cloud.header.frame_id, "base_footprint",
        static_cloud.cloud.header.stamp, base_to_camera_tf);
    tf::transformTFToMsg(base_to_camera_tf, static_cloud.base_to_camera);
  } catch (tf::TransformException e) {
    ROS_WARN("%s", e.what());
  }

  // Save static cloud
  static_cloud.name = args[0];
  db_->Save(static_cloud);
}

DeleteCommand::DeleteCommand(Database* db) : db_(db) {}

void DeleteCommand::Execute(std::vector<std::string>& args) {
  bool success = db_->Remove(args[0]);
  if (!success) {
    cout << "Invalid ID " << args[0] << ", nothing deleted." << endl;
  }
}

UseCommand::UseCommand(Database* db, Estimators* estimators,
                       const std::string& type, const ros::Publisher& pub)
    : db_(db), estimators_(estimators), type_(type), pub_(pub) {}

void UseCommand::Execute(std::vector<std::string>& args) {
  StaticCloud cloud;
  bool success = db_->Get(args[0], &cloud);
  if (!success) {
    cout << "Invalid ID " << args[0] << endl;
    return;
  }

  PointCloud<PointXYZRGB>::Ptr pcl_cloud_filtered(new PointCloud<PointXYZRGB>);
  pcl::fromROSMsg(cloud.cloud, *pcl_cloud_filtered);

  if (type_ == "scene") {
    // Filter NaNs
    std::vector<int> mapping;
    pcl_cloud_filtered->is_dense = false;  // Force check for NaNs
    pcl::removeNaNFromPointCloud(*pcl_cloud_filtered, *pcl_cloud_filtered,
                                 mapping);
    ROS_INFO("Filtered NaNs, there are now %ld points",
             pcl_cloud_filtered->size());
  }

  // Transform into base_footprint
  tf::Transform base_to_camera;
  tf::transformMsgToTF(cloud.base_to_camera, base_to_camera);
  PointCloud<PointXYZRGB>::Ptr pcl_cloud_base(new PointCloud<PointXYZRGB>);
  pcl_ros::transformPointCloud(*pcl_cloud_filtered, *pcl_cloud_base,
                               base_to_camera.inverse());
  pcl_cloud_base->header.frame_id = cloud.parent_frame_id;

  ROS_INFO("Loaded point cloud with %ld points", pcl_cloud_base->size());

  if (type_ == "scene") {
    vector<int> indices;
    CropScene(pcl_cloud_base, &indices);
    pcl::PointIndicesPtr indices_ptr(new pcl::PointIndices);
    indices_ptr->indices = indices;

    // Get cropped versions of scene in both frames
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud(pcl_cloud);
    // extract.setIndices(indices_ptr);
    // extract.filter(*pcl_cloud);

    // Get cropped version of camera frame image
    extract.setInputCloud(pcl_cloud_base);
    extract.setIndices(indices_ptr);
    extract.filter(*pcl_cloud_base);
  }

  // Visualize the scene/object
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*pcl_cloud_base, msg);
  if (pub_) {
    pub_.publish(msg);
  }

  // RANSAC estimator does downsampling internally.
  PointCloud<PointXYZRGB>::Ptr pcl_cloud_downsampled(
      new PointCloud<PointXYZRGB>);
  double leaf_size = 0.01;
  ros::param::param<double>("leaf_size", leaf_size, 0.01);
  pcl::VoxelGrid<PointXYZRGB> vox;
  vox.setInputCloud(pcl_cloud_base);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*pcl_cloud_downsampled);
  ROS_INFO("Downsampled to %ld points", pcl_cloud_downsampled->size());
  if (estimators_->ransac->voxel_size() != leaf_size) {
    estimators_->ransac->set_voxel_size(leaf_size);
  }

  if (type_ == "object") {
    estimators_->custom->set_object(pcl_cloud_downsampled);
    estimators_->custom->set_roi(cloud.roi);
    // estimators_->ransac->set_object(pcl_cloud_base);
    estimators_->grouping->set_object(pcl_cloud_base);
  } else {
    estimators_->custom->set_scene(pcl_cloud_downsampled);
    // estimators_->ransac->set_scene(pcl_cloud_base);
    estimators_->grouping->set_scene(pcl_cloud_base);
  }
}

void UseCommand::CropScene(PointCloud<PointXYZRGB>::Ptr scene,
                           vector<int>* indices) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param<double>("min_x", min_x, 0.2);
  ros::param::param<double>("min_y", min_y, -1);
  ros::param::param<double>("min_z", min_z, 0.3);
  ros::param::param<double>("max_x", max_x, 1.2);
  ros::param::param<double>("max_y", max_y, 1);
  ros::param::param<double>("max_z", max_z, 1.7);
  ROS_INFO(
      "Cropping:\n"
      "  min_x: %f\n"
      "  min_y: %f\n"
      "  min_z: %f\n"
      "  max_x: %f\n"
      "  max_y: %f\n"
      "  max_z: %f\n",
      min_x, min_y, min_z, max_x, max_y, max_z);

  pcl::CropBox<PointXYZRGB> crop;
  crop.setInputCloud(scene);
  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMin(min);
  crop.setMax(max);
  crop.filter(*indices);
  // crop.filter(*scene);
  ROS_INFO("Cropped to %ld points", indices->size());
}

RunCommand::RunCommand(Estimators* estimators, const ros::Publisher& output_pub)
    : estimators_(estimators), matches_(), output_pub_(output_pub) {}

void RunCommand::Execute(std::vector<std::string>& args) {
  const std::string& algorithm = args[0];
  matches_.clear();
  if (algorithm == "custom") {
    UpdateCustomParams();
    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->custom->Find(&matches_);
  } else if (algorithm == "ransac") {
    UpdateRansacParams();
    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->ransac->Find(&matches_);
  } else if (algorithm == "grouping") {
    UpdateGroupingParams();
    pcl::ScopeTime timer(("Running algorithm: " + algorithm).c_str());
    estimators_->grouping->Find(&matches_);
  } else {
    ROS_ERROR("Unknown algorithm: %s", algorithm.c_str());
    return;
  }
  VisualizeMatches(output_pub_, matches_);
}

void RunCommand::matches(
    std::vector<rapid::perception::PoseEstimationMatch>* matches) {
  *matches = matches_;
}

void RunCommand::UpdateCustomParams() {
  double sample_ratio;
  int max_samples;
  int num_candidates;
  double fitness_threshold;
  double sigma_threshold;
  double nms_radius;
  int min_results;

  ros::param::param<double>("sample_ratio", sample_ratio, 0.01);
  ros::param::param<int>("max_samples", max_samples, 1000);
  ros::param::param<int>("num_candidates", num_candidates, 100);
  ros::param::param<double>("fitness_threshold", fitness_threshold, 0.0055);
  ros::param::param<double>("sigma_threshold", sigma_threshold, 2);
  ros::param::param<double>("nms_radius", nms_radius, 0.03);
  ros::param::param<int>("min_results", min_results, 0);
  ROS_INFO(
      "Parameters:\n"
      "sample_ratio: %f\n"
      "max_samples: %d\n"
      "num_candidates: %d\n"
      "fitness_threshold: %f\n"
      "sigma_threshold: %f\n"
      "nms_radius: %f\n"
      "min_results: %d\n",
      sample_ratio, max_samples, num_candidates, fitness_threshold,
      sigma_threshold, nms_radius, min_results);

  if (estimators_->custom->heat_mapper()->name() == "random") {
    rapid::perception::RandomHeatMapper* mapper =
        static_cast<rapid::perception::RandomHeatMapper*>(
            estimators_->custom->heat_mapper());
    mapper->set_sample_ratio(sample_ratio);
    mapper->set_max_samples(max_samples);
  }
  estimators_->custom->set_num_candidates(num_candidates);
  estimators_->custom->set_fitness_threshold(fitness_threshold);
  estimators_->custom->set_sigma_threshold(sigma_threshold);
  estimators_->custom->set_nms_radius(nms_radius);
  estimators_->custom->set_min_results(min_results);
}

void RunCommand::UpdateRansacParams() {
  double pair_width;
  double scene_radius;
  double object_radius;
  double ransac_threshold;
  ros::param::param<double>("pair_width", pair_width, 0.025);
  ros::param::param<double>("scene_radius", scene_radius, 0.025);
  ros::param::param<double>("object_radius", object_radius, 0.01);
  ros::param::param<double>("ransac_threshold", ransac_threshold, 0.3);
  ROS_INFO(
      "Parameters:\n"
      "pair_width: %f\n"
      "scene_radius: %f\n"
      "object_radius: %f\n"
      "ransac_threshold: %f\n",
      pair_width, scene_radius, object_radius, ransac_threshold);

  if (estimators_->ransac->pair_width() != pair_width) {
    estimators_->ransac->set_pair_width(pair_width);
  }
  estimators_->ransac->set_scene_normal_radius(scene_radius);
  estimators_->ransac->set_object_normal_radius(object_radius);
  estimators_->ransac->set_threshold(ransac_threshold);
}

void RunCommand::UpdateGroupingParams() {
  int normal_k;
  double shot_radius;
  double object_vox;
  double scene_vox;
  double corr_match_threshold;
  bool use_hough;
  double rf_radius;
  double cg_size;
  double cg_threshold;

  ros::param::param<int>("normal_k", normal_k,
                         GroupingPoseEstimator::kDefaultNormalK);
  ros::param::param<double>("shot_radius", shot_radius,
                            GroupingPoseEstimator::kDefaultShotRadius);
  ros::param::param<double>("object_vox", object_vox,
                            GroupingPoseEstimator::kDefaultObjectVox);
  ros::param::param<double>("scene_vox", scene_vox,
                            GroupingPoseEstimator::kDefaultSceneVox);
  ros::param::param<double>("corr_match_threshold", corr_match_threshold,
                            GroupingPoseEstimator::kDefaultCorrMatchThreshold);
  ros::param::param<bool>("use_hough", use_hough,
                          GroupingPoseEstimator::kDefaultUseHough);
  ros::param::param<double>("rf_radius", rf_radius,
                            GroupingPoseEstimator::kDefaultRfRadius);
  ros::param::param<double>("cg_size", cg_size,
                            GroupingPoseEstimator::kDefaultCgSize);
  ros::param::param<double>("cg_threshold", cg_threshold,
                            GroupingPoseEstimator::kDefaultCgThreshold);

  ROS_INFO(
      "Parameters:\n"
      "normal_k: %d\n"
      "shot_radius: %f\n"
      "object_vox: %f\n"
      "scene_vox: %f\n"
      "corr_match_threshold: %f\n"
      "use_hough: %d\n"
      "rf_radius: %f\n"
      "cg_size: %f\n"
      "cg_threshold: %f\n",
      normal_k, shot_radius, object_vox, scene_vox, corr_match_threshold,
      use_hough, rf_radius, cg_size, cg_threshold);

  estimators_->grouping->normal_k_ = normal_k;
  estimators_->grouping->shot_radius_ = shot_radius;
  estimators_->grouping->object_vox_ = object_vox;
  estimators_->grouping->scene_vox_ = scene_vox;
  estimators_->grouping->corr_match_threshold_ = corr_match_threshold;
  estimators_->grouping->use_hough_ = use_hough;
  estimators_->grouping->rf_radius_ = rf_radius;
  estimators_->grouping->cg_size_ = cg_size;
  estimators_->grouping->cg_threshold_ = cg_threshold;
}

SetDebugCommand::SetDebugCommand(Estimators* estimators)
    : estimators_(estimators) {}

void SetDebugCommand::Execute(std::vector<std::string>& args) {
  if (args[0] == "on") {
    estimators_->custom->set_debug(true);
  } else {
    estimators_->custom->set_debug(false);
  }
}
}  // namespace object_search
