#include <iostream>
#include <string>
#include <vector>

#include "pcl/common/time.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/sample_consensus_prerejective.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/box3d_roi_server.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_msgs/GetStaticCloud.h"
#include "rapid_msgs/ListStaticClouds.h"
#include "rapid_msgs/RemoveStaticCloud.h"
#include "rapid_msgs/SaveStaticCloud.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "object_search/capture_roi.h"
#include "object_search/commands.h"
#include "object_search/command_line.h"
#include "object_search/cloud_database.h"

using sensor_msgs::PointCloud2;
using pcl::FPFHSignature33;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointXYZRGBNormal;
using visualization_msgs::Marker;
namespace rp = rapid::perception;

using namespace object_search;

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Build databases
  ros::ServiceClient get_cloud =
      nh.serviceClient<rapid_msgs::GetStaticCloud>("get_static_cloud");
  ros::ServiceClient list_clouds =
      nh.serviceClient<rapid_msgs::ListStaticClouds>("list_static_clouds");
  ros::ServiceClient remove_cloud =
      nh.serviceClient<rapid_msgs::RemoveStaticCloud>("remove_static_cloud");
  ros::ServiceClient save_cloud =
      nh.serviceClient<rapid_msgs::SaveStaticCloud>("save_static_cloud");
  Database object_db("object_search", "objects", get_cloud, list_clouds,
                     remove_cloud, save_cloud);
  Database scene_db("object_search", "scenes", get_cloud, list_clouds,
                    remove_cloud, save_cloud);

  // Visualization publishers
  ros::Publisher object_pub = nh.advertise<PointCloud2>("/landmark", 1, true);
  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
  ros::Publisher heatmap_pub = nh.advertise<PointCloud2>("/heatmap", 1, true);
  ros::Publisher candidates_pub =
      nh.advertise<PointCloud2>("/candidate_samples", 1, true);
  ros::Publisher best_pub =
      nh.advertise<PointCloud2>("/best_alignment", 1, true);

  // Build ROI server
  tf::TransformListener tf_listener;
  rapid::perception::Box3DRoiServer roi_server("roi");
  CaptureRoi capture(&roi_server);

  // Build pose estimator
  rapid::perception::PoseEstimator pose_estimator;

  ListCommand list_objects(&object_db, "object");
  ListCommand list_scenes(&scene_db, "scene");
  RecordObjectCommand record_object(&object_db, &capture);
  RecordSceneCommand record_scene(&scene_db);
  DeleteCommand delete_object(&object_db);
  DeleteCommand delete_scene(&scene_db);
  UseCommand use_object(&object_db, &pose_estimator, "object", object_pub);
  UseCommand use_scene(&scene_db, &pose_estimator, "scene", scene_pub);
  RunCommand run(&pose_estimator, heatmap_pub, candidates_pub, best_pub);
  SetDebugCommand set_debug(&pose_estimator);

  CommandLine cli;
  cli.set_list_objects(&list_objects);
  cli.set_list_scenes(&list_scenes);
  cli.set_record_object(&record_object);
  cli.set_record_scene(&record_scene);
  cli.set_delete_object(&delete_object);
  cli.set_delete_scene(&delete_scene);
  cli.set_use_object(&use_object);
  cli.set_use_scene(&use_scene);
  cli.set_run(&run);
  cli.set_debug(&set_debug);

  while (cli.Next()) {
  }

  spinner.stop();
  return 0;
}

// bool FindLandmark(PointCloud<PointXYZRGB>::Ptr scene,
//                  PointCloud<PointXYZRGB>::Ptr landmark,
//                  PointCloud<PointXYZRGB>::Ptr found_landmark, bool debug) {
//  pcl::ScopeTime t("FindLandmark");
//
//  // Normal estimation
//  double normal_radius;
//  ros::param::param<double>("normal_radius", normal_radius, 0.01);
//  // Features
//  double feature_radius;
//  ros::param::param<double>("feature_radius", feature_radius, 0.045);
//  // Alignment
//  double sample_ratio;
//  int max_samples;
//  double max_sample_radius;
//  int max_neighbors;
//  double feature_threshold;
//  int num_candidates;
//  ros::param::param<double>("sample_ratio", sample_ratio, 0.1);
//  ros::param::param<int>("max_samples", max_samples, 1000);
//  ros::param::param<double>("max_sample_radius", max_sample_radius, 0.1);
//  ros::param::param<int>("max_neighbors", max_neighbors, 400);
//  ros::param::param<double>("feature_threshold", feature_threshold, 1500);
//  ros::param::param<int>("num_candidates", num_candidates, 100);
//  ROS_INFO(
//      "Parameters:\n"
//      "sample_ratio: %f\n"
//      "max_samples: %d\n"
//      "max_sample_radius: %f\n"
//      "max_neighbors: %d\n"
//      "feature_threshold: %f\n"
//      "num_candidates: %d\n",
//      sample_ratio, max_samples, max_sample_radius, max_neighbors,
//      feature_threshold, num_candidates);
//
//  ros::NodeHandle nh;
//  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
//
//  // Filter NaNs
//  std::vector<int> mapping;
//  scene->is_dense = false;  // Force check for NaNs
//  pcl::removeNaNFromPointCloud(*scene, *scene, mapping);
//  ROS_INFO("Filtered NaNs: %ld", scene->size());
//
//  // Visualize scene
//  PointCloud2 scene_msg;
//  pcl::toROSMsg(*scene, scene_msg);
//  scene_pub.publish(scene_msg);
//
//  PointCloud<PointXYZRGBNormal>::Ptr scene_normals(
//      new PointCloud<PointXYZRGBNormal>);
//  PointCloud<PointXYZRGBNormal>::Ptr landmark_normals(
//      new PointCloud<PointXYZRGBNormal>);
//  pcl::copyPointCloud(*scene, *scene_normals);
//  pcl::copyPointCloud(*landmark, *landmark_normals);
//  ROS_INFO("Estimating scene normals...");
//  pcl::NormalEstimationOMP<PointXYZRGB, PointXYZRGBNormal> nest;
//  nest.setRadiusSearch(normal_radius);
//  nest.setInputCloud(scene);
//  nest.compute(*scene_normals);
//  nest.setInputCloud(landmark);
//  {
//    pcl::ScopeTime nest_time("Computing normals");
//    nest.compute(*landmark_normals);
//  }
//
//  // Estimate features
//  ROS_INFO("Estimating features...");
//  pcl::FPFHEstimationOMP<PointXYZRGBNormal, PointXYZRGBNormal,
//  FPFHSignature33>
//      fest;
//  fest.setRadiusSearch(feature_radius);
//  PointCloud<FPFHSignature33>::Ptr object_features(
//      new PointCloud<FPFHSignature33>());
//  fest.setInputCloud(landmark_normals);
//  fest.setInputNormals(landmark_normals);
//  {
//    pcl::ScopeTime fest_time("Estimating landmark features");
//    fest.compute(*object_features);
//  }
//  PointCloud<FPFHSignature33>::Ptr scene_features(
//      new PointCloud<FPFHSignature33>());
//  fest.setInputCloud(scene_normals);
//  fest.setInputNormals(scene_normals);
//  {
//    pcl::ScopeTime fest_time("Estimating scene features");
//    fest.compute(*scene_features);
//  }
//
//  // Perform alignment
//  ROS_INFO("Starting alignment...");
//  rp::PoseEstimator pose_est;
//  pose_est.set_scene(scene_normals);
//  pose_est.set_scene_features(scene_features);
//  pose_est.set_object(landmark_normals);
//  pose_est.set_object_features(object_features);
//  pose_est.set_sample_ratio(sample_ratio);
//  pose_est.set_max_samples(max_samples);
//  pose_est.set_max_sample_radius(max_sample_radius);
//  pose_est.set_max_neighbors(max_neighbors);
//  pose_est.set_feature_threshold(feature_threshold);
//  pose_est.set_num_candidates(num_candidates);
//  pose_est.set_debug(debug);
//
//  // Set up visualization
//  ros::Publisher heatmap_pub = nh.advertise<PointCloud2>("/heatmap", 1, true);
//  pose_est.set_heatmap_publisher(heatmap_pub);
//  ros::Publisher candidates_pub =
//      nh.advertise<PointCloud2>("/candidate_samples", 1, true);
//  pose_est.set_candidates_publisher(candidates_pub);
//  ros::Publisher best_pub =
//      nh.advertise<PointCloud2>("/best_alignment", 1, true);
//  pose_est.set_best_publisher(best_pub);
//  bool found = pose_est.Find();
//  if (found) {
//    return true;
//  } else {
//    return false;
//  }
//}
