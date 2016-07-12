#include <iostream>
#include <string>
#include <vector>

#include "pbd_landmarks/GetLandmark.h"
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
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

using pbd_landmarks::Landmark;
using sensor_msgs::PointCloud2;
using pcl::FPFHSignature33;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using pcl::PointXYZRGBNormal;

bool GetLandmark(const std::string& id, Landmark* landmark);
bool FindLandmark(PointCloud<PointXYZRGB>::Ptr cloud,
                  PointCloud<PointXYZRGB>::Ptr landmark,
                  PointCloud<PointXYZRGB>::Ptr found_landmark);
void TransformToBase(const PointCloud2& cloud_in, PointCloud2* cloud_out);

void TransformToBase(const tf::TransformListener& tf_listener,
                     const PointCloud2& cloud_in, PointCloud2* cloud_out) {
  while (true) {
    try {
      bool success = pcl_ros::transformPointCloud("/base_footprint", cloud_in,
                                                  *cloud_out, tf_listener);
      if (success) {
        return;
      } else {
        ROS_ERROR_THROTTLE(1, "Failed to transform point cloud");
      }
    } catch (tf::TransformException e) {
      ROS_WARN_THROTTLE(1, "Trying again: %s", e.what());
    }
    ros::Duration(1).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search");
  ros::NodeHandle nh;
  tf::TransformListener tf_listener;
  ros::Duration(2).sleep();

  Landmark landmark;
  bool success = GetLandmark("577dca1e5163504a3d47a495", &landmark);
  if (!success) {
    return 1;
  }
  PointCloud2 landmark_msg = landmark.cloud;
  landmark_msg.header.stamp = ros::Time::now();
  PointCloud2 transformed_landmark;
  TransformToBase(tf_listener, landmark_msg, &transformed_landmark);
  PointCloud<PointXYZRGB>::Ptr landmark_cloud(new PointCloud<PointXYZRGB>());
  while (true) {
    pcl::fromROSMsg(transformed_landmark, *landmark_cloud);

    PointCloud2::ConstPtr pc =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cloud_in", nh,
                                                             ros::Duration(10));
    sensor_msgs::PointCloud2 transformed_scene;
    TransformToBase(tf_listener, *pc, &transformed_scene);
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    pcl::fromROSMsg(transformed_scene, *cloud);
    ROS_INFO("Loaded scene with %ld points", cloud->size());

    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param<double>("min_x", min_x, 0.2);
    ros::param::param<double>("min_y", min_y, -1);
    ros::param::param<double>("min_z", min_z, 0.3);
    ros::param::param<double>("max_x", max_x, 1.2);
    ros::param::param<double>("max_y", max_y, 1);
    ros::param::param<double>("max_z", max_z, 1.7);
    ROS_INFO(
        "Cropping:"
        "  min_x: %f\n"
        "  min_y: %f\n"
        "  min_z: %f\n"
        "  max_x: %f\n"
        "  max_y: %f\n"
        "  max_z: %f\n",
        min_x, min_y, min_z, max_x, max_y, max_z);

    pcl::CropBox<PointXYZRGB> crop;
    crop.setInputCloud(cloud);
    Eigen::Vector4f min;
    min << min_x, min_y, min_z, 1;
    Eigen::Vector4f max;
    max << max_x, max_y, max_z, 1;
    crop.setMin(min);
    crop.setMax(max);
    crop.filter(*cloud);
    ROS_INFO("Cropped to %ld points", cloud->size());

    ros::Publisher found_pub =
        nh.advertise<PointCloud2>("/found_landmark", 1, true);
    PointCloud<PointXYZRGB>::Ptr found_landmark(new PointCloud<PointXYZRGB>());
    bool found = FindLandmark(cloud, landmark_cloud, found_landmark);
    if (!found) {
      ROS_ERROR("Unable to find landmark in current scene.");
    } else {
      sensor_msgs::PointCloud2 found_cloud;
      pcl::toROSMsg(*found_landmark, found_cloud);
      found_cloud.header.stamp = ros::Time::now();
      found_pub.publish(found_cloud);
    }
    std::cout << "Press enter to try again. ";
    std::string enter;
    std::getline(std::cin, enter);
  }

  ros::spin();
  return 0;
}

bool GetLandmark(const std::string& id, Landmark* landmark) {
  pbd_landmarks::GetLandmarkRequest req;
  req.id = "577dca1e5163504a3d47a495";
  pbd_landmarks::GetLandmarkResponse res;
  bool exists = ros::service::exists("get_landmark", true);
  if (!exists) {
    ROS_ERROR("GetLandmark service does not exist.");
    return false;
  }
  bool success = ros::service::call("get_landmark", req, res);
  if (!success) {
    ROS_ERROR("GetLandmark call failed.");
    return false;
  }
  if (res.error != "") {
    ROS_ERROR("GetLandmark call failed: %s", res.error.c_str());
    return false;
  }
  *landmark = res.landmark;
  return true;
}

bool FindLandmark(PointCloud<PointXYZRGB>::Ptr scene,
                  PointCloud<PointXYZRGB>::Ptr landmark,
                  PointCloud<PointXYZRGB>::Ptr found_landmark) {
  // Normal estimation
  double normal_radius;
  ros::param::param<double>("normal_radius", normal_radius, 0.01);
  // Features
  double feature_radius;
  ros::param::param<double>("feature_radius", feature_radius, 0.025);
  // Alignment
  int max_iterations;
  ros::param::param<int>("iterations", max_iterations, 50000);
  int num_samples;
  ros::param::param<int>("num_samples", num_samples, 3);
  int randomness;
  ros::param::param<int>("randomness", randomness, 5);
  double sim_threshold;
  ros::param::param<double>("sim_threshold", sim_threshold, 0.9);
  double distance_threshold;
  ros::param::param<double>("distance_threshold", distance_threshold, 0.01);
  double inlier_fraction;
  ros::param::param<double>("inlier_fraction", inlier_fraction, 0.25);
  double landmark_sample;
  ros::param::param<double>("landmark_sample", landmark_sample, 1);
  double scene_sample;
  ros::param::param<double>("scene_sample", scene_sample, 1);
  ROS_INFO(
      "Parameters:\n"
      "normal_radius: %f\n"
      "feature_radius: %f\n"
      "iterations: %d\n"
      "num_samples: %d\n"
      "randomness: %d\n"
      "sim_threshold: %f\n"
      "distance_threshold: %f\n"
      "inlier_fraction: %f\n",
      normal_radius, feature_radius, max_iterations, num_samples, randomness,
      sim_threshold, distance_threshold, inlier_fraction);

  ros::NodeHandle nh;
  ros::Publisher scene_pub = nh.advertise<PointCloud2>("/scene", 1, true);
  ros::Publisher landmark_pub = nh.advertise<PointCloud2>("/landmark", 1, true);

  // Filter NaNs
  std::vector<int> mapping;
  scene->is_dense = false;  // Force check for NaNs
  pcl::removeNaNFromPointCloud(*scene, *scene, mapping);
  ROS_INFO("Filtered NaNs: %ld", scene->size());

  // Voxelize scene and landmark.
  // ROS_INFO("scene: %ld, landmark: %ld", scene->size(), landmark->size());
  // const double kLeafSize = 0.005;
  // pcl::VoxelGrid<PointXYZRGB> grid;
  // grid.setLeafSize(kLeafSize, kLeafSize, kLeafSize);
  // grid.setInputCloud(landmark);
  // grid.filter(*landmark);
  // grid.setInputCloud(scene);
  // grid.filter(*scene);
  // ROS_INFO("Downsampled scene: %ld, landmark: %ld", scene->size(),
  //         landmark->size());

  PointCloud2 landmark_msg;
  pcl::toROSMsg(*landmark, landmark_msg);
  landmark_msg.header.stamp = ros::Time::now();
  landmark_pub.publish(landmark_msg);

  // Visualize scene
  PointCloud2 scene_msg;
  pcl::toROSMsg(*scene, scene_msg);
  scene_pub.publish(scene_msg);

  PointCloud<PointXYZRGBNormal>::Ptr scene_normals(
      new PointCloud<PointXYZRGBNormal>);
  PointCloud<PointXYZRGBNormal>::Ptr landmark_normals(
      new PointCloud<PointXYZRGBNormal>);
  pcl::copyPointCloud(*scene, *scene_normals);
  pcl::copyPointCloud(*landmark, *landmark_normals);
  ROS_INFO("Estimating scene normals...");
  pcl::NormalEstimationOMP<PointXYZRGB, PointXYZRGBNormal> nest;
  nest.setRadiusSearch(normal_radius);
  nest.setInputCloud(scene);
  nest.compute(*scene_normals);
  nest.setInputCloud(landmark);
  nest.compute(*landmark_normals);

  // Estimate features
  ROS_INFO("Estimating features...");
  pcl::FPFHEstimationOMP<PointXYZRGBNormal, PointXYZRGBNormal, FPFHSignature33>
      fest;
  fest.setRadiusSearch(feature_radius);
  PointCloud<FPFHSignature33>::Ptr object_features(
      new PointCloud<FPFHSignature33>());
  fest.setInputCloud(landmark_normals);
  fest.setInputNormals(landmark_normals);
  fest.compute(*object_features);
  PointCloud<FPFHSignature33>::Ptr scene_features(
      new PointCloud<FPFHSignature33>());
  fest.setInputCloud(scene_normals);
  fest.setInputNormals(scene_normals);
  fest.compute(*scene_features);

  // Perform alignment
  ROS_INFO("Starting alignment...");
  pcl::SampleConsensusPrerejective<PointXYZRGBNormal, PointXYZRGBNormal,
                                   FPFHSignature33> align;
  align.setInputSource(landmark_normals);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene_normals);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(max_iterations);
  align.setNumberOfSamples(num_samples);
  align.setCorrespondenceRandomness(randomness);
  align.setSimilarityThreshold(sim_threshold);
  align.setMaxCorrespondenceDistance(distance_threshold);
  align.setInlierFraction(inlier_fraction);

  PointCloud<PointXYZRGBNormal>::Ptr found_landmark_normals(
      new PointCloud<PointXYZRGBNormal>);
  {
    pcl::ScopeTime t("Alignment");
    align.align(*found_landmark_normals);
  }

  if (align.hasConverged()) {
    // Print results
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    std::cout << transformation << std::endl;
    ROS_INFO("Inliers: %ld/%ld", align.getInliers().size(), landmark->size());
    pcl::copyPointCloud(*found_landmark_normals, *found_landmark);

    return true;
  } else {
    return false;
  }
}
