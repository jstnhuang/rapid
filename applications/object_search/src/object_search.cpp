#include "object_search/object_search.h"

#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "rapid_perception/grouping_pose_estimator.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_perception/ransac_pose_estimator.h"
#include "ros/ros.h"

using rapid::perception::GroupingPoseEstimator;

namespace object_search {
void UpdateEstimatorParams(rapid::perception::PoseEstimator* custom) {
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

  if (custom->heat_mapper()->name() == "random") {
    rapid::perception::RandomHeatMapper* mapper =
        static_cast<rapid::perception::RandomHeatMapper*>(
            custom->heat_mapper());
    mapper->set_sample_ratio(sample_ratio);
    mapper->set_max_samples(max_samples);
  }
  custom->set_num_candidates(num_candidates);
  custom->set_fitness_threshold(fitness_threshold);
  custom->set_sigma_threshold(sigma_threshold);
  custom->set_nms_radius(nms_radius);
  custom->set_min_results(min_results);
}

void UpdateEstimatorParams(rapid::perception::RansacPoseEstimator* ransac) {
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

  if (ransac->pair_width() != pair_width) {
    ransac->set_pair_width(pair_width);
  }
  ransac->set_scene_normal_radius(scene_radius);
  ransac->set_object_normal_radius(object_radius);
  ransac->set_threshold(ransac_threshold);
}

void UpdateEstimatorParams(GroupingPoseEstimator* grouping) {
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

  grouping->normal_k_ = normal_k;
  grouping->shot_radius_ = shot_radius;
  grouping->object_vox_ = object_vox;
  grouping->scene_vox_ = scene_vox;
  grouping->corr_match_threshold_ = corr_match_threshold;
  grouping->use_hough_ = use_hough;
  grouping->rf_radius_ = rf_radius;
  grouping->cg_size_ = cg_size;
  grouping->cg_threshold_ = cg_threshold;
}

void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped) {
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

  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(scene);
  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMin(min);
  crop.setMax(max);
  crop.filter(*cropped);
  ROS_INFO("Cropped to %ld points", cropped->size());
}

void Downsample(const double leaf_size,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(cloud_in);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*cloud_out);
}
}  // namespace object_search
