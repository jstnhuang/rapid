#include "rapid_perception/pose_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <vector>

#include "pcl/common/common.h"
#include "pcl/common/time.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "Eigen/Core"

#include "rapid_utils/stochastic_universal_sampling.h"

typedef pcl::PointXYZRGBNormal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<FPFH> PointCloudF;
typedef pcl::search::KdTree<FPFH> FeatureTree;
typedef pcl::search::KdTree<PointN> PointTree;
using std::vector;

namespace rapid {
namespace perception {
PoseEstimator::PoseEstimator()
    : scene_(new PointCloudN()),
      scene_tree_(),
      object_(new PointCloudN()),
      object_radius_est_(0),
      object_center_(),
      scene_features_(new PointCloudF()),
      object_features_(new PointCloudF()),
      object_features_tree_(),
      sample_ratio_(0.01),
      max_samples_(2000),
      max_sample_radius_(0.05),
      max_neighbors_(400),
      feature_threshold_(1500),
      num_candidates_(100),
      debug_(false),
      heatmap_pub_(),
      initial_pub_(),
      candidates_pub_(),
      best_pub_() {}

void PoseEstimator::set_scene(PointCloudN::ConstPtr scene) {
  scene_.reset(new PointCloudN());
  *scene_ = *scene;
  scene_tree_.reset(new PointTree());
  scene_tree_->setInputCloud(scene_);
}

void PoseEstimator::set_object(PointCloudN::ConstPtr object) {
  object_.reset(new PointCloudN());
  *object_ = *object;

  // Estimate object radius, just average of x/y/z dimensions.
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  double x = max_pt.x() - min_pt.x();
  double y = max_pt.y() - min_pt.y();
  double z = max_pt.z() - min_pt.z();
  object_radius_est_ = (x + y + z) / 3;
  ROS_INFO("Estimated object radius: %f, dims: %f %f %f", object_radius_est_, x,
           y, z);

  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;
  ROS_INFO("Object centered at: %f %f %f", object_center_.x(),
           object_center_.y(), object_center_.z());
}

void PoseEstimator::set_scene_features(PointCloudF::ConstPtr scene_features) {
  scene_features_.reset(new PointCloudF());
  *scene_features_ = *scene_features;
}

void PoseEstimator::set_object_features(PointCloudF::ConstPtr object_features) {
  object_features_.reset(new PointCloudF());
  *object_features_ = *object_features;
  object_features_tree_.reset(new FeatureTree());
  object_features_tree_->setInputCloud(object_features_);
}

void PoseEstimator::set_sample_ratio(double val) { sample_ratio_ = val; }
void PoseEstimator::set_max_samples(int val) { max_samples_ = val; }
void PoseEstimator::set_max_sample_radius(double val) {
  max_sample_radius_ = val;
}
void PoseEstimator::set_max_neighbors(int val) { max_neighbors_ = val; }
void PoseEstimator::set_feature_threshold(double val) {
  feature_threshold_ = val;
}
void PoseEstimator::set_debug(bool val) { debug_ = val; }
void PoseEstimator::set_num_candidates(int val) { num_candidates_ = val; }

void PoseEstimator::set_heatmap_publisher(const ros::Publisher& pub) {
  heatmap_pub_ = pub;
}
void PoseEstimator::set_candidates_publisher(const ros::Publisher& pub) {
  candidates_pub_ = pub;
}
void PoseEstimator::set_best_publisher(const ros::Publisher& pub) {
  best_pub_ = pub;
}

bool PoseEstimator::Find() {
  PointCloudN::Ptr working_scene(new PointCloudN);
  *working_scene = *scene_;

  // Sample points in the scene.
  pcl::PointIndicesPtr indices(new pcl::PointIndices());
  pcl::RandomSample<PointN> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  // For each sample, look at an area around it and count the number of features
  // in common with the object.
  double search_radius = std::min(max_sample_radius_, object_radius_est_);

  Eigen::VectorXd importances(indices->indices.size());
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    vector<int> k_indices;
    vector<float> k_distances;
    scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
                              max_neighbors_);
    double avg_close_matches = 0;
    for (size_t k = 0; k < k_indices.size(); ++k) {
      int k_index = k_indices[k];
      const FPFH& feature = scene_features_->at(k_index);
      vector<int> nearest_indices;
      vector<float> nearest_distances;
      object_features_tree_->nearestKSearch(feature, 1, nearest_indices,
                                            nearest_distances);
      if (nearest_distances[0] < feature_threshold_) {
        avg_close_matches += 1;
      }
    }
    avg_close_matches /= k_indices.size();
    importances(indices_i) = avg_close_matches;
  }

  double max = importances.maxCoeff();
  importances /= max;

  // Color point cloud for visualization
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * importances(indices_i)));
    int index = indices->indices[indices_i];
    working_scene->points[index].r = color;
    working_scene->points[index].g = color;
    working_scene->points[index].b = color;
  }
  PointCloudN::Ptr viz_cloud(new PointCloudN());
  pcl::ExtractIndices<PointN> extract;
  extract.setInputCloud(working_scene);
  extract.setIndices(indices);
  extract.filter(*viz_cloud);
  PublishCloud(heatmap_pub_, *viz_cloud);

  // Sample points based on importance
  double sum = importances.sum();
  importances /= sum;

  vector<double> cdf;
  double cdf_val = 0;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    double prob = importances(indices_i);
    cdf_val += prob;
    cdf.push_back(cdf_val);
  }

  // Indices of indices! We are sampling the indices vector by importance. The
  // sampled indices themselves are stored in sampled_important_indices, so the
  // actual point index for i in sampled_important_indices is
  // cloud[indices[sampled_important_indices[i]]].
  vector<int> sampled_important_indices;
  utils::StochasticUniversalSampling(cdf, num_candidates_,
                                     &sampled_important_indices);

  pcl::PointIndices::Ptr candidate_indices(new pcl::PointIndices);
  for (size_t i = 0; i < sampled_important_indices.size(); ++i) {
    int index_into_indices = sampled_important_indices[i];
    int point_index = indices->indices[index_into_indices];
    candidate_indices->indices.push_back(point_index);
  }

  // Visualize the candidate points
  viz_cloud.reset(new PointCloudN());
  extract.setInputCloud(working_scene);
  extract.setIndices(candidate_indices);
  extract.filter(*viz_cloud);
  PublishCloud(candidates_pub_, *viz_cloud);

  // For each candidate point, run ICP
  // Copy the object and its center.
  PointCloudN::Ptr working_object(new PointCloudN);
  Eigen::Quaterniond identity_rotation = Eigen::Quaterniond::Identity();
  double best_score = std::numeric_limits<double>::max();
  std::string user_input;
  PointCloudN::Ptr best_alignment(new PointCloudN);
  pcl::IterativeClosestPoint<PointN, PointN> icp;
  icp.setInputTarget(scene_);
  PointCloudN::Ptr aligned_object(new PointCloudN);
  for (size_t ci = 0; ci < candidate_indices->indices.size(); ++ci) {
    int index = candidate_indices->indices[ci];
    const PointN& center = working_scene->points[index];
    // Transform the object to be centered on the center point
    // So far we just use a translation offset
    // Also assume that the object and scene are in the same frame
    Eigen::Vector3d translation;
    translation.x() = center.x - object_center_.x();
    translation.y() = center.y - object_center_.y();
    translation.z() = center.z - object_center_.z();

    pcl::transformPointCloud(*object_, *working_object, translation,
                             identity_rotation);
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*working_object, min_pt, max_pt);
    PublishCloud(best_pub_, *working_object);
    if (debug_) {
      std::cout << "ICP initialized, press enter to align ";
      std::getline(std::cin, user_input);
    }

    // Run ICP
    icp.setInputSource(working_object);
    icp.align(*aligned_object);
    PublishCloud(best_pub_, *aligned_object);
    if (icp.hasConverged()) {
      double fitness = icp.getFitnessScore();
      ROS_INFO("ICP converged with score: %f", fitness);
      if (fitness < best_score) {
        best_score = fitness;
        *best_alignment = *aligned_object;
      }
    }
    if (debug_) {
      std::cout << "Press enter to continue ";
      std::getline(std::cin, user_input);
    }
  }
  if (best_score != -1) {
    ROS_INFO("Best score was: %f", best_score);
    PublishCloud(best_pub_, *best_alignment);
  } else {
    ROS_INFO("No matches found.");
  }
  return true;
}

bool PoseEstimator::Find2() {
  pcl::SampleConsensusInitialAlignment<PointN, PointN, FPFH> initial_align;
  double kMinSampleDistance = 0.02;  // Distance between sample points.
  // Max distance between two corresponding points between source/target.
  double kMaxCorrespondenceDistance = 0.01;
  int kMaxIterations = 20000;
  initial_align.setMinSampleDistance(kMinSampleDistance);
  initial_align.setMaxCorrespondenceDistance(kMaxCorrespondenceDistance);
  initial_align.setMaximumIterations(kMaxIterations);
  initial_align.setInputSource(object_);
  initial_align.setSourceFeatures(object_features_);
  initial_align.setInputTarget(scene_);
  initial_align.setTargetFeatures(scene_features_);
  PointCloudN::Ptr aligned(new PointCloudN);
  ROS_INFO("Starting initial alignment");
  initial_align.align(*aligned);
  ROS_INFO("Done, publishing initial alignment");
  PublishCloud(initial_pub_, *aligned);

  return true;
}

void PoseEstimator::PublishCloud(
    const ros::Publisher& pub, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }
}

std::string PoseEstimator::FeatureString(const FPFH& feature) {
  std::stringstream ss;
  for (int i = 0; i < 33; ++i) {
    ss << feature.histogram[i];
    if (i != 32) {
      ss << ", ";
    }
  }
  return ss.str();
}

void PoseEstimator::set_initial_publisher(const ros::Publisher& pub) {
  initial_pub_ = pub;
}
}  // namespace perception
}  // namespace rapid
