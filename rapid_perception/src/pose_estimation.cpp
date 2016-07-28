#include "rapid_perception/pose_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <vector>

#include "Eigen/Core"
#include "cv_bridge/cv_bridge.h"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/common/time.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/random_sample.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/ia_ransac.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"

#include "rapid_perception/cloud_projection.h"
#include "rapid_perception/image_recognition.h"
#include "rapid_utils/stochastic_universal_sampling.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZRGBNormal PointN;
typedef pcl::PointXYZ PointP;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::PointCloud<PointP> PointCloudP;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<FPFH> PointCloudF;
typedef pcl::search::KdTree<FPFH> FeatureTree;
typedef pcl::search::KdTree<PointC> PointCTree;
typedef pcl::search::KdTree<PointN> PointNTree;
typedef pcl::search::KdTree<PointP> PointPTree;
using std::string;
using std::vector;

namespace rapid {
namespace perception {
PoseEstimationHeatMapper::PoseEstimationHeatMapper()
    : debug_(false), heatmap_pub_() {}
void PoseEstimationHeatMapper::set_debug(bool val) { debug_ = val; }
void PoseEstimationHeatMapper::set_heatmap_publisher(
    const ros::Publisher& pub) {
  heatmap_pub_ = pub;
}

CnnHeatMapper::CnnHeatMapper()
    : scene_(new PointCloudC()),
      scene_camera_(new PointCloudC()),
      scene_tree_(),
      object_(new PointCloudC()),
      object_camera_(new PointCloudC()),
      object_radius_est_(0),
      object_image_(),
      image_recognizer_(),
      sample_ratio_(0.01),
      max_samples_(2000),
      max_sample_radius_(0.05),
      max_neighbors_(400),
      cnn_layer_("fc6"),
      landmark_image_pub_(),
      scene_image_pub_() {}

void CnnHeatMapper::Compute(pcl::PointIndicesPtr indices,
                            Eigen::VectorXd* importances) {
  // Sample points in the scene.
  pcl::RandomSample<PointC> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  // For each sample, look at the points around it, project the image into 2D,
  // and use CNN features to compare to the landmark.
  double search_radius = std::min(max_sample_radius_, object_radius_est_);
  importances->resize(indices->indices.size());
  pcl::PointIndicesPtr k_indices_ptr(new pcl::PointIndices);
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    vector<int> k_indices;
    vector<float> k_distances;
    // scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
    //                          max_neighbors_);
    scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
                              object_->size());
    k_indices_ptr->indices = k_indices;
    cv::Mat scene_image;
    CloudToImage(HeadMountKinectCameraInfo(), *scene_camera_, k_indices_ptr,
                 &scene_image);
    PublishImage(scene_image_pub_, scene_camera_->header, scene_image);

    // Paste the landmark into the scene
    // Resize landmark image if needed
    // if (debug_) {
    //  ROS_INFO("Scene image size: %d %d, landmark size: %d %d",
    //           scene_image.cols, scene_image.rows, object_image_.cols,
    //           object_image_.rows);
    //}
    double width_ratio =
        object_image_.cols / static_cast<double>(scene_image.cols);
    double height_ratio =
        object_image_.rows / static_cast<double>(scene_image.rows);
    double max_ratio = std::max(width_ratio, height_ratio);
    cv::Mat object_resized;
    if (max_ratio > 1) {
      double ratio = 1 / max_ratio;
      cv::resize(object_image_, object_resized, cv::Size(), ratio, ratio);
    } else {
      object_resized = object_image_;
    }
    // Paste into the center
    cv::Mat landmark_pasted = scene_image.clone();
    int row_offset = (landmark_pasted.rows - object_resized.rows) / 2;
    int col_offset = (landmark_pasted.cols - object_resized.cols) / 2;
    // if (debug_) {
    //  ROS_INFO("Object resized to: %d %d", object_resized.cols,
    //           object_resized.rows);
    //  ROS_INFO("Will be pasted into scene image with top left col: %d row:
    //  %d",
    //           col_offset, row_offset);
    //}
    for (int row = 0; row < object_resized.rows; ++row) {
      uchar* obj_p = object_resized.ptr<uchar>(row);
      uchar* pasted_p = landmark_pasted.ptr<uchar>(row + row_offset);
      for (int col = 0; col < object_resized.cols; ++col) {
        bool is_black = true;
        for (int color = 0; color < 3; ++color) {
          if (obj_p[3 * col + color] != 0) {
            is_black = false;
            break;
          }
        }
        if (!is_black) {
          for (int color = 0; color < 3; ++color) {
            pasted_p[3 * (col + col_offset) + color] = obj_p[3 * col + color];
          }
        }
      }
    }
    PublishImage(landmark_image_pub_, object_camera_->header, landmark_pasted);

    image_recognizer_.set_image(landmark_pasted);
    string error;
    cv::Mat pasted_features = image_recognizer_.layer(cnn_layer_, &error);
    if (error != "") {
      ROS_ERROR("Error getting object image: %s", error.c_str());
    }

    image_recognizer_.set_image(scene_image);
    error = "";
    cv::Mat scene_features = image_recognizer_.layer(cnn_layer_, &error);
    if (error != "") {
      ROS_ERROR("Error getting scene image: %s", error.c_str());
    }
    double cosine_sim = scene_features.dot(pasted_features) /
                        (cv::norm(scene_features) * cv::norm(pasted_features));
    (*importances)(indices_i) = cosine_sim;
    if (debug_) {
      PointCloudC::Ptr debug(new PointCloudC);
      const PointC& current = scene_->at(index);
      PointC current_copy = current;
      current_copy.r = 255;
      current_copy.g = 0;
      current_copy.b = 0;
      debug->push_back(current_copy);
      debug->header.frame_id = scene_->header.frame_id;
      PublishCloud(heatmap_pub_, *debug);
      std::cout << "# scene points: " << k_indices.size()
                << ", cosine sim: " << cosine_sim;
      string input;
      std::getline(std::cin, input);
    }
  }

  double min = importances->minCoeff();
  if (min < 0) {
    ROS_ERROR("Min cosine distance was negative");
  }
  (*importances) = (importances->array() - min).matrix();
  double max = importances->maxCoeff();
  (*importances) /= max;

  // Do softmax on top of normalizing between 0 and 1
  (*importances) = importances->array().exp().matrix();

  // Color point cloud for visualization
  PointCloudC::Ptr working_scene(new PointCloudC);
  *working_scene = *scene_;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * (*importances)(indices_i)));
    int index = indices->indices[indices_i];
    working_scene->points[index].r = color;
    working_scene->points[index].g = color;
    working_scene->points[index].b = color;
  }
  PointCloudC::Ptr viz_cloud(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(working_scene);
  extract.setIndices(indices);
  extract.filter(*viz_cloud);
  PublishCloud(heatmap_pub_, *viz_cloud);
}

void CnnHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  scene_tree_.reset(new PointCTree());
  scene_tree_->setInputCloud(scene_);
}

void CnnHeatMapper::set_scene_camera(PointCloudC::Ptr scene) {
  scene_camera_ = scene;
}

void CnnHeatMapper::set_object(PointCloudC::Ptr object) {
  object_ = object;

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  double x = max_pt.x() - min_pt.x();
  double y = max_pt.y() - min_pt.y();
  double z = max_pt.z() - min_pt.z();
  object_radius_est_ = std::max(x, std::max(y, z));
}

void CnnHeatMapper::set_object_camera(PointCloudC::Ptr object) {
  object_camera_ = object;
  CloudToImage(HeadMountKinectCameraInfo(), *object_camera_, &object_image_);
  PublishImage(landmark_image_pub_, object_camera_->header, object_image_);
}

void CnnHeatMapper::set_image_recognizer(const ImageRecognizer& val) {
  image_recognizer_ = val;
}

void CnnHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }
void CnnHeatMapper::set_max_samples(int val) { max_samples_ = val; }
void CnnHeatMapper::set_max_sample_radius(double val) {
  max_sample_radius_ = val;
}
void CnnHeatMapper::set_max_neighbors(int val) { max_neighbors_ = val; }
void CnnHeatMapper::set_cnn_layer(string val) { cnn_layer_ = val; }

void CnnHeatMapper::set_landmark_image_publisher(const ros::Publisher& pub) {
  landmark_image_pub_ = pub;
}
void CnnHeatMapper::set_scene_image_publisher(const ros::Publisher& pub) {
  scene_image_pub_ = pub;
}

FpfhHeatMapper::FpfhHeatMapper()
    : scene_(new PointCloudN()),
      scene_tree_(),
      scene_features_(),
      object_(new PointCloudN()),
      object_radius_est_(0),
      object_features_(),
      object_features_tree_(),
      normal_radius_(0.03),
      feature_radius_(0.04),
      sample_ratio_(0.01),
      max_samples_(2000),
      max_sample_radius_(0.05),
      max_neighbors_(400),
      feature_threshold_(1500) {}

void FpfhHeatMapper::Compute(pcl::PointIndicesPtr indices,
                             Eigen::VectorXd* importances) {
  // Sample points in the scene.
  pcl::RandomSample<PointN> random;
  random.setInputCloud(scene_);
  int num_samples = static_cast<int>(round(sample_ratio_ * scene_->size()));
  random.setSample(std::min(max_samples_, num_samples));
  random.filter(indices->indices);
  ROS_INFO("Randomly sampled %ld points", indices->indices.size());

  // For each sample, look at the points around it, project the image into 2D,
  // and use CNN features to compare to the landmark.
  double search_radius = std::min(max_sample_radius_, object_radius_est_);
  // For each scene point, caches whether there is a close match on the object.
  // 0 = unknown, 1 = yes, -1 = no
  vector<int8_t> importance_cache(scene_->size(), 0);
  importances->resize(indices->indices.size());
  pcl::PointIndicesPtr k_indices_ptr(new pcl::PointIndices);
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int index = indices->indices[indices_i];
    vector<int> k_indices;
    vector<float> k_distances;
    scene_tree_->radiusSearch(index, search_radius, k_indices, k_distances,
                              max_neighbors_);
    k_indices_ptr->indices = k_indices;

    double avg_close_matches = 0;
    for (size_t k = 0; k < k_indices.size(); ++k) {
      int k_index = k_indices[k];
      if (importance_cache[k_index] == 1) {
        avg_close_matches += 1;
        continue;
      } else if (importance_cache[k_index] == -1) {
        continue;
      }
      const FPFH& feature = scene_features_->at(k_index);
      vector<int> nearest_indices;
      vector<float> nearest_distances;
      object_features_tree_->nearestKSearch(feature, 1, nearest_indices,
                                            nearest_distances);
      if (nearest_distances[0] < feature_threshold_) {
        avg_close_matches += 1;
        importance_cache[k_index] = 1;
      } else {
        importance_cache[k_index] = -1;
      }
    }
    avg_close_matches /= k_indices.size();
    (*importances)(indices_i) = avg_close_matches;
  }

  double min = importances->minCoeff();
  if (min < 0) {
    ROS_ERROR("Min cosine distance was negative");
  }
  (*importances) = (importances->array() - min).matrix();
  double max = importances->maxCoeff();
  (*importances) /= max;

  // Do softmax on top of normalizing between 0 and 1
  (*importances) = importances->array().exp().matrix();

  // Color point cloud for visualization
  PointCloudN::Ptr working_scene(new PointCloudN);
  *working_scene = *scene_;
  for (size_t indices_i = 0; indices_i < indices->indices.size(); ++indices_i) {
    int color = static_cast<int>(round(255 * (*importances)(indices_i)));
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
}

void FpfhHeatMapper::set_scene(PointCloudC::Ptr scene) {
  scene_.reset(new PointCloudN);
  pcl::copyPointCloud(*scene, *scene_);
  scene_tree_.reset(new PointNTree());
  scene_tree_->setInputCloud(scene_);
  ComputeNormals(scene_);
  ComputeFeatures(scene_, scene_features_);
}

void FpfhHeatMapper::set_object(PointCloudC::Ptr object) {
  object_.reset(new PointCloudN);
  pcl::copyPointCloud(*object, *object_);

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  double x = max_pt.x() - min_pt.x();
  double y = max_pt.y() - min_pt.y();
  double z = max_pt.z() - min_pt.z();
  object_radius_est_ = (x + y + z) / 3;

  ComputeNormals(object_);
  ComputeFeatures(object_, object_features_);
  object_features_tree_.reset(new FeatureTree());
  object_features_tree_->setInputCloud(object_features_);
}

void FpfhHeatMapper::set_sample_ratio(double val) { sample_ratio_ = val; }
void FpfhHeatMapper::set_max_samples(int val) { max_samples_ = val; }
void FpfhHeatMapper::set_max_sample_radius(double val) {
  max_sample_radius_ = val;
}
void FpfhHeatMapper::set_max_neighbors(int val) { max_neighbors_ = val; }
void FpfhHeatMapper::set_feature_threshold(double val) {
  feature_threshold_ = val;
}

void FpfhHeatMapper::ComputeNormals(PointCloudN::Ptr cloud) {
  double normal_radius;
  ros::param::param<double>("normal_radius", normal_radius, 0.03);
  ROS_INFO("Computing normals with radius: %f", normal_radius);

  pcl::ScopeTime normal_timer("Computing normals");
  pcl::NormalEstimationOMP<PointN, PointN> nest;
  nest.setRadiusSearch(normal_radius);
  nest.setInputCloud(cloud);
  nest.compute(*cloud);
}

void FpfhHeatMapper::ComputeFeatures(PointCloudN::Ptr in,
                                     PointCloudF::Ptr out) {
  double feature_radius;
  ros::param::param<double>("feature_radius", feature_radius, 0.04);
  ROS_INFO("Computing features with radius: %f", feature_radius);

  pcl::ScopeTime fest_time("Computing features");
  pcl::FPFHEstimationOMP<PointN, PointN, FPFH> fest;
  fest.setRadiusSearch(feature_radius);
  fest.setInputCloud(in);
  fest.setInputNormals(in);
  fest.compute(*out);
}

PoseEstimationMatch::PoseEstimationMatch(PointCloudC::Ptr cloud, double fitness)
    : cloud_(new PointCloudC), fitness_(fitness) {
  *cloud_ = *cloud;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  center_.x = min_pt.x() + (max_pt.x() - min_pt.x()) / 2;
  center_.y = min_pt.y() + (max_pt.y() - min_pt.y()) / 2;
  center_.z = min_pt.z() + (max_pt.z() - min_pt.z()) / 2;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PoseEstimationMatch::cloud() {
  return cloud_;
}

pcl::PointXYZ PoseEstimationMatch::center() const { return center_; }

double PoseEstimationMatch::fitness() const { return fitness_; }

void PoseEstimationMatch::set_fitness(double fitness) { fitness_ = fitness; }

PoseEstimator::PoseEstimator()
    : scene_(new PointCloudC()),
      object_(new PointCloudC()),
      object_center_(),
      num_candidates_(100),
      fitness_threshold_(0.00002),
      nms_radius_(0.03),
      debug_(false),
      candidates_pub_(),
      alignment_pub_(),
      output_pub_() {}

void PoseEstimator::set_heat_mapper(PoseEstimationHeatMapper* heat_mapper) {
  heat_mapper_ = heat_mapper;
}

void PoseEstimator::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  heat_mapper_->set_scene(scene);
}

void PoseEstimator::set_object(PointCloudC::Ptr object) {
  object_ = object;

  // Estimate object radius, just average of x/y/z dimensions.
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;

  heat_mapper_->set_object(object);
}

PoseEstimationHeatMapper* PoseEstimator::heat_mapper() { return heat_mapper_; }

void PoseEstimator::set_debug(bool val) {
  debug_ = val;
  heat_mapper_->set_debug(val);
}
void PoseEstimator::set_num_candidates(int val) { num_candidates_ = val; }
void PoseEstimator::set_fitness_threshold(double val) {
  fitness_threshold_ = val;
}
void PoseEstimator::set_nms_radius(double val) { nms_radius_ = val; }

void PoseEstimator::set_candidates_publisher(const ros::Publisher& pub) {
  candidates_pub_ = pub;
}
void PoseEstimator::set_alignment_publisher(const ros::Publisher& pub) {
  alignment_pub_ = pub;
}
void PoseEstimator::set_output_publisher(const ros::Publisher& pub) {
  output_pub_ = pub;
}

bool PoseEstimator::Find() {
  // Compute heatmap of features
  pcl::PointIndicesPtr heatmap_indices(new pcl::PointIndices);
  Eigen::VectorXd importances;
  heat_mapper_->Compute(heatmap_indices, &importances);

  // Sample candidate points from the heatmap
  pcl::PointIndices::Ptr candidate_indices(new pcl::PointIndices);
  ComputeCandidates(importances, heatmap_indices, candidate_indices);

  // For each candidate point, run ICP
  vector<PoseEstimationMatch> output_objects;
  RunIcpCandidates(candidate_indices, &output_objects);
  if (output_objects.size() == 0) {
    ROS_WARN("No matches found.");
    return false;
  }

  // Non-max suppression
  vector<bool> keep;
  NonMaxSuppression(output_objects, &keep);

  PointCloudC::Ptr output_cloud(new PointCloudC);
  int num_instances = 0;
  for (size_t i = 0; i < output_objects.size(); ++i) {
    if (keep[i]) {
      double r = static_cast<double>(rand()) / RAND_MAX;
      double g = static_cast<double>(rand()) / RAND_MAX;
      double b = static_cast<double>(rand()) / RAND_MAX;
      Colorize(output_objects[i].cloud(), r, g, b);
      *output_cloud += *output_objects[i].cloud();
      ++num_instances;
    }
  }
  ROS_INFO("Found %d instances of the model", num_instances);
  output_cloud->header.frame_id = scene_->header.frame_id;
  PublishCloud(output_pub_, *output_cloud);
  return true;
}

void PoseEstimator::ComputeCandidates(Eigen::VectorXd& importances,
                                      pcl::PointIndicesPtr heatmap_indices,
                                      pcl::PointIndicesPtr candidate_indices) {
  // Sample points based on importance
  double sum = importances.sum();
  importances /= sum;

  vector<double> cdf;
  double cdf_val = 0;
  for (size_t indices_i = 0; indices_i < heatmap_indices->indices.size();
       ++indices_i) {
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

  for (size_t i = 0; i < sampled_important_indices.size(); ++i) {
    int index_into_indices = sampled_important_indices[i];
    int point_index = heatmap_indices->indices[index_into_indices];
    candidate_indices->indices.push_back(point_index);
  }

  // Visualize the candidate points
  PointCloudC::Ptr viz_cloud(new PointCloudC());
  viz_cloud.reset(new PointCloudC());
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(scene_);
  extract.setIndices(candidate_indices);
  extract.filter(*viz_cloud);
  PublishCloud(candidates_pub_, *viz_cloud);
}

void PoseEstimator::RunIcpCandidates(
    pcl::PointIndices::Ptr candidate_indices,
    vector<PoseEstimationMatch>* output_objects) {
  // Copy the object and its center.
  PointCloudC::Ptr working_object(new PointCloudC);
  PointCloudC::Ptr aligned_object(new PointCloudC);
  pcl::IterativeClosestPoint<PointC, PointC> icp;
  icp.setInputTarget(scene_);
  output_objects->clear();
  for (size_t ci = 0; ci < candidate_indices->indices.size(); ++ci) {
    int index = candidate_indices->indices[ci];
    const PointC& center = scene_->points[index];
    // Transform the object to be centered on the center point
    // So far we just use a translation offset
    // Also assume that the object and scene are in the same frame
    Eigen::Vector3d translation;
    translation.x() = center.x - object_center_.x();
    translation.y() = center.y - object_center_.y();
    translation.z() = center.z - object_center_.z();
    pcl::transformPointCloud(*object_, *working_object, translation,
                             Eigen::Quaterniond::Identity());

    // Run ICP
    icp.setInputSource(working_object);
    icp.align(*aligned_object);
    PublishCloud(alignment_pub_, *aligned_object);
    if (icp.hasConverged()) {
      double fitness = icp.getFitnessScore();
      string threshold_marker = "";
      if (fitness < fitness_threshold_) {
        output_objects->push_back(PoseEstimationMatch(aligned_object, fitness));
        threshold_marker = " -- below threshold";
      }
      ROS_INFO("ICP converged with score: %f%s", fitness,
               threshold_marker.c_str());
    }
    if (debug_) {
      std::cout << "Press enter to continue ";
      string user_input;
      std::getline(std::cin, user_input);
    }
  }
}

void PoseEstimator::NonMaxSuppression(
    vector<PoseEstimationMatch>& output_objects, vector<bool>* keep) {
  // Non-max supression of output objects
  // Index centers into a tree
  PointCloudP::Ptr output_centers(new PointCloudP);
  for (size_t i = 0; i < output_objects.size(); ++i) {
    output_centers->push_back(output_objects[i].center());
  }
  PointPTree output_centers_tree;
  output_centers_tree.setInputCloud(output_centers);

  // Suppress non max within a radius
  for (int i = 0; i < static_cast<int>(output_objects.size()); ++i) {
    PoseEstimationMatch& match = output_objects[i];
    vector<int> neighbor_indices;
    vector<float> neighbor_distances;
    output_centers_tree.radiusSearch(match.center(), nms_radius_,
                                     neighbor_indices, neighbor_distances);

    bool keep_match = true;
    for (size_t k = 0; k < neighbor_indices.size(); ++k) {
      int index = neighbor_indices[k];
      if (index == i) {
        continue;
      }
      PoseEstimationMatch& neighbor = output_objects[index];
      if (neighbor.fitness() < match.fitness()) {
        if (debug_) {
          ROS_INFO("Suppressing match %d, %f >= neighbor %d, %f", i,
                   match.fitness(), index, neighbor.fitness());
        }
        keep_match = false;
        break;
      } else if (neighbor.fitness() == match.fitness()) {
        // Keep this one, bump neighbor's fitness score up so that it won't come
        // up again.
        neighbor.set_fitness(neighbor.fitness() * 1.01);
      }
    }
    keep->push_back(keep_match);
  }
}

string FpfhFeatureString(const FPFH& feature) {
  std::stringstream ss;
  for (int i = 0; i < 33; ++i) {
    ss << feature.histogram[i];
    if (i != 32) {
      ss << ", ";
    }
  }
  return ss.str();
}

void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->points[i].r = static_cast<int>(round(r * 255));
    cloud->points[i].g = static_cast<int>(round(g * 255));
    cloud->points[i].b = static_cast<int>(round(b * 255));
  }
}

void PublishCloud(const ros::Publisher& pub, PointCloudC& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }
}

void PublishCloud(const ros::Publisher& pub, PointCloudN& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }
}

void PublishImage(const ros::Publisher& pub, const pcl::PCLHeader& cloud_header,
                  const cv::Mat& mat) {
  std_msgs::Header header;
  header.frame_id = cloud_header.frame_id;
  cv_bridge::CvImage cv_bridge(header, sensor_msgs::image_encodings::TYPE_8UC3,
                               mat);
  sensor_msgs::Image image_msg;
  cv_bridge.toImageMsg(image_msg);
  pub.publish(image_msg);
}
}  // namespace perception
}  // namespace rapid
