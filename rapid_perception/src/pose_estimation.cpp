#include "rapid_perception/pose_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/icp_fitness_functions.h"
#include "rapid_perception/template_matching_heat_mapper.h"
#include "rapid_utils/eigen_conversions.h"
#include "rapid_utils/pcl_conversions.h"
#include "rapid_utils/stochastic_universal_sampling.h"
#include "rapid_viz/publish.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointP;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointP> PointCloudP;
typedef pcl::FPFHSignature33 FPFH;
typedef pcl::PointCloud<FPFH> PointCloudF;
typedef pcl::search::KdTree<FPFH> FeatureTree;
typedef pcl::search::KdTree<PointC> PointCTree;
typedef pcl::search::KdTree<PointP> PointPTree;
using std::pair;
using std::string;
using std::vector;

namespace rapid {
namespace perception {
PoseEstimationMatch::PoseEstimationMatch()
    : cloud_(new PointCloudC), fitness_(std::numeric_limits<double>::max()) {}

PoseEstimationMatch::PoseEstimationMatch(PointCloudC::Ptr cloud,
                                         const geometry_msgs::Pose& pose,
                                         double fitness)
    : cloud_(new PointCloudC), pose_(pose), fitness_(fitness) {
  *cloud_ = *cloud;
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  center_.x = min_pt.x() + (max_pt.x() - min_pt.x()) / 2;
  center_.y = min_pt.y() + (max_pt.y() - min_pt.y()) / 2;
  center_.z = min_pt.z() + (max_pt.z() - min_pt.z()) / 2;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PoseEstimationMatch::cloud() const {
  return cloud_;
}

geometry_msgs::Pose PoseEstimationMatch::pose() const {
  return pose_;
}

pcl::PointXYZ PoseEstimationMatch::center() const { return center_; }

double PoseEstimationMatch::fitness() const { return fitness_; }

void PoseEstimationMatch::set_fitness(double fitness) { fitness_ = fitness; }

PoseEstimator::PoseEstimator(PoseEstimationHeatMapper* heat_mapper)
    : heat_mapper_(heat_mapper),
      scene_(new PointCloudC()),
      object_(new PointCloudC()),
      object_center_(),
      num_candidates_(100),
      fitness_threshold_(0.0045),
      sigma_threshold_(8),
      nms_radius_(0.02),
      min_results_(0),
      debug_(false),
      candidates_pub_(),
      alignment_pub_(),
      output_pub_(),
      marker_pub_(NULL) {}

void PoseEstimator::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  heat_mapper_->set_scene(scene);
  viz::PublishCloud(scene_pub_, *scene);
}

void PoseEstimator::set_object(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object,
    const rapid_msgs::Roi3D& roi) {
  object_ = object;
  object_roi_ = roi;

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;

  heat_mapper_->set_object(object_);
  if (heat_mapper_->name() == "template_matching") {
    static_cast<TemplateMatchingHeatMapper*>(heat_mapper_)->set_object_roi(roi);
  }

  viz::PublishCloud(object_pub_, *object);
}

PoseEstimationHeatMapper* PoseEstimator::heat_mapper() { return heat_mapper_; }

void PoseEstimator::set_debug(bool val) {
  debug_ = val;
  heat_mapper_->set_debug(val);
}
void PoseEstimator::set_num_candidates(int val) { num_candidates_ = val; }
void PoseEstimator::set_sigma_threshold(double val) { sigma_threshold_ = val; }
void PoseEstimator::set_nms_radius(double val) { nms_radius_ = val; }
void PoseEstimator::set_fitness_threshold(double val) {
  fitness_threshold_ = val;
}
void PoseEstimator::set_min_results(int val) { min_results_ = val; }

void PoseEstimator::set_scene_publisher(const ros::Publisher& pub) {
  scene_pub_ = pub;
}
void PoseEstimator::set_object_publisher(const ros::Publisher& pub) {
  object_pub_ = pub;
}
void PoseEstimator::set_candidates_publisher(const ros::Publisher& pub) {
  candidates_pub_ = pub;
}
void PoseEstimator::set_alignment_publisher(const ros::Publisher& pub) {
  alignment_pub_ = pub;
}
void PoseEstimator::set_output_publisher(const ros::Publisher& pub) {
  output_pub_ = pub;
}
void PoseEstimator::set_marker_publisher(
    rapid_ros::Publisher<visualization_msgs::Marker>* pub) {
  marker_pub_ = pub;
}

void PoseEstimator::Find(vector<PoseEstimationMatch>* matches) {
  // Compute heatmap of features
  pcl::PointIndicesPtr heatmap_indices(new pcl::PointIndices);
  Eigen::VectorXd importances;
  heat_mapper_->Compute(heatmap_indices, &importances);

  // Sample candidate points from the heatmap
  pcl::PointIndices::Ptr candidate_indices(new pcl::PointIndices);
  ComputeTopCandidates(importances, heatmap_indices, candidate_indices);

  // For each candidate point, run ICP
  vector<PoseEstimationMatch> aligned_objects;
  RunIcpCandidates(candidate_indices, &aligned_objects);

  // Non-max suppression
  vector<int> deduped_indices;
  NonMaxSuppression(aligned_objects, &deduped_indices);
  ROS_INFO("Non-max suppression reduced %ld candidates to %ld",
           aligned_objects.size(), deduped_indices.size());

  // Final filtering by score
  vector<int> output_indices;
  FilterMatches(aligned_objects, deduped_indices, &output_indices);
  if (output_indices.size() == 0) {
    ROS_WARN("No matches found.");
    return;
  }

  if (output_pub_) {
    PointCloudC::Ptr output_cloud(new PointCloudC);
    int num_instances = 0;
    for (size_t i = 0; i < output_indices.size(); ++i) {
      int index = output_indices[i];
      double r = static_cast<double>(rand()) / RAND_MAX;
      double g = static_cast<double>(rand()) / RAND_MAX;
      double b = static_cast<double>(rand()) / RAND_MAX;
      Colorize(aligned_objects[index].cloud(), r, g, b);
      *output_cloud += *aligned_objects[index].cloud();
      ++num_instances;
    }
    ROS_INFO("Found %d instances of the model", num_instances);
    output_cloud->header.frame_id = scene_->header.frame_id;
    viz::PublishCloud(output_pub_, *output_cloud);
  }

  matches->clear();
  for (size_t i = 0; i < output_indices.size(); ++i) {
    int index = output_indices[i];
    matches->push_back(aligned_objects[index]);
    ROS_INFO("Pose: %f %f %f", aligned_objects[index].pose().position.x,
      aligned_objects[index].pose().position.y,
      aligned_objects[index].pose().position.z);
  }
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
  if (candidates_pub_) {
    PointCloudC::Ptr viz_cloud(new PointCloudC());
    viz_cloud.reset(new PointCloudC());
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(scene_);
    extract.setIndices(candidate_indices);
    extract.filter(*viz_cloud);
    viz::PublishCloud(candidates_pub_, *viz_cloud);
  }
}

// This is used to sort heatmap scores, represented as (index, score).
// Lower scores are better.
bool CompareScores(const pair<int, double>& a, const pair<int, double>& b) {
  return a.second < b.second;
}

void PoseEstimator::ComputeTopCandidates(
    Eigen::VectorXd& importances, pcl::PointIndicesPtr heatmap_indices,
    pcl::PointIndicesPtr candidate_indices) {
  vector<pair<int, double> > scores;
  for (size_t i = 0; i < heatmap_indices->indices.size(); ++i) {
    int index = heatmap_indices->indices[i];
    double score = importances[i];
    scores.push_back(std::make_pair(index, score));
  }

  size_t num = std::min(scores.size(), static_cast<size_t>(num_candidates_));
  std::partial_sort(scores.begin(), scores.begin() + num, scores.end(),
                    &CompareScores);

  candidate_indices->indices.clear();
  for (size_t i = 0; i < num; ++i) {
    candidate_indices->indices.push_back(scores[i].first);
  }

  // Visualize the candidate points
  if (candidates_pub_) {
    PointCloudC::Ptr viz_cloud(new PointCloudC());
    viz_cloud.reset(new PointCloudC());
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(scene_);
    extract.setIndices(candidate_indices);
    extract.filter(*viz_cloud);
    viz::PublishCloud(candidates_pub_, *viz_cloud);
  }
}

void PoseEstimator::RunIcpCandidates(
    pcl::PointIndices::Ptr candidate_indices,
    vector<PoseEstimationMatch>* aligned_objects) {
  // Copy the object and its center.
  PointCloudC::Ptr working_object(new PointCloudC);
  PointCloudC::Ptr aligned_object(new PointCloudC);
  pcl::IterativeClosestPoint<PointC, PointC> icp;
  icp.setInputTarget(scene_);
  aligned_objects->clear();

  // Compute object to base transform.
  Eigen::Affine3f object_to_base;
  object_to_base = Eigen::Translation3f(-object_center_);

  pcl::PointXYZ roi_translation;
  utils::GeometryMsgToPcl(object_roi_.transform.translation, &roi_translation);
  Eigen::Quaternionf roi_rotation;
  utils::GeometryMsgToEigen(object_roi_.transform.rotation, &roi_rotation);

  for (size_t ci = 0; ci < candidate_indices->indices.size(); ++ci) {
    int index = candidate_indices->indices[ci];
    const PointC& center = scene_->points[index];
    rapid_msgs::Roi3D roi = object_roi_;

    Eigen::Affine3f base_to_candidate;
    base_to_candidate = Eigen::Translation3f(center.x, center.y, center.z);

    // Transform the object to be centered on the center point
    // Also assume that the object and scene are in the same frame

    // Apply one of 8 rotations: 0/90 degree roll, 0/90 degree pitch, 0/90
    // degree yaw
    vector<Eigen::Quaternionf> rotations;
    GenerateRotations(&rotations);
    for (size_t rot_i = 0; rot_i < rotations.size(); ++rot_i) {
      Eigen::Affine3f candidate_rotation = Eigen::Affine3f::Identity();
      candidate_rotation *= rotations[rot_i];

      Eigen::Affine3f candidate_transform =
          base_to_candidate * candidate_rotation * object_to_base;
      pcl::transformPointCloud(*object_, *working_object, candidate_transform);
      // if (debug_) {
      //  viz::PublishCloud(alignment_pub_, *working_object);
      //  std::cout << "Showing candidate object, press enter to continue";
      //  string input;
      //  std::getline(std::cin, input);
      //}

      // Run ICP
      icp.setInputSource(working_object);
      icp.align(*aligned_object);
      viz::PublishCloud(alignment_pub_, *aligned_object);
      if (icp.hasConverged()) {
        Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
        Eigen::Affine3f icp_affine;
        icp_affine = icp_transform;
        Eigen::Affine3f final_affine = icp_affine * candidate_transform;

        // Apply transform to ROI position
        pcl::PointXYZ roi_pt =
            pcl::transformPoint(roi_translation, final_affine);
        utils::PclToGeometryMsg(roi_pt, &roi.transform.translation);

        // Apply transform to ROI rotation
        Eigen::Affine3f final_rotation = final_affine * roi_rotation;
        Eigen::Quaternionf q;
        q = final_rotation.rotation();
        utils::EigenToGeometryMsg(q, &roi.transform.rotation);

        // The transformation of the match, in the base frame.
        double fitness =
            ComputeIcpFitness(scene_, aligned_object, roi, debug_, marker_pub_);
        geometry_msgs::Pose output_pose;
        output_pose.position.x = roi.transform.translation.x;
        output_pose.position.y = roi.transform.translation.y;
        output_pose.position.z = roi.transform.translation.z;
        output_pose.orientation = roi.transform.rotation;
        aligned_objects->push_back(
            PoseEstimationMatch(aligned_object, output_pose, fitness));
      }
    }
  }
}

void PoseEstimator::NonMaxSuppression(
    vector<PoseEstimationMatch>& aligned_objects,
    vector<int>* deduped_indices) {
  deduped_indices->clear();
  // Non-max supression of output objects
  // Index centers into a tree
  PointCloudP::Ptr output_centers(new PointCloudP);
  for (size_t i = 0; i < aligned_objects.size(); ++i) {
    output_centers->push_back(aligned_objects[i].center());
  }
  PointPTree output_centers_tree;
  output_centers_tree.setInputCloud(output_centers);

  // Suppress non max within a radius
  for (int i = 0; i < static_cast<int>(aligned_objects.size()); ++i) {
    PoseEstimationMatch& match = aligned_objects[i];
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
      PoseEstimationMatch& neighbor = aligned_objects[index];
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
    if (keep_match) {
      deduped_indices->push_back(i);
    }
  }
}

void PoseEstimator::FilterMatches(
    const vector<PoseEstimationMatch>& aligned_objects,
    const vector<int>& deduped_indices, vector<int>* output_indices) {
  double best_fitness = std::numeric_limits<double>::max();
  int best_index = 0;
  vector<double> scores(deduped_indices.size());
  vector<pair<int, double> > index_scores(deduped_indices.size());
  for (size_t di = 0; di < deduped_indices.size(); ++di) {
    int index = deduped_indices[di];
    const PoseEstimationMatch& match = aligned_objects[index];
    double fitness = match.fitness();
    scores[di] = fitness;
    index_scores[di] = std::make_pair(index, fitness);
    string threshold_marker = "";
    if (fitness < fitness_threshold_) {
      output_indices->push_back(index);
      threshold_marker = " -- below threshold";
    }
    ROS_INFO("(%ld/%ld) ICP converged with score: %f%s", di + 1,
             deduped_indices.size(), fitness, threshold_marker.c_str());

    if (fitness < best_fitness) {
      best_fitness = fitness;
      best_index = index;
    }

    viz::PublishCloud(alignment_pub_, *(match.cloud()));
    if (debug_) {
      std::cout << "Press enter to continue ";
      string user_input;
      std::getline(std::cin, user_input);
    }
  }
  ROS_INFO("Best score in candidates: %f", best_fitness);

  if (output_indices->size() == 0) {
    // Some matches don't meet the threshold but are significantly better than
    // the other matches. We look at the # of standard deviations from the mean
    // to see if the best match should be output anyway.
    double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    double mean = sum / scores.size();
    double sq_sum =
        std::inner_product(scores.begin(), scores.end(), scores.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / scores.size() - mean * mean);

    double sigmas = (mean - best_fitness) / stdev;
    if (sigmas >= sigma_threshold_) {
      output_indices->push_back(best_index);
    }
    ROS_INFO("Fitness mean: %f stdev: %f, best is %f sigmas from mean", mean,
             stdev, sigmas);
  }

  if (static_cast<int>(output_indices->size()) < min_results_) {
    int num_results =
        std::min(min_results_, static_cast<int>(index_scores.size()));
    std::partial_sort(index_scores.begin(), index_scores.begin() + num_results,
                      index_scores.end(), &CompareScores);
    int start = static_cast<int>(output_indices->size());
    for (int i = start; i < num_results; ++i) {
      const pair<int, double>& index_score = index_scores[i];
      output_indices->push_back(index_score.first);
    }
  }
}

void PoseEstimator::GenerateRotations(
    std::vector<Eigen::Quaternionf>* rotations) {
  rotations->clear();
  // for (double roll = 0; roll < 2 * M_PI - 0.01; roll += M_PI / 2) {
  // for (double pitch = 0; pitch < M_PI / 2 + 0.01; pitch += M_PI / 2) {
  //  for (double yaw = 0; yaw < M_PI / 2 + 0.01; yaw += M_PI / 2) {
  Eigen::Affine3f affine = pcl::getTransformation(0, 0, 0, 0, 0, 0);
  Eigen::Quaternionf q(affine.rotation());
  rotations->push_back(q);
  //}
  //}
  //}
}

void Colorize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double r, double g,
              double b) {
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud->points[i].r = static_cast<int>(round(r * 255));
    cloud->points[i].g = static_cast<int>(round(g * 255));
    cloud->points[i].b = static_cast<int>(round(b * 255));
  }
}
}  // namespace perception
}  // namespace rapid
