#include "rapid_perception/pose_estimation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "boost/asio/io_service.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"

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
#include "rapid_perception/pose_estimation_match.h"
#include "rapid_perception/random_heat_mapper.h"
#include "rapid_perception/template_matching_heat_mapper.h"
#include "rapid_utils/eigen_conversions.h"
#include "rapid_utils/pcl_conversions.h"
#include "rapid_utils/stochastic_universal_sampling.h"
#include "rapid_viz/markers.h"
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
PoseEstimator::PoseEstimator(PoseEstimationHeatMapper* heat_mapper)
    : heat_mapper_(heat_mapper),
      scene_(new PointCloudC()),
      object_(new PointCloudC()),
      object_center_(),
      object_box_(viz::Marker::Null()),
      output_boxes_(),
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
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object) {
  object_ = object;

  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_, min_pt, max_pt);
  object_center_.x() = (max_pt.x() + min_pt.x()) / 2;
  object_center_.y() = (max_pt.y() + min_pt.y()) / 2;
  object_center_.z() = (max_pt.z() + min_pt.z()) / 2;
  object_dims_.x() = max_pt.x() - min_pt.x();
  object_dims_.y() = max_pt.y() - min_pt.y();
  object_dims_.z() = max_pt.z() - min_pt.z();

  heat_mapper_->set_object(object_);
  viz::PublishCloud(object_pub_, *object);
}

void PoseEstimator::set_roi(const rapid_msgs::Roi3D& roi) {
  object_roi_ = roi;

  if (heat_mapper_->name() == "template_matching") {
    static_cast<TemplateMatchingHeatMapper*>(heat_mapper_)->set_object_roi(roi);
  }

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = object_->header.frame_id;
  pose.pose.position.x = roi.transform.translation.x;
  pose.pose.position.y = roi.transform.translation.y;
  pose.pose.position.z = roi.transform.translation.z;
  pose.pose.orientation = roi.transform.rotation;
  object_box_ = viz::Marker::OutlineBox(marker_pub_, pose, roi.dimensions);
  geometry_msgs::Vector3 scale;
  scale.x = 0.0025;
  object_box_.SetScale(scale);
  object_box_.SetNamespace("object");
  object_box_.Publish();
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
  output_boxes_.clear();  // Clear visualization.

  // Compute heatmap of features
  PointCloudC::Ptr heatmap(new PointCloudC);
  Eigen::VectorXd importances;
  if (heat_mapper_->name() == "random") {
    static_cast<RandomHeatMapper*>(heat_mapper_)
        ->set_landmark_dimensions(object_dims_.x(), object_dims_.y(),
                                  object_dims_.z());
  }
  heat_mapper_->Compute(heatmap, &importances);

  // Sample candidate points from the heatmap
  PointCloudC::Ptr candidates(new PointCloudC);
  ComputeTopCandidates(importances, heatmap, candidates);

  // For each candidate point, run ICP
  vector<PoseEstimationMatch> aligned_objects;
  RunIcpCandidates(candidates, &aligned_objects);

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

  ROS_INFO("Found %ld instances of the model", output_indices.size());

  matches->clear();
  for (size_t i = 0; i < output_indices.size(); ++i) {
    int index = output_indices[i];
    matches->push_back(aligned_objects[index]);
  }

  VisualizeMatches(output_pub_, *matches);

  // Sort matches by score.
  std::sort(matches->begin(), matches->end(), &ComparePoseEstimationMatch);
  for (size_t i = 0; i < matches->size(); ++i) {
    const PoseEstimationMatch& match = matches->at(i);
    ROS_INFO("Score: %f, Pose: %f %f %f", match.fitness(),
             match.pose().position.x, match.pose().position.y,
             match.pose().position.z);
  }
}

void PoseEstimator::ComputeCandidates(Eigen::VectorXd& importances,
                                      PointCloudC::Ptr heatmap,
                                      PointCloudC::Ptr candidates) {
  // Sample points based on importance
  double sum = importances.sum();
  importances /= sum;

  vector<double> cdf;
  double cdf_val = 0;
  for (size_t indices_i = 0; indices_i < heatmap->size(); ++indices_i) {
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
    int index = sampled_important_indices[i];
    candidates->push_back(heatmap->at(index));
  }
  // viz::PublishCloud(candidates_pub_, *candidates);
}

// This is used to sort heatmap scores, represented as (index, score).
// Lower scores are better.
bool CompareScores(const pair<int, double>& a, const pair<int, double>& b) {
  return a.second < b.second;
}

void PoseEstimator::ComputeTopCandidates(Eigen::VectorXd& importances,
                                         PointCloudC::Ptr heatmap,
                                         PointCloudC::Ptr candidates) {
  // TODO(jstn): totally remove num_candidates_ as a parameter.
  num_candidates_ = heatmap->size();

  vector<pair<int, double> > scores;
  for (size_t i = 0; i < heatmap->size(); ++i) {
    double score = importances[i];
    scores.push_back(std::make_pair(i, score));
  }

  size_t num = std::min(scores.size(), static_cast<size_t>(num_candidates_));
  std::partial_sort(scores.begin(), scores.begin() + num, scores.end(),
                    &CompareScores);

  candidates->clear();
  for (size_t i = 0; i < num; ++i) {
    candidates->push_back(heatmap->at(scores[i].first));
  }
  // viz::PublishCloud(candidates_pub_, *candidates);
}

void PoseEstimator::RunIcpCandidateInThread(
    PointCloudC::Ptr candidates, size_t candidate_index, int max_iterations,
    boost::mutex& output_mutex, vector<PoseEstimationMatch>* aligned_objects) {
  // Copy the object and its center.
  PointCloudC::Ptr working_object(new PointCloudC);
  PointCloudC::Ptr aligned_object(new PointCloudC);
  pcl::IterativeClosestPoint<PointC, PointC> icp;
  icp.setInputTarget(scene_);
  icp.setMaximumIterations(max_iterations);

  // Compute object to base transform.
  Eigen::Affine3f object_to_base;
  object_to_base = Eigen::Translation3f(-object_center_);

  pcl::PointXYZ roi_translation;
  utils::GeometryMsgToPcl(object_roi_.transform.translation, &roi_translation);
  Eigen::Quaternionf roi_rotation;
  utils::GeometryMsgToEigen(object_roi_.transform.rotation, &roi_rotation);

  const PointC& center = candidates->at(candidate_index);
  rapid_msgs::Roi3D roi = object_roi_;

  Eigen::Affine3f base_to_candidate;
  base_to_candidate = Eigen::Translation3f(center.x, center.y, center.z);

  // Transform the object to be centered on the center point
  // Also assume that the object and scene are in the same frame

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
    Eigen::Matrix4f icp_transform = icp.getFinalTransformation();
    Eigen::Affine3f icp_affine;
    icp_affine = icp_transform;
    Eigen::Affine3f final_affine = icp_affine * candidate_transform;

    // Apply transform to ROI position
    pcl::PointXYZ roi_pt = pcl::transformPoint(roi_translation, final_affine);
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
    output_mutex.lock();
    aligned_objects->push_back(
        PoseEstimationMatch(aligned_object, output_pose, fitness));
    output_mutex.unlock();
  }
}

void PoseEstimator::RunIcpCandidates(
    PointCloudC::Ptr candidates, vector<PoseEstimationMatch>* aligned_objects) {
  boost::asio::io_service io_service;
  boost::thread_group threadpool;
  boost::shared_ptr<boost::asio::io_service::work> work(
      new boost::asio::io_service::work(io_service));

  aligned_objects->clear();

  int num_threads = 4;
  ros::param::param<int>("num_threads", num_threads, 4);
  for (int i = 0; i < num_threads; ++i) {
    threadpool.create_thread(
        boost::bind(&boost::asio::io_service::run, &io_service));
  }

  int max_iterations;
  ros::param::param("icp_max_iterations", max_iterations, 50);

  boost::mutex output_mutex;
  for (size_t ci = 0; ci < candidates->size(); ++ci) {
    io_service.post(boost::bind(&PoseEstimator::RunIcpCandidateInThread, this,
                                candidates, ci, max_iterations, boost::ref(output_mutex),
                                aligned_objects));
  }
  work.reset();
  threadpool.join_all();
}

void PoseEstimator::NonMaxSuppression(
    vector<PoseEstimationMatch>& aligned_objects,
    vector<int>* deduped_indices) {
  // TODO(jstn): remove nms_radius as a parameter. The radius should be
  // dynamically sized based on the object dimensions. Instead of having just
  // one radius, it should search for neighbors in an ellipsoid. Below, we are
  // still using a single radius whose length is half of the longest dimension.
  nms_radius_ =
      std::max(std::max(object_dims_.x(), object_dims_.y()), object_dims_.z()) /
      2.0;

  deduped_indices->clear();
  // Non-max supression of output objects
  // Index centers into a tree
  PointCloudP::Ptr output_centers(new PointCloudP);
  PointCloudC::Ptr viz_deduped(new PointCloudC);
  viz_deduped->header.frame_id = scene_->header.frame_id;
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
        neighbor.set_fitness(neighbor.fitness() + 0.0001);
      }
    }
    if (keep_match) {
      deduped_indices->push_back(i);
      PointC max;
      max.x = match.center().x;
      max.y = match.center().y;
      max.z = match.center().z;
      viz_deduped->push_back(max);
    }
  }

  viz::PublishCloud(candidates_pub_, *viz_deduped);
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
    if (debug_) {
      ROS_INFO("(%ld/%ld) ICP converged with score: %f%s", di + 1,
               deduped_indices.size(), fitness, threshold_marker.c_str());
    }

    if (fitness < best_fitness) {
      best_fitness = fitness;
      best_index = index;
    }

    viz::PublishCloud(alignment_pub_, *(match.cloud()));
    if (debug_ && match.fitness() < 0.01) {
      std::cout << "Score: " << match.fitness() << ", press enter to continue ";
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
}  // namespace perception
}  // namespace rapid
