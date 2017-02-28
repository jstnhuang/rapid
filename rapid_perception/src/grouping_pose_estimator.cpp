#include "rapid_perception/grouping_pose_estimator.h"

#include <vector>

#include "Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/correspondence.h"
#include "pcl/features/board.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/cg/geometric_consistency.h"
#include "pcl/recognition/cg/hough_3d.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "rapid_perception/pose_estimation_match.h"
#include "rapid_viz/publish.h"

using pcl::Normal;
using pcl::SHOT352;
using pcl::ReferenceFrame;
using pcl::PointCloud;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;
typedef pcl::PointCloud<pcl::SHOT352> PointCloudD;

namespace rapid {
namespace perception {

const int GroupingPoseEstimator::kDefaultNormalK = 10;
const double GroupingPoseEstimator::kDefaultShotRadius = 0.02;
const double GroupingPoseEstimator::kDefaultObjectVox = 0.01;
const double GroupingPoseEstimator::kDefaultSceneVox = 0.03;
const double GroupingPoseEstimator::kDefaultCorrMatchThreshold = 0.25;
const bool GroupingPoseEstimator::kDefaultUseHough = true;
const double GroupingPoseEstimator::kDefaultRfRadius = 0.015;
const double GroupingPoseEstimator::kDefaultCgSize = 0.01;
const double GroupingPoseEstimator::kDefaultCgThreshold = 5;

GroupingPoseEstimator::GroupingPoseEstimator()
    : normal_k_(kDefaultNormalK),
      shot_radius_(kDefaultShotRadius),
      object_vox_(kDefaultObjectVox),
      scene_vox_(kDefaultSceneVox),
      corr_match_threshold_(kDefaultCorrMatchThreshold),
      use_hough_(kDefaultUseHough),
      rf_radius_(kDefaultRfRadius),
      cg_size_(kDefaultCgSize),
      cg_threshold_(kDefaultCgThreshold),
      scene_(new PointCloudC),
      scene_normals_(new PointCloudN),
      scene_keypoints_(new PointCloudC),
      scene_descriptors_(new PointCloudD),
      object_(new PointCloudC),
      object_normals_(new PointCloudN),
      object_keypoints_(new PointCloudC),
      object_descriptors_(new PointCloudD),
      correspondence_pub_(),
      scene_keypoints_pub_(),
      object_keypoints_pub_() {}

void GroupingPoseEstimator::set_scene(PointCloudC::Ptr scene) {
  scene_ = scene;
  ComputeNormals(scene_, scene_normals_);
  ComputeKeypoints(scene_, scene_vox_, scene_keypoints_);
  ComputeDescriptors(scene_keypoints_, scene_normals_, scene_,
                     scene_descriptors_);
  viz::PublishCloud(scene_keypoints_pub_, *scene_keypoints_);
  ROS_INFO("Loaded scene with %ld points, %ld keypoints", scene_->size(),
           scene_keypoints_->size());
}

void GroupingPoseEstimator::set_object(
    const PointCloud<pcl::PointXYZRGB>::Ptr& object) {
  object_ = object;
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*object, min_pt, max_pt);
  object_dimensions_.x = max_pt.x() - min_pt.x();
  object_dimensions_.y = max_pt.y() - min_pt.y();
  object_dimensions_.z = max_pt.z() - min_pt.z();
  ComputeNormals(object_, object_normals_);
  ComputeKeypoints(object_, object_vox_, object_keypoints_);
  ComputeDescriptors(object_keypoints_, object_normals_, object_,
                     object_descriptors_);
  viz::PublishCloud(object_keypoints_pub_, *object_keypoints_);
  ROS_INFO("Loaded object with %ld points, %ld keypoints", object_->size(),
           object_keypoints_->size());
}

void GroupingPoseEstimator::Find(std::vector<PoseEstimationMatch>* matches) {
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  FindCorrespondences(model_scene_corrs);

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
      rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
  ClusterCorrespondences(model_scene_corrs, &rototranslations,
                         &clustered_corrs);

  ROS_INFO("Found %ld instances of the landmark.", rototranslations.size());

  for (size_t i = 0; i < rototranslations.size(); ++i) {
    Eigen::Affine3f transform(rototranslations[i]);
    Eigen::Quaternionf quat(transform.rotation());
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();

    PointCloudC::Ptr working(new PointCloudC);
    pcl::transformPointCloud(*object_, *working, transform);

    PoseEstimationMatch match(working, pose, object_dimensions_,
                              1);  // TODO(jstn): no score yet
    matches->push_back(match);
  }
}

void GroupingPoseEstimator::set_correspondence_publisher(
    const ros::Publisher& pub) {
  correspondence_pub_ = pub;
}

void GroupingPoseEstimator::set_scene_keypoints_publisher(
    const ros::Publisher& pub) {
  scene_keypoints_pub_ = pub;
}

void GroupingPoseEstimator::set_object_keypoints_publisher(
    const ros::Publisher& pub) {
  object_keypoints_pub_ = pub;
}

void GroupingPoseEstimator::ComputeNormals(
    const PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    PointCloud<pcl::Normal>::Ptr normals) {
  pcl::NormalEstimationOMP<PointC, Normal> nest;
  nest.setKSearch(normal_k_);
  nest.setInputCloud(cloud);
  nest.compute(*normals);
}

void GroupingPoseEstimator::ComputeKeypoints(
    const PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const double sampling_radius, PointCloud<pcl::PointXYZRGB>::Ptr keypoints) {
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(sampling_radius, sampling_radius, sampling_radius);
  vox.filter(*keypoints);
}

void GroupingPoseEstimator::ComputeDescriptors(
    const PointCloud<pcl::PointXYZRGB>::Ptr& keypoints,
    const PointCloud<pcl::Normal>::Ptr& normals,
    const PointCloud<pcl::PointXYZRGB>::Ptr& surface,
    PointCloud<pcl::SHOT352>::Ptr descriptors) {
  pcl::SHOTEstimation<PointC, Normal, SHOT352> shot;
  shot.setRadiusSearch(shot_radius_);
  shot.setInputCloud(keypoints);
  shot.setInputNormals(normals);
  shot.setSearchSurface(surface);
  shot.compute(*descriptors);
}

void GroupingPoseEstimator::FindCorrespondences(
    pcl::CorrespondencesPtr model_scene_corrs) {
  pcl::KdTreeFLANN<SHOT352> match_search;
  match_search.setInputCloud(object_descriptors_);
  // For each scene keypoint descriptor, find nearest neighbor into the model
  // keypoints descriptor cloud and add it to the correspondences vector.
  int nan_count = 0;
  int no_neighbors = 0;
  int under_threshold = 0;
  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < scene_descriptors_->size(); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!pcl_isfinite(scene_descriptors_->at(i).descriptor[0])) {
      ++nan_count;
      continue;
    }
    int found_neighs = match_search.nearestKSearch(
        scene_descriptors_->at(i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs < 1) {
      ++no_neighbors;
      continue;
    }
    // add match only if the squared descriptor distance is less than 0.25
    // (SHOT descriptor distances are between 0 and 1 by design)
    if (neigh_sqr_dists[0] < corr_match_threshold_) {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                               neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    } else {
      ++under_threshold;
    }
  }
  // Visualize correspondences
  for (size_t i = 0; i < model_scene_corrs->size(); ++i) {
    int index_query = model_scene_corrs->at(i).index_query;
    int index_match = model_scene_corrs->at(i).index_match;

    const PointC& model_point = object_keypoints_->at(index_query);
    const PointC& scene_point = scene_keypoints_->at(index_match);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.ns = "corrs";
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.0025;
    marker.scale.y = 0.005;
    marker.color.r = 1;
    marker.color.a = 0.5;
    marker.points.resize(2);
    marker.points[0].x = model_point.x;
    marker.points[0].y = model_point.y;
    marker.points[0].z = model_point.z;
    marker.points[1].x = scene_point.x;
    marker.points[1].y = scene_point.y;
    marker.points[1].z = scene_point.z;
    markers.markers.push_back(marker);
  }

  correspondence_pub_.publish(markers);

  ROS_INFO("Found %ld correspondences", model_scene_corrs->size());
  ROS_INFO(
      "Out of %ld keypoints, %d were NaNs, %d had no neighbors, and %d were "
      "over the threshold %f",
      scene_descriptors_->size(), nan_count, no_neighbors, under_threshold,
      corr_match_threshold_);
}

void GroupingPoseEstimator::ClusterCorrespondences(
    const pcl::CorrespondencesPtr& model_scene_corrs,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >*
        rototranslations,
    std::vector<pcl::Correspondences>* clustered_corrs) {
  if (use_hough_) {
    PointCloud<ReferenceFrame>::Ptr model_rf(new PointCloud<ReferenceFrame>());
    PointCloud<ReferenceFrame>::Ptr scene_rf(new PointCloud<ReferenceFrame>());

    pcl::BOARDLocalReferenceFrameEstimation<PointC, Normal, ReferenceFrame>
        rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_radius_);

    rf_est.setInputCloud(object_keypoints_);
    rf_est.setInputNormals(object_normals_);
    rf_est.setSearchSurface(object_);
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_keypoints_);
    rf_est.setInputNormals(scene_normals_);
    rf_est.setSearchSurface(scene_);
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointC, PointC, ReferenceFrame, ReferenceFrame>
        clusterer;
    clusterer.setHoughBinSize(cg_size_);
    clusterer.setHoughThreshold(cg_threshold_);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);
    clusterer.setInputCloud(object_keypoints_);
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_keypoints_);
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);
    viz::PublishCloud(scene_keypoints_pub_, *scene_keypoints_);
    viz::PublishCloud(object_keypoints_pub_, *object_keypoints_);
    clusterer.recognize(*rototranslations, *clustered_corrs);
  } else {
    pcl::GeometricConsistencyGrouping<PointC, PointC> gc_clusterer;
    gc_clusterer.setGCSize(cg_size_);
    gc_clusterer.setGCThreshold(cg_threshold_);
    gc_clusterer.setInputCloud(object_keypoints_);
    gc_clusterer.setSceneCloud(scene_keypoints_);
    gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
    gc_clusterer.recognize(*rototranslations, *clustered_corrs);
  }
}
}  // namespace perception
}  // namespace rapid
