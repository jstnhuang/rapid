#include "rapid_perception/ransac_pose_estimator.h"

#include <iostream>
#include <list>

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/PoseArray.h"
#include "pcl/common/common.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/ransac_based/obj_rec_ransac.h"

#include "rapid_perception/pose_estimation_interface.h"
#include "rapid_perception/pose_estimation_match.h"
#include "rapid_viz/publish.h"
#include "rapid_ros/publisher.h"

typedef pcl::PointXYZ PointP;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointP> PointCloudP;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<pcl::Normal> PointCloudN;

using pcl::recognition::ObjRecRANSAC;

namespace rapid {
namespace perception {
const double RansacPoseEstimator::kDefaultPairWidth = 0.025;
const double RansacPoseEstimator::kDefaultVoxelSize = 0.01;
const double RansacPoseEstimator::kDefaultSceneNormalRadius = 0.025;
const double RansacPoseEstimator::kDefaultObjectNormalRadius = 0.01;
const double RansacPoseEstimator::kDefaultThreshold = 0.3;
const char RansacPoseEstimator::kObjectName[] = "object";

RansacPoseEstimator::RansacPoseEstimator()
    : pair_width_(kDefaultPairWidth),
      voxel_size_(kDefaultVoxelSize),
      scene_normal_radius_(kDefaultSceneNormalRadius),
      object_normal_radius_(kDefaultObjectNormalRadius),
      threshold_(kDefaultThreshold),
      recognizer_(new ObjRecRANSAC(pair_width_, voxel_size_)),
      scene_(new PointCloudP),
      scene_normals_(new PointCloudN) {}

void RansacPoseEstimator::set_scene(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) {
  last_scene_in_ = scene;
  pcl::copyPointCloud(*scene, *scene_);
  pcl::NormalEstimationOMP<PointP, pcl::Normal> nest;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  nest.setInputCloud(scene_);
  nest.setNumberOfThreads(4);
  nest.setSearchMethod(tree);
  nest.setRadiusSearch(scene_normal_radius_);
  nest.compute(*scene_normals_);
  viz::PublishCloud(scene_pub_, *scene);
  ROS_INFO("RANSAC set scene with %ld points", scene->size());
}

void RansacPoseEstimator::set_object(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object_in) {
  recognizer_->clear();
  last_object_in_ = object_in;

  // Demean object
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*object_in, min_pt, max_pt);
  Eigen::Vector4f centroid((max_pt.x() + min_pt.x()) / 2,
                           (max_pt.y() + min_pt.y()) / 2,
                           (max_pt.z() + min_pt.z()) / 2, 1);
  PointCloudC::Ptr object(new PointCloudC);
  pcl::demeanPointCloud(*object_in, centroid, *object);

  color_object_ = object;
  PointCloudP::Ptr xyz_cloud(new PointCloudP);
  pcl::copyPointCloud(*object, *xyz_cloud);
  pcl::NormalEstimationOMP<PointP, pcl::Normal> nest;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  nest.setInputCloud(xyz_cloud);
  PointCloudN::Ptr normal_cloud(new PointCloudN);
  nest.setNumberOfThreads(4);
  nest.setSearchMethod(tree);
  nest.setRadiusSearch(object_normal_radius_);
  nest.compute(*normal_cloud);
  recognizer_->addModel(*xyz_cloud, *normal_cloud, kObjectName);
  viz::PublishCloud(object_pub_, *object);
  ROS_INFO("RANSAC set object with %ld points", object->size());
}

void RansacPoseEstimator::Find(std::vector<PoseEstimationMatch>* matches) {
  matches->clear();
  std::list<ObjRecRANSAC::Output> output;
  recognizer_->recognize(*scene_, *scene_normals_, output);
  geometry_msgs::PoseArray p_arr;
  p_arr.header.frame_id = "base_footprint";
  for (std::list<ObjRecRANSAC::Output>::iterator it = output.begin();
       it != output.end(); ++it) {
    float* rt = it->rigid_transform_;

    if (it->match_confidence_ < threshold_) {
      ROS_INFO("Match at %f %f %f: %f (below threshold)", rt[9], rt[10], rt[11],
               it->match_confidence_);
      continue;
    }

    Eigen::Matrix3f rotation;
    rotation << rt[0], rt[1], rt[2], rt[3], rt[4], rt[5], rt[6], rt[7], rt[8];
    Eigen::Quaternionf quaternion(rotation);

    PointCloudC::Ptr working(new PointCloudC);
    pcl::copyPointCloud(*color_object_, *working);
    Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Zero();
    transform_mat.topLeftCorner(3, 3) = rotation;
    transform_mat(0, 3) = rt[9];
    transform_mat(1, 3) = rt[10];
    transform_mat(2, 3) = rt[11];
    transform_mat(3, 3) = 1;
    Eigen::Affine3f affine(transform_mat);
    PointCloudC::Ptr working_transformed(new PointCloudC);
    pcl::transformPointCloud(*working, *working_transformed, affine);

    geometry_msgs::Pose pose;
    pose.orientation.w = quaternion.w();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.position.x = rt[9];
    pose.position.y = rt[10];
    pose.position.z = rt[11];
    p_arr.poses.push_back(pose);
    PoseEstimationMatch match(working_transformed, pose, it->match_confidence_);
    matches->push_back(match);
    ROS_INFO("Match at %f %f %f: %f", pose.position.x, pose.position.y,
             pose.position.z, it->match_confidence_);
  }

  ROS_INFO("RANSAC found %ld matches", matches->size());

  VisualizeMatches(output_pub_, *matches);
  if (pose_pub_) {
    pose_pub_.publish(p_arr);
  }
}

void RansacPoseEstimator::set_roi(const rapid_msgs::Roi3D& roi) {
  object_roi_ = roi;
}

void RansacPoseEstimator::set_scene_publisher(const ros::Publisher& pub) {
  scene_pub_ = pub;
}
void RansacPoseEstimator::set_object_publisher(const ros::Publisher& pub) {
  object_pub_ = pub;
}
void RansacPoseEstimator::set_output_publisher(const ros::Publisher& pub) {
  output_pub_ = pub;
}
void RansacPoseEstimator::set_pose_publisher(const ros::Publisher& pub) {
  pose_pub_ = pub;
}

void RansacPoseEstimator::set_pair_width(double val) {
  pair_width_ = val;
  recognizer_.reset(new ObjRecRANSAC(pair_width_, voxel_size_));
  if (last_scene_in_) {
    set_scene(last_scene_in_);
  }
  if (last_object_in_) {
    set_object(last_object_in_);
  }
}
void RansacPoseEstimator::set_voxel_size(double val) {
  voxel_size_ = val;
  recognizer_.reset(new ObjRecRANSAC(pair_width_, voxel_size_));
  if (last_scene_in_) {
    set_scene(last_scene_in_);
  }
  if (last_object_in_) {
    set_object(last_object_in_);
  }
}
void RansacPoseEstimator::set_scene_normal_radius(double val) {
  scene_normal_radius_ = val;
}
void RansacPoseEstimator::set_object_normal_radius(double val) {
  object_normal_radius_ = val;
}
void RansacPoseEstimator::set_threshold(double val) { threshold_ = val; }
double RansacPoseEstimator::pair_width() const { return pair_width_; }
double RansacPoseEstimator::voxel_size() const { return voxel_size_; }
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
