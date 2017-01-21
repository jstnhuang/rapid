#include "rapid_perception/ransac_pose_estimator.h"

#include <list>

#include "boost/shared_ptr.hpp"
#include "pcl/common/common.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/ransac_based/obj_rec_ransac.h"

#include "rapid_perception/pose_estimation_interface.h"
#include "rapid_perception/pose_estimation_match.h"

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
const char RansacPoseEstimator::kObjectName[] = "object";

RansacPoseEstimator::RansacPoseEstimator()
    : recognizer_(new ObjRecRANSAC(kDefaultPairWidth, kDefaultVoxelSize)),
      scene_(new PointCloudP) {}

void RansacPoseEstimator::set_scene(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) {
  pcl::copyPointCloud(*scene, *scene_);
  pcl::NormalEstimationOMP<PointP, pcl::Normal> nest;
  nest.setInputCloud(scene_);
  nest.compute(*scene_normals_);
}

void RansacPoseEstimator::set_object(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object) {
  color_object_ = object;
  PointCloudP::Ptr xyz_cloud(new PointCloudP);
  pcl::copyPointCloud(*object, *xyz_cloud);
  pcl::NormalEstimationOMP<PointP, pcl::Normal> nest;
  nest.setInputCloud(xyz_cloud);
  PointCloudN::Ptr normal_cloud(new PointCloudN);
  nest.compute(*normal_cloud);
  recognizer_->addModel(*xyz_cloud, *normal_cloud, kObjectName);
}

void RansacPoseEstimator::Find(std::vector<PoseEstimationMatch>* matches) {
  matches->clear();
  std::list<ObjRecRANSAC::Output> output;
  recognizer_->recognize(*scene_, *scene_normals_, output);
  for (std::list<ObjRecRANSAC::Output>::iterator it = output.begin();
       it != output.end(); ++it) {
    Eigen::Matrix3f rotation;
    for (int i = 0; i < 9; ++i) {
      rotation << it->rigid_transform_[i];
    }
    Eigen::Quaternionf quaternion(rotation);
    geometry_msgs::Pose pose;
    pose.orientation.w = quaternion.w();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.position.x = it->rigid_transform_[9];
    pose.position.y = it->rigid_transform_[10];
    pose.position.z = it->rigid_transform_[11];
    PoseEstimationMatch match(color_object_, pose, it->match_confidence_);
    matches->push_back(match);
  }
}
}  // namespace perception
}  // namespace rapid

int main(int argc, char** argv) { return 0; }
