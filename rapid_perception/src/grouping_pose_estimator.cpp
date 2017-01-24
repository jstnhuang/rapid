#include "rapid_perception/grouping_pose_estimator.h"

#include "rapid_perception/rgbd.h"

namespace rapid {
namespace perception {
GroupingPoseEstimator::GroupingPoseEstimator() {}

void GroupingPoseEstimator::set_scene(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) {}

void GroupingPoseEstimator::set_object(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object) {}

void GroupingPoseEstimator::Find(std::vector<PoseEstimationMatch>* matches) {}
}  // namespace perception
}  // namespace rapid
