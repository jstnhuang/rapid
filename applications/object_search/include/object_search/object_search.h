#ifndef _OBJECT_SEARCH_OBJECT_SEARCH_H_
#define _OBJECT_SEARCH_OBJECT_SEARCH_H_

#include "rapid_perception/grouping_pose_estimator.h"
#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/ransac_pose_estimator.h"

namespace object_search {
void UpdateEstimatorParams(rapid::perception::PoseEstimator* custom);
void UpdateEstimatorParams(rapid::perception::RansacPoseEstimator* ransac);
void UpdateEstimatorParams(rapid::perception::GroupingPoseEstimator* grouping);

void CropScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped);
void Downsample(const double leaf_size,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
}  // namespace object_search

#endif  // _OBJECT_SEARCH_OBJECT_SEARCH_H_
