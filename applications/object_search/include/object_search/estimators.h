#ifndef _OBJECT_SEARCH_ESTIMATORS_H_
#define _OBJECT_SEARCH_ESTIMATORS_H_

#include "rapid_perception/pose_estimation.h"
#include "rapid_perception/ransac_pose_estimator.h"

namespace object_search {
struct Estimators {
  rapid::perception::PoseEstimator* custom;
  rapid::perception::RansacPoseEstimator* ransac;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_ESTIMATORS_H_
