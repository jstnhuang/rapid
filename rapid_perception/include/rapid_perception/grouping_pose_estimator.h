#ifndef _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_
#define _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_

#include "rapid_perception/pose_estimation_interface.h"

#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
namespace perception {
// PoseEstimator based on correspondence grouping.
class GroupingPoseEstimator : public PoseEstimationInterface {
 public:
  GroupingPoseEstimator();
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object);
  void Find(std::vector<PoseEstimationMatch>* matches);

 private:
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_GROUPING_POSE_ESTIMATOR_H_
