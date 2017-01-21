#ifndef _RAPID_RANSAC_POSE_ESTIMATOR_H_
#define _RAPID_RANSAC_POSE_ESTIMATOR_H_

#include "boost/shared_ptr.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/ransac_based/obj_rec_ransac.h"

#include "rapid_perception/pose_estimation_interface.h"
#include "rapid_perception/pose_estimation_match.h"

namespace rapid {
namespace perception {
class RansacPoseEstimator : public PoseEstimationInterface {
 public:
  RansacPoseEstimator();
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object);
  void Find(std::vector<PoseEstimationMatch>* matches);

  static const double kDefaultPairWidth;
  static const double kDefaultVoxelSize;
  static const char kObjectName[];

 private:
  boost::shared_ptr<pcl::recognition::ObjRecRANSAC> recognizer_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_object_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_RANSAC_POSE_ESTIMATOR_H_
