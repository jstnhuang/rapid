#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_LINEMOD_HEAT_MAPPER_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_LINEMOD_HEAT_MAPPER_H_

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//#include "pcl/recognition/line_rgbd.h"
//#include "pcl/recognition/linemod/line_rgbd.h"

#include "rapid_perception/pose_estimation_heat_mapper.h"

namespace rapid {
namespace perception {
class LinemodHeatMapper : public PoseEstimationHeatMapper {
 public:
  LinemodHeatMapper();
  void Compute(pcl::PointIndicesPtr indices, Eigen::VectorXd* importances);
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);
  void set_sample_ratio(double val);
  void set_max_samples(int val);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_;
  Eigen::Vector3d object_center_;
  double sample_ratio_;
  int max_samples_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_LINEMOD_HEAT_MAPPER_H_
