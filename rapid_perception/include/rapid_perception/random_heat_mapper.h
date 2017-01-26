#ifndef _RAPID_PERCEPTION_RANDOM_HEAT_MAPPER_H_
#define _RAPID_PERCEPTION_RANDOM_HEAT_MAPPER_H_

#include "Eigen/Core"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/pose_estimation_heat_mapper.h"

namespace rapid {
namespace perception {
// A heat mapper that just gives equal weight to a random sample of the scene.
class RandomHeatMapper : public PoseEstimationHeatMapper {
 public:
  RandomHeatMapper();
  void Compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap,
               Eigen::VectorXd* importances);
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);
  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_landmark_dimensions(double x, double y, double z);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_;

  double sample_ratio_;
  int max_samples_;
  double scale_x_;
  double scale_y_;
  double scale_z_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_RANDOM_HEAT_MAPPER_H_
