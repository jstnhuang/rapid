#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_HEAT_MAPPER_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_HEAT_MAPPER_H_

#include "Eigen/Core"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "ros/ros.h"

namespace rapid {
namespace perception {
// Base class for a heat mapper for pose estimation.
// All heat mappers get a debug flag and a point cloud publisher.
// The scene, object, parameters, etc. are passed into subclasses.
class PoseEstimationHeatMapper {
 public:
  PoseEstimationHeatMapper();
  virtual ~PoseEstimationHeatMapper() {}

  // Generates a heatmap, represented as a list of indices into the cloud
  // representing the scene and a parallel list of "importance" scores.
  virtual void Compute(pcl::PointIndicesPtr indices,
                       Eigen::VectorXd* importances) = 0;

  virtual void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) = 0;
  virtual void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object) = 0;
  void set_debug(bool val);
  void set_heatmap_publisher(const ros::Publisher& pub);

 protected:
  bool debug_;
  ros::Publisher heatmap_pub_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_HEAT_MAPPER_H_
