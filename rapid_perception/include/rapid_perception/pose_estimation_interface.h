#ifndef _RAPID_POSE_ESTIMATION_INTERFACE_H_
#define _RAPID_POSE_ESTIMATION_INTERFACE_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
namespace perception {
class PoseEstimationInterface {
 public:
  virtual ~PoseEstimationInterface();
  virtual void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene) = 0;
  virtual void set_object(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object) = 0;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_POSE_ESTIMATION_INTERFACE_H_
