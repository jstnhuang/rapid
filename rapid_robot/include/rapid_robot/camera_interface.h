#ifndef _RAPID_ROBOT_CAMERA_INTERFACE_H_
#define _RAPID_ROBOT_CAMERA_INTERFACE_H_

#include "geometry_msgs/TransformStamped.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
/// \brief Interface for a camera that outputs point clouds.
class PointCloudCameraInterface {
 public:
  virtual ~PointCloudCameraInterface() {}

  /// \brief Returns the most recent point cloud from the camera.
  /// \returns Returns the most recent point cloud from the camera.
  virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud() const = 0;

  /// \brief Returns the camera pose relative to the base frame.
  /// \returns The transform describing the camera frame with respect to the
  ///   base frame.
  virtual geometry_msgs::TransformStamped camera_pose() const = 0;
};
}  // namespace rapid

#endif  // _RAPID_ROBOT_CAMERA_INTERFACE_H_
