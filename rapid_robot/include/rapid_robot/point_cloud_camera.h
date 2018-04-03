#ifndef _RAPID_ROBOT_POINT_CLOUD_CAMERA_H_
#define _RAPID_ROBOT_POINT_CLOUD_CAMERA_H_

#include <string>

#include "rapid_robot/camera_interface.h"
#include "tf/transform_listener.h"

namespace rapid {
/// \brief Gets point cloud and camera pose.
///
/// Usage:
/// \code
/// #include "rapid_robot/point_cloud_camera.h"
///
/// PointCloudCamera pr2_camera("/head_mount_kinect/depth_registered/points",
///   "base_link");
/// pr2_camera.cloud();
/// pr2_camera.camera_pose();
/// \endcode
///
/// \note This uses ros::topic::waitForMessage, which may not be as fast as
/// directly subscribing to a point cloud topic.
class PointCloudCamera : public PointCloudCameraInterface {
 public:
  /// \brief Constructor
  ///
  /// \param[in] topic The topic to get point clouds from.
  /// \param[in] base_link The name of the base frame.
  explicit PointCloudCamera(const std::string& topic,
                            const std::string& base_link);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud() const;

  geometry_msgs::TransformStamped camera_pose() const;

 private:
  std::string topic_;
  mutable std::string base_link_;
  tf::TransformListener tf_listener_;

  mutable std::string camera_frame_id_;
};
}  // namespace rapid

#endif  // _RAPID_ROBOT_POINT_CLOUD_CAMERA_H_
