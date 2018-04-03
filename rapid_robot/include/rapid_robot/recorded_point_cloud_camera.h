#ifndef _RAPID_ROBOT_RECORDED_POINT_CLOUD_CAMERA_H_
#define _RAPID_ROBOT_RECORDED_POINT_CLOUD_CAMERA_H_

#include "rapid_robot/camera_interface.h"

namespace rapid {
/// \brief A simulated camera that loads data from a bag file.
///
/// Specifically, the bag file should be in the format output by
/// rapid_perception's save_cloud executable:
/// - One sensor_msgs/PointCloud2 on the "/cloud" topic.
/// - One geometry_msgs/TransformStamped on the "/camera_in_base" topic.
///
/// Usage:
/// \code
/// RecordedPointCloudCamera sim_cam;
/// sim_cam.LoadBag(argv[1]);
/// sim_cam.cloud();
/// sim_cam.LoadBag(argv[2]);
/// sim_cam.cloud();
/// \endcode
class RecordedPointCloudCamera : public PointCloudCameraInterface {
 public:
  RecordedPointCloudCamera();
  /// \brief Loads a bag file saved by rapid_perception's save_cloud.
  ///
  /// This method can be called multiple times. Whatever cloud was saved will be
  /// returned in the next call to cloud(), and the same with camera_pose().
  ///
  /// If there is more than one message on either topic, only the first will be
  /// loaded.
  ///
  /// \param[in] path The file path to the bag file saved by save_cloud.
  void LoadBag(const std::string& path);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud() const;
  geometry_msgs::TransformStamped camera_pose() const;

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  geometry_msgs::TransformStamped camera_pose_;
};
}  // namespace rapid

#endif  // _RAPID_ROBOT_RECORDED_POINT_CLOUD_CAMERA_H_
