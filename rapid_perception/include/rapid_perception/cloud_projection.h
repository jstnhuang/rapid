#ifndef _RAPID_PERCEPTION_CLOUD_PROJECTION_H_
#define _RAPID_PERCEPTION_CLOUD_PROJECTION_H_

#include "opencv2/core/core.hpp"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/CameraInfo.h"

namespace rapid {
namespace perception {
// Projects the given point cloud onto the image plane.
void CloudToImage(const sensor_msgs::CameraInfo& camera_info,
                  const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  cv::Mat* image);
void CloudToImage(const sensor_msgs::CameraInfo& camera_info,
                  const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  const pcl::PointIndices::Ptr& indices, cv::Mat* image);

// Returns the standard CameraInfo value for the head-mounted Kinect 360 on the
// PR2.
sensor_msgs::CameraInfo HeadMountKinectCameraInfo();
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_CLOUD_PROJECTION_H_
