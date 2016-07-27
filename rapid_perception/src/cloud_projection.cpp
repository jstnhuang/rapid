#include "rapid_perception/cloud_projection.h"

#include <iostream>
#include <limits.h>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/core/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "sensor_msgs/CameraInfo.h"

namespace rapid {
namespace perception {
void CloudToImage(const sensor_msgs::CameraInfo& camera_info,
                  const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  const pcl::PointIndices::Ptr& indices, cv::Mat* image) {
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);
  cv::Mat output =
      cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC3);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
  //    new pcl::PointCloud<pcl::PointXYZRGB>);
  int min_row = std::numeric_limits<int>::max();
  int max_row = std::numeric_limits<int>::min();
  int min_col = std::numeric_limits<int>::max();
  int max_col = std::numeric_limits<int>::min();
  size_t size = cloud.size();
  if (indices) {
    size = indices->indices.size();
  }
  for (size_t i = 0; i < size; ++i) {
    int index = i;
    if (indices) {
      index = indices->indices[i];
    }
    const pcl::PointXYZRGB& point = cloud.points[index];
    cv::Point3d p3d(point.x, point.y, point.z);
    std::cout << "index: " << index << ", x: " << point.x << ", y: " << point.y
              << ", z: " << point.z << std::endl;
    cv::Point2d p2d = model.project3dToPixel(p3d);
    int row = static_cast<int>(p2d.y);
    int col = static_cast<int>(p2d.x);
    if (row > max_row) {
      max_row = row;
    }
    if (row < min_row) {
      min_row = row;
    }
    if (col > max_col) {
      max_col = col;
    }
    if (col < min_col) {
      min_col = col;
    }
    output.at<cv::Vec3b>(row, col)[0] = point.b;
    output.at<cv::Vec3b>(row, col)[1] = point.g;
    output.at<cv::Vec3b>(row, col)[2] = point.r;
  }
  // Crop image to non-zero boundaries
  cv::Rect roi(min_col, min_row, max_col - min_col, max_row - min_row);
  cv::Mat cropped = output(roi);
  *image = cropped;

  // cv_bridge::CvImage cv_bridge(
  //    cloud.header, sensor_msgs::image_encodings::TYPE_8UC3, cv_image);
  // cv_bridge.toImageMsg(*image);
}

void CloudToImage(const sensor_msgs::CameraInfo& camera_info,
                  const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  cv::Mat* image) {
  pcl::PointIndices::Ptr indices;
  CloudToImage(camera_info, cloud, indices, image);
}

sensor_msgs::CameraInfo HeadMountKinectCameraInfo() {
  sensor_msgs::CameraInfo info;
  info.header.frame_id = "/head_mount_kinect_rgb_optical_frame";
  info.height = 480;
  info.width = 640;
  info.distortion_model = "plumb_bob";
  info.D.resize(5, 0);
  info.K.fill(0);
  info.K[0] = 525;
  info.K[2] = 319.5;
  info.K[4] = 525;
  info.K[5] = 239.5;
  info.K[8] = 1;
  info.R.fill(0);
  info.R[0] = 1;
  info.R[4] = 1;
  info.R[8] = 1;
  info.P.fill(0);
  info.P[0] = 525;
  info.P[2] = 319.5;
  info.P[5] = 525;
  info.P[6] = 239.5;
  info.P[10] = 1;
  return info;
}
}  // namespace perception
}  // namespace rapid
