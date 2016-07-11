#include <string>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "opencv2/core/core.hpp"
#include "pbd_landmarks/GetLandmark.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"

using pbd_landmarks::Landmark;
using sensor_msgs::CameraInfo;
using sensor_msgs::Image;
using sensor_msgs::PointCloud2;

bool GetLandmark(const std::string& id, Landmark* landmark);
void CloudToImage(const CameraInfo& camera_info, const PointCloud2& cloud,
                  Image* image);

void CloudToImage(const CameraInfo& camera_info, const PointCloud2& cloud,
                  Image* image) {
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);
  // cv::Mat cv_image =
  //    cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC3);
  ROS_ERROR("Built pinhole model");
  cv::Mat cv_image =
      cv::Mat::zeros(camera_info.height, camera_info.width, CV_8UC3);
  ROS_ERROR("Built cv bridge");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(cloud, *pcl_cloud);
  ROS_ERROR("Built pcl cloud");
  for (size_t i = 0; i < pcl_cloud->size(); ++i) {
    const pcl::PointXYZRGB& point = pcl_cloud->points[i];
    cv::Point3d p3d(point.x, point.y, point.z);
    cv::Point2d p2d = model.project3dToPixel(p3d);
    int row = static_cast<int>(p2d.y);
    int col = static_cast<int>(p2d.x);
    cv_image.at<cv::Vec3b>(row, col)[0] = point.b;
    cv_image.at<cv::Vec3b>(row, col)[1] = point.g;
    cv_image.at<cv::Vec3b>(row, col)[2] = point.r;
  }
  ROS_ERROR("Copied data");
  cv_bridge::CvImage cv_bridge(
      cloud.header, sensor_msgs::image_encodings::TYPE_8UC3, cv_image);
  ROS_ERROR("Built CV bridge");
  cv_bridge.toImageMsg(*image);
  ROS_ERROR("returned");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_search");
  ros::NodeHandle nh;

  Landmark landmark;
  bool success = GetLandmark("577dca1e5163504a3d47a495", &landmark);
  if (!success) {
    return 1;
  }
  ROS_ERROR("Got landmark, %s", landmark.cloud.header.frame_id.c_str());

  CameraInfo::ConstPtr camera_info = ros::topic::waitForMessage<CameraInfo>(
      "camera_info", nh, ros::Duration(5));
  ROS_ERROR("Got camera info, %s", camera_info->header.frame_id.c_str());

  sensor_msgs::Image image_msg;
  CloudToImage(*camera_info, landmark.cloud, &image_msg);
  ros::Publisher image_pub =
      nh.advertise<sensor_msgs::Image>("landmark_image", 1, true);
  image_pub.publish(image_msg);

  ros::spin();
  return 0;
}

bool GetLandmark(const std::string& id, Landmark* landmark) {
  pbd_landmarks::GetLandmarkRequest req;
  req.id = "577dca1e5163504a3d47a495";
  pbd_landmarks::GetLandmarkResponse res;
  bool exists = ros::service::exists("get_landmark", true);
  if (!exists) {
    ROS_ERROR("GetLandmark service does not exist.");
    return false;
  }
  bool success = ros::service::call("get_landmark", req, res);
  if (!success) {
    ROS_ERROR("GetLandmark call failed.");
    return false;
  }
  if (res.error != "") {
    ROS_ERROR("GetLandmark call failed: %s", res.error.c_str());
    return false;
  }
  *landmark = res.landmark;
  return true;
}
