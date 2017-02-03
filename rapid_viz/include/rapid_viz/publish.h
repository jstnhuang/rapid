// Containers functions to publish common data formats.

#ifndef _RAPID_VIZ_PUBLISH_H_
#define _RAPID_VIZ_PUBLISH_H_

#include "opencv2/core/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace rapid {
namespace viz {
// Publish a blank point cloud to "erase" a point cloud display on a latched
// topic. If your task context doesn't have a base_link frame, set frame_id to
// whatever frame you can visualize with.
void PublishBlankCloud(const ros::Publisher& pub,
                       const std::string& frame_id = "base_link");
// Publish a point cloud using the given publisher.
void PublishCloud(const ros::Publisher& pub,
                  const sensor_msgs::PointCloud2& cloud);
void PublishCloud(const ros::Publisher& pub,
                  pcl::PointCloud<pcl::PointXYZRGB>& cloud);
void PublishCloud(const ros::Publisher& pub,
                  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);
void PublishImage(const ros::Publisher& pub, const pcl::PCLHeader& cloud_header,
                  const cv::Mat& mat);

}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_PUBLISH_H_
