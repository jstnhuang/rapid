#include "rapid_viz/publish.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZRGBNormal PointN;
typedef pcl::PointCloud<PointC> PointCloudC;
typedef pcl::PointCloud<PointN> PointCloudN;

namespace rapid {
namespace viz {
void PublishBlankCloud(const ros::Publisher& pub, const std::string& frame_id) {
  PointCloudC blank;
  PointC blank_pt;
  blank_pt.a = 0;
  blank.push_back(blank_pt);
  blank.header.frame_id = frame_id;
  PublishCloud(pub, blank);
}

void PublishCloud(const ros::Publisher& pub,
                  const sensor_msgs::PointCloud2& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 cloud_now = cloud;
    cloud_now.header.stamp = ros::Time::now();
    pub.publish(cloud);
  }
}

void PublishCloud(const ros::Publisher& pub, PointCloudC& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }
}

void PublishCloud(const ros::Publisher& pub, PointCloudN& cloud) {
  if (pub) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
  }
}

void PublishImage(const ros::Publisher& pub, const pcl::PCLHeader& cloud_header,
                  const cv::Mat& mat) {
  if (pub) {
    std_msgs::Header header;
    header.frame_id = cloud_header.frame_id;
    cv_bridge::CvImage cv_bridge(header,
                                 sensor_msgs::image_encodings::TYPE_8UC3, mat);
    sensor_msgs::Image image_msg;
    cv_bridge.toImageMsg(image_msg);
    pub.publish(image_msg);
  }
}
}  // namespace viz
}  // namespace rapid
