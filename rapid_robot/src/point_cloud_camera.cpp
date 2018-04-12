#include "rapid_robot/point_cloud_camera.h"

#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"

namespace rapid {
PointCloudCamera::PointCloudCamera(const std::string& topic,
                                   const std::string& base_link)
    : topic_(topic),
      base_link_(base_link),
      tf_listener_(),
      camera_frame_id_("") {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudCamera::cloud() const {
  const ros::Time start = ros::Time::now();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  while (ros::ok()) {
    sensor_msgs::PointCloud2::ConstPtr ros_cloud =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_,
                                                             ros::Duration(5));
    if (!ros_cloud) {
      ROS_WARN("Waiting for point cloud on topic %s", topic_.c_str());
    } else if (ros_cloud->header.stamp < start) {
      ROS_WARN("Got old point cloud! Requested time: %f, actual: %f",
               start.toSec(), ros_cloud->header.stamp.toSec());
      camera_frame_id_ = ros_cloud->header.frame_id;
    } else {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*ros_cloud, *cloud);
      camera_frame_id_ = ros_cloud->header.frame_id;
      break;
    }
  }
  return cloud;
}

geometry_msgs::TransformStamped PointCloudCamera::camera_pose() const {
  if (camera_frame_id_ == "") {
    cloud();
  }
  tf_listener_.waitForTransform(base_link_, camera_frame_id_, ros::Time(0),
                                ros::Duration(5));
  tf::StampedTransform stamped_transform;
  tf_listener_.lookupTransform(base_link_, camera_frame_id_, ros::Time(0),
                               stamped_transform);
  geometry_msgs::TransformStamped result;
  tf::transformStampedTFToMsg(stamped_transform, result);
  return result;
}
}  // namespace rapid
