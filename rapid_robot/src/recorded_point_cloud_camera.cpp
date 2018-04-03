#include "rapid_robot/recorded_point_cloud_camera.h"

#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"

namespace rapid {
RecordedPointCloudCamera::RecordedPointCloudCamera() {}

void RecordedPointCloudCamera::LoadBag(const std::string& path) {
  cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/cloud");
  topics.push_back("/camera_in_base");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  bool set_cloud = false;
  bool set_camera_pose = false;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (!set_cloud && it->getTopic() == "/cloud") {
      sensor_msgs::PointCloud2::ConstPtr ros_cloud =
          it->instantiate<sensor_msgs::PointCloud2>();
      ROS_ASSERT(ros_cloud != NULL);
      pcl::fromROSMsg(*ros_cloud, *cloud_);
      set_cloud = true;
    }
    if (!set_camera_pose && it->getTopic() == "/camera_in_base") {
      geometry_msgs::TransformStamped::ConstPtr temp;
      temp = it->instantiate<geometry_msgs::TransformStamped>();
      ROS_ASSERT(temp != NULL);
      camera_pose_ = *temp;
      set_camera_pose = true;
    }
    if (set_cloud && set_camera_pose) {
      break;
    }
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RecordedPointCloudCamera::cloud() const {
  return cloud_;
}

geometry_msgs::TransformStamped RecordedPointCloudCamera::camera_pose() const {
  return camera_pose_;
}
}  // namespace rapid
