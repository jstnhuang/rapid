#include <iostream>
#include <string>
#include <vector>

#include "eigen_conversions/eigen_msg.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_ros/time.h"
#include "rapid_utils/pcl_typedefs.h"

void PrintUsage() {
  std::cout
      << "Transforms and publishes a saved point cloud from NAME.bag to "
         "/saved_cloud. NAME.bag follows the format used by rapid_perception"
      << std::endl;
  std::cout << "Usage: rosrun rapid_perception publish_save_cloud NAME.bag"
            << std::endl;
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_saved_cloud");
  if (argc < 2) {
    PrintUsage();
    return 0;
  }

  ros::NodeHandle nh;
  rapid::WaitForTime();

  rosbag::Bag bag;
  std::string bag_path(argv[1]);
  bag.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("/cloud");
  topics.push_back("/camera_in_base");
  rosbag::View view(bag);

  sensor_msgs::PointCloud2 cloud;
  geometry_msgs::TransformStamped camera_pose;
  bool set_cloud = false;
  bool set_camera_pose = false;
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    if (!set_cloud && it->getTopic() == "/cloud") {
      sensor_msgs::PointCloud2::ConstPtr temp =
          it->instantiate<sensor_msgs::PointCloud2>();
      ROS_ASSERT(temp != NULL);
      cloud = *temp;
      set_cloud = true;
    }
    if (!set_camera_pose && it->getTopic() == "/camera_in_base") {
      geometry_msgs::TransformStamped::ConstPtr temp;
      temp = it->instantiate<geometry_msgs::TransformStamped>();
      ROS_ASSERT(temp != NULL);
      camera_pose = *temp;
      set_camera_pose = true;
    }
    if (set_cloud && set_camera_pose) {
      break;
    }
  }
  if (!set_cloud || !set_camera_pose) {
    ROS_ERROR("Wrong bag format: missing either /cloud or /camera_in_base");
    return 1;
  }

  Eigen::Affine3d affine;
  tf::transformMsgToEigen(camera_pose.transform, affine);
  Eigen::Matrix4f cam_in_base_mat(affine.matrix().cast<float>());

  sensor_msgs::PointCloud2 cloud_in_base;
  pcl_ros::transformPointCloud(cam_in_base_mat, cloud, cloud_in_base);
  cloud_in_base.header.frame_id = camera_pose.header.frame_id;

  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("saved_cloud", 1);
  ros::Rate rate(10);
  while (ros::ok()) {
    cloud_pub.publish(cloud_in_base);
    rate.sleep();
  }

  return 0;
}
