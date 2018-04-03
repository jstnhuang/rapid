#include <iostream>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

#include "rapid_ros/time.h"

void PrintUsage() {
  std::cout
      << "Saves a point cloud on cloud_in to NAME.bag in the current directory."
      << std::endl
      << "Also saves the transform to the base frame (Default: base_link)."
      << std::endl;
  std::cout << "Usage: rosrun perception save_cloud NAME" << std::endl;
  std::cout << "       rosrun perception save_cloud NAME BASE_FRAME"
            << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "save_cloud");
  if (argc < 2) {
    PrintUsage();
    return 0;
  }
  std::string bag_name(argv[1]);

  ros::NodeHandle nh;
  rapid::WaitForTime();

  const std::string resolved_topic(ros::names::resolve("cloud_in"));
  if (resolved_topic == "/cloud_in") {
    ROS_WARN("Did you forget to remap cloud_in?");
  }

  // Read point cloud
  sensor_msgs::PointCloud2ConstPtr cloud_in;
  const ros::Time now = ros::Time::now();
  while (ros::ok()) {
    cloud_in =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(resolved_topic);
    if (cloud_in->header.stamp >= now) {
      break;
    } else {
      ROS_WARN_THROTTLE(1, "Got old point cloud at time %f, waiting for %f",
                        cloud_in->header.stamp.toSec(), now.toSec());
    }
  }

  // Read transform from camera frame to base frame
  std::string target_frame("/base_link");
  if (argc >= 3) {
    target_frame = argv[2];
  }

  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform(target_frame, cloud_in->header.frame_id,
                            ros::Time(0), ros::Duration(10));
  listener.lookupTransform(target_frame, cloud_in->header.frame_id,
                           ros::Time(0), transform);

  geometry_msgs::TransformStamped cam_to_base;
  tf::transformStampedTFToMsg(transform, cam_to_base);

  // Save data
  rosbag::Bag bag;
  bag.open(bag_name + ".bag", rosbag::bagmode::Write);
  bag.write("/cloud", ros::Time::now(), cloud_in);
  bag.write("/camera_in_base", ros::Time::now(), cam_to_base);
  bag.close();

  return 0;
}
