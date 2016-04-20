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

int main(int argc, char** argv) {
  ros::init(argc, argv, "record_kinect");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  sensor_msgs::PointCloud2ConstPtr cloud_in =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
  sensor_msgs::PointCloud2 cloud;
  std::string target_frame = "/base_footprint";
  ros::param::param<std::string>("record_kinect/target_frame", target_frame,
                                 "/base_footprint");

  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.waitForTransform(target_frame, cloud_in->header.frame_id,
                            ros::Time(0), ros::Duration(10));
  listener.lookupTransform(target_frame, cloud_in->header.frame_id,
                           ros::Time(0), transform);
  pcl_ros::transformPointCloud(target_frame, transform, *cloud_in, cloud);

  geometry_msgs::TransformStamped cam_to_base;
  tf::transformStampedTFToMsg(transform, cam_to_base);

  rosbag::Bag bag;
  bag.open("kinect_frame.bag", rosbag::bagmode::Write);
  bag.write("/cloud", ros::Time::now(), cloud);
  bag.write("/tf_camera_to_base", ros::Time::now(), cam_to_base);
  bag.close();

  spinner.stop();
  return 0;
}
