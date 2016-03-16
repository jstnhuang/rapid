#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "record_kinect");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  sensor_msgs::PointCloud2ConstPtr cloud_in =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/camera/depth_registered/points");

  sensor_msgs::PointCloud2 cloud;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform("/world", cloud_in->header.frame_id, ros::Time(0),
                            ros::Duration(10));
  listener.lookupTransform("/world", cloud_in->header.frame_id, ros::Time(0),
                           transform);
  pcl_ros::transformPointCloud("/world", transform, *cloud_in, cloud);

  ROS_INFO("Recorded point cloud with %ld points", cloud.data.size());

  rosbag::Bag bag;
  bag.open("kinect_frame.bag", rosbag::bagmode::Write);
  bag.write("/camera/depth_registered/points", ros::Time::now(), cloud);
  bag.close();

  spinner.stop();

  return 0;
}
