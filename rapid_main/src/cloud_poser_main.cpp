// A test program demonstrating the use of rapid::viz::CloudPoser.

#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "ros/package.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"

#include "rapid_viz/cloud_poser.h"

void Load(const std::string& path, sensor_msgs::PointCloud2* cloud) {
  rosbag::Bag bag;
  bag.open(path, rosbag::bagmode::Read);
  std::vector<std::string> types;
  types.push_back("sensor_msgs/PointCloud2");

  rosbag::View view(bag, rosbag::TypeQuery(types));
  rosbag::View::iterator it = view.begin();
  for (; it != view.end(); ++it) {
    const rosbag::MessageInstance& mi = *it;
    sensor_msgs::PointCloud2::ConstPtr msg =
        mi.instantiate<sensor_msgs::PointCloud2>();
    *cloud = *msg;
    break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_poser_main");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  sensor_msgs::PointCloud2 cloud;
  Load(ros::package::getPath("rapid_perception") + "/test_data/table1.bag",
       &cloud);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>("cloud", 1, true);
  cloud_pub.publish(cloud);
  rapid::viz::CloudPoser poser(cloud, cloud_pub, "cloud_poser");

  bool exit = false;
  while (!exit) {
    poser.Start();

    std::cout << "Press enter to see the pose, or Ctrl+D to quit: ";
    std::string line;
    exit = !std::getline(std::cin, line);
    if (exit) {
      std::cout << std::endl;
      break;
    }

    poser.Stop();
    geometry_msgs::Pose pose = poser.pose();
    Eigen::Vector3f translation(pose.position.x, pose.position.y,
                                pose.position.z);
    Eigen::Quaternionf quaternion(pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z);
    std::cout << "Translation: " << std::endl
              << translation << std::endl;
    std::cout << "Orientation: " << std::endl
              << quaternion.matrix() << std::endl;
  }

  poser.Stop();
  spinner.stop();
  return 0;
}
