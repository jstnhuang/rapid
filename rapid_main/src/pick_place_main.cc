#include <iostream>
#include <string>

#include "boost/shared_ptr.hpp"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_manipulation/pick_place.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.hpp"
#include "rapid_perception/scene.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

using boost::shared_ptr;
using rapid::perception::Scene;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;

  // Read point cloud.
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  tf::TransformListener tf_listener;
  shared_ptr<moveit::planning_interface::MoveGroup> right_arm(
      new moveit::planning_interface::MoveGroup("right_arm"));

  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cloud_in");
  sensor_msgs::PointCloud2 transformed;
  pcl_ros::transformPointCloud("/base_link", *msg, transformed, tf_listener);
  pcl::fromROSMsg(transformed, cloud);

  // Crop point cloud
  visualization_msgs::Marker ws;
  rapid::perception::pr2::GetManipulationWorkspace(&ws);
  pcl::PointCloud<pcl::PointXYZRGB> ws_cloud;
  rapid::perception::CropWorkspace(cloud, ws, &ws_cloud);

  Scene scene;
  scene.set_cloud(ws_cloud);
  scene.Parse();

  rapid::manipulation::Picker picker(right_arm);
  picker.UpdatePlanningScene(scene);
  ROS_INFO("Updated planning scene with %ld objects",
           scene.GetPrimarySurface()->objects().size());

  ros::spin();
  return 0;
}
