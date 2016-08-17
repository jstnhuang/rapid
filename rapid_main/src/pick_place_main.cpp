#include <iostream>
#include <string>
#include <vector>

#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid_manipulation/arm.h"
#include "rapid_manipulation/gripper.h"
#include "rapid_manipulation/pick.h"
#include "rapid_manipulation/place.h"
#include "rapid_perception/pr2.h"
#include "rapid_perception/rgbd.h"
#include "rapid_perception/scene.h"
#include "rapid_perception/scene_viz.h"
#include "rapid_perception/scene_parsing.h"
#include "rapid_pr2/pr2.h"
#include "rapid_ros/publisher.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

using pcl::PointCloud;
using pcl::PointXYZRGB;
using rapid::manipulation::ArmInterface;
using rapid::manipulation::MoveItArm;
using rapid::manipulation::GripperInterface;
using rapid::manipulation::Gripper;
using rapid::perception::Object;
using rapid::perception::Pr2Params;
using rapid::perception::Scene;
using rapid::perception::SceneViz;
using rapid::pr2::Pr2;
using rapid_ros::Publisher;
using std::string;
using std::vector;
using visualization_msgs::Marker;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  tf::TransformListener tf_listener;
  Pr2* pr2 = rapid::pr2::BuildReal(nh);

  bool first = true;
  while (true) {
    if (!first) {
      string response;
      std::cout << "Regrasp (y/n)? ";
      std::cin >> response;
      if (response != "y") {
        break;
      }
    }
    first = false;

    // Read point cloud.
    pr2->tuck_arms()->DeployArms();
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    sensor_msgs::PointCloud2ConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/cloud_in");
    sensor_msgs::PointCloud2 transformed;
    pcl_ros::transformPointCloud("/base_footprint", *msg, transformed,
                                 tf_listener);
    pcl::fromROSMsg(transformed, *cloud);

    // Parse scene
    Scene scene;
    ros::Publisher marker_ros_pub =
        nh.advertise<Marker>("/code_it_markers", 100);
    Publisher<Marker>* marker_pub = new Publisher<Marker>(marker_ros_pub);
    SceneViz viz(marker_pub);
    rapid::perception::ParseScene(cloud, Pr2Params(), &scene);
    viz.set_scene(scene);
    viz.Visualize();
    const vector<Object>& objects = scene.primary_surface().objects();
    if (objects.size() == 0) {
      ROS_ERROR("No objects found.");
      pr2->tuck_arms()->DeployArms();
      continue;
    } else {
      ROS_INFO("Found %ld objects", objects.size());
    }

    // Pick up first object and place it somewhere else.
    Object first_obj = objects[0];
    ROS_INFO("Attempting to pick up %s", first_obj.name().c_str());
    bool success = pr2->right_picker()->Pick(first_obj, 30);
    if (!success) {
      pr2->right_gripper()->Open();
      pr2->tuck_arms()->DeployArms();
      continue;
    }
    pr2->tuck_arms()->DeployArms();

    success = pr2->right_placer()->Place(first_obj, scene.primary_surface());
    if (!success) {
      pr2->right_gripper()->Open();
      pr2->tuck_arms()->DeployArms();
      ROS_ERROR("Place failed.");
      continue;
    } else {
      ROS_INFO("Place succeeded.");
    }
  }
  return 0;
}
