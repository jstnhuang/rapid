#include "rapid/pr2/pr2.h"

#include "visualization_msgs/Marker.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "rapid/display/display.h"
#include "rapid/perception/rgbd.hpp"
#include "rapid/pr2/head.h"
#include "rapid/sound/sound.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

using visualization_msgs::Marker;

namespace rapid {
namespace pr2 {
Pr2::Pr2(rapid::display::DisplayInterface& display,
         rapid::pr2::HeadInterface& head, rapid::sound::SoundInterface& sound)
    : display(display), head(head), sound(sound) {}

bool Pr2::GetManipulationScene(rapid::perception::Scene* scene) {
  // Read message into pcl_cloud.
  tf::TransformListener tf_listener;
  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/head_mount_kinect/depth_registered/points");
  sensor_msgs::PointCloud2 transformed;
  bool success = tf_listener.waitForTransform(
      "/base_link", msg->header.frame_id, msg->header.stamp, ros::Duration(10));
  if (!success) {
    return false;
  }
  success = pcl_ros::transformPointCloud("/base_link", *msg, transformed,
                                         tf_listener);
  if (!success) {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pcl::fromROSMsg(transformed, pcl_cloud);

  // Crop the point cloud to the manipulation workspace area.
  visualization_msgs::Marker ws;
  rapid::pr2::GetManipulationWorkspace(&ws);
  pcl::PointCloud<pcl::PointXYZRGB> ws_cloud;
  rapid::perception::CropWorkspace(pcl_cloud, ws, &ws_cloud);

  scene->set_cloud(ws_cloud);
  scene->Parse();
  return true;
}

void GetManipulationWorkspace(Marker* ws) {
  // All measurements are in meters relative to base_link.
  // Since this is used for perception, we err on the side of a larger
  // workspace.
  double min_x = 0.2;  // PR2 can't reach closer than 20 cm in front.
  double max_x = 1.2;  // PR2 can't reach farther than 120 cm in front.
  double min_y = -1;   // PR2 can't reach farther than 100 cm right.
  double max_y = 1;    // PR2 can't reach farther than 100 cm left.
  double min_z = 0.3;  // PR2 can't reach lower than 30 cm above ground.
  double max_z = 1.7;  // PR2 can't reach higher than 170 cm above ground.
  ws->header.frame_id = "base_link";
  ws->type = Marker::CUBE;
  ws->pose.position.x = (max_x + min_x) / 2;
  ws->pose.position.y = (max_y + min_y) / 2;
  ws->pose.position.z = (max_z + min_z) / 2;
  ws->pose.orientation.w = 1;  // Assume axis-aligned box.
  ws->scale.x = max_x - min_x;
  ws->scale.y = max_y - min_y;
  ws->scale.z = max_z - min_z;
}

Pr2 BuildReal() {
  rapid::display::DisplayInterface* display = new rapid::display::Blinky();
  HeadInterface* head = new Head();
  rapid::sound::SoundInterface* sound = new rapid::sound::SoundPlay();
  Pr2 pr2(*display, *head, *sound);
  return pr2;
}
}  // namespace pr2
}  // namespace rapid
