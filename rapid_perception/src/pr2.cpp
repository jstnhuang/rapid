// Contains PR2-specific perception information.
// TODO: refactor this in a better way.

#include "rapid_perception/pr2.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include "rapid_perception/scene_parsing.h"

using visualization_msgs::Marker;

namespace rapid {
namespace perception {
namespace pr2 {
bool GetManipulationScene(const tf::TransformListener& tf_listener,
                          rapid::perception::Scene* scene) {
  // Read message into pcl_cloud.
  sensor_msgs::PointCloud2ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          "/head_mount_kinect/depth_registered/points");
  sensor_msgs::PointCloud2 transformed;
  bool success = pcl_ros::transformPointCloud("/base_footprint", *msg,
                                              transformed, tf_listener);
  if (!success) {
    ROS_ERROR("Failed to transform point cloud.");
    return false;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(transformed, *pcl_cloud);

  // scene->Visualize();
  return ParseScene(pcl_cloud, Pr2Params(), scene);
}
}  // namespace pr2
}  // namespace perception
}  // namespace rapid
