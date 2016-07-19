#ifndef _OBJECT_SEARCH_CAPTURE_ROI_H_
#define _OBJECT_SEARCH_CAPTURE_ROI_H_

#include "geometry_msgs/Transform.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rapid_msgs/CaptureRoi3D.h"
#include "rapid_perception/box3d_roi_server.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"

namespace object_search {
// Given a point cloud, runs an interactive marker server to capture a region of
// the cloud.
//
// Usage:
//  CaptureRoi capture(tf_listener);
//  capture.ShowMarker(); // Starts interactive marker server
//  // User adjusts marker in rviz
//  .. cloud = ros::topic::waitForMessage("cloud_in") ...
//  capture.set_cloud(cloud);
//  pcl::PointCloud output;
//  capture.Capture(&output); // Call to capture region inside marker with the
//                            // above cloud.
//  capture.HideMarker(); // Stop server.
class CaptureRoi {
 public:
  CaptureRoi(rapid::perception::Box3DRoiServer* marker_server);
  void ShowMarker();
  void HideMarker();
  void set_cloud(sensor_msgs::PointCloud2ConstPtr cloud);
  void Capture(sensor_msgs::PointCloud2* cloud_out);
  tf::StampedTransform cloud_to_base();

 private:
  tf::TransformListener tf_listener_;
  rapid::perception::Box3DRoiServer* marker_server_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
  tf::StampedTransform cloud_to_base_;
};
}  // namespace object_search

#endif  // _OBJECT_SEARCH_CAPTURE_ROI_H_
