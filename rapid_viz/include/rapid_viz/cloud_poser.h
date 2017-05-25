#ifndef _RAPID_VIZ_CLOUD_POSER_H_
#define _RAPID_VIZ_CLOUD_POSER_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "interactive_markers/interactive_marker_server.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "transform_graph/graph.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"

namespace rapid {
namespace viz {
// CloudPoser is a server for visually adjusting the pose of a point cloud,
// which is useful for manually aligning the point cloud with another.
//
// To use CloudPoser, provide the point cloud and publishers for the point cloud
// and interactive marker:
//
// CloudPoser poser(cloud, cloud_pub, "cloud_poser");
//
// For best results, the cloud should be supplied in base frame. The interactive
// marker to position the object will be in the same frame as the cloud.
//
// You can set an initial pose offset (in the same frame as the point cloud):
// poser.set_pose(pose);
// poser.Start();
//
// You can stop the server to hide the visualization.
// poser.Stop();
//
// You can read the pose offset at any time. The pose offset is the transform
// relative to the cloud's pose when it was passed to set_cloud.
//
// geometry_msgs::Pose pose_out;
// poser.GetPose(&pose_out);
class CloudPoser {
 public:
  CloudPoser(const sensor_msgs::PointCloud2& cloud,
             const ros::Publisher& cloud_pub, const std::string& im_topic);
  static std::string int_marker_name();

  void Start();
  void Stop();

  geometry_msgs::Pose pose() const;
  void set_pose(const geometry_msgs::Pose& pose);

 private:
  void ProcessFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void Update();

  sensor_msgs::PointCloud2 cloud_;
  ros::Publisher cloud_pub_;  // Point cloud publisher
  interactive_markers::InteractiveMarkerServer im_server_;

  visualization_msgs::InteractiveMarker int_marker_;  // The interactive marker.
  geometry_msgs::Pose pose_;  // The current pose of the marker, which is
                              // positioned at the center of the point cloud.
  double max_dim_;  // The length of the maximum dimension of the point cloud.
  transform_graph::Graph graph_;
};
}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_CLOUD_POSER_H_
