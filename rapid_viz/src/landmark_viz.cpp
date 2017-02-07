#include "rapid_viz/landmark_viz.h"

#include "geometry_msgs/PoseStamped.h"
#include "pcl_ros/transforms.h"
#include "stf/stf.h"

#include "rapid_viz/publish.h"

namespace rapid {
namespace viz {
LandmarkViz::LandmarkViz(const ros::Publisher& cloud_pub,
                         const ros::Publisher& marker_pub)
    : cloud_pub_(cloud_pub),
      base_frame_("base_link"),
      cloud_(),
      roi_(),
      pose_(),
      marker_pub_(marker_pub),
      box_marker_(Marker::Null()) {}

LandmarkViz::LandmarkViz(const ros::Publisher& cloud_pub,
                         const ros::Publisher& marker_pub,
                         const std::string& base_frame)
    : cloud_pub_(cloud_pub),
      base_frame_(base_frame),
      cloud_(),
      roi_(),
      pose_(),
      marker_pub_(marker_pub),
      box_marker_(Marker::Null()) {}

void LandmarkViz::set_landmark(const sensor_msgs::PointCloud2& cloud,
                               const rapid_msgs::Roi3D& roi) {
  cloud_ = cloud;
  roi_ = roi;
  pose_.position.x = roi.transform.translation.x;
  pose_.position.y = roi.transform.translation.y;
  pose_.position.z = roi.transform.translation.z;
  pose_.orientation = roi.transform.rotation;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = base_frame_;
  ps.pose = pose_;
  box_marker_ = Marker::OutlineBox(&marker_pub_, ps, roi_.dimensions);
}

void LandmarkViz::UpdatePose(const geometry_msgs::Pose& pose) {
  roi_.transform.translation.x = pose.position.x;
  roi_.transform.translation.y = pose.position.y;
  roi_.transform.translation.z = pose.position.z;
  roi_.transform.rotation = pose.orientation;

  // Transform point cloud
  stf::Graph graph;
  graph.Add("previous", stf::RefFrame("base_link"), pose_);
  graph.Add("current", stf::RefFrame("base_link"), pose);
  stf::Transform prev_to_new;
  graph.ComputeMapping(stf::From("previous"), stf::To("current"), &prev_to_new);
  sensor_msgs::PointCloud2 out;
  pcl_ros::transformPointCloud(prev_to_new.matrix().cast<float>(), cloud_, out);
  cloud_ = out;

  pose_ = pose;
}

void LandmarkViz::Publish() {
  sensor_msgs::PointCloud2 transformed_cloud;

  PublishCloud(cloud_pub_, cloud_);
  box_marker_.Publish();
}

void LandmarkViz::Clear() {
  PublishBlankCloud(cloud_pub_);
  box_marker_.Delete();
}
}  // namespace viz
}  // namespace rapid
