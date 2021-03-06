#include "rapid_viz/cloud_poser.h"

#include <algorithm>
#include <iostream>
#include <string>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "boost/bind.hpp"
#include "geometry_msgs/Pose.h"
#include "interactive_markers/interactive_marker_server.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "transform_graph/transform_graph.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerControl.h"

#include "rapid_viz/publish.h"

using visualization_msgs::InteractiveMarker;
using visualization_msgs::InteractiveMarkerControl;

namespace rapid {
namespace viz {
namespace {
void MakeMoveAxisControl(
    const std::string& axis,
    visualization_msgs::InteractiveMarkerControl* control) {
  if (axis != "x" && axis != "y" && axis != "z") {
    ROS_ERROR("Cannot create control with axis: %s", axis.c_str());
    return;
  }
  control->name = "move_" + axis;
  control->orientation.w = 1;
  if (axis == "x") {
    control->orientation.x = 1;
  } else if (axis == "y") {
    control->orientation.y = 1;
  } else if (axis == "z") {
    control->orientation.z = 1;
  }
  control->orientation_mode = InteractiveMarkerControl::FIXED;
  control->interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  control->always_visible = false;
}

void MakeRotateAxisControl(
    const std::string& axis,
    visualization_msgs::InteractiveMarkerControl* control) {
  if (axis != "x" && axis != "y" && axis != "z") {
    ROS_ERROR("Cannot create control with axis: %s", axis.c_str());
    return;
  }
  control->name = "move_" + axis;
  control->orientation.w = 1;
  if (axis == "x") {
    control->orientation.x = 1;
  } else if (axis == "y") {
    control->orientation.y = 1;
  } else if (axis == "z") {
    control->orientation.z = 1;
  }
  control->orientation_mode = InteractiveMarkerControl::FIXED;
  control->interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  control->always_visible = false;
}

void Make6DofMarker(const std::string& frame_id, double scale,
                    const geometry_msgs::Pose& pose,
                    InteractiveMarker* int_marker) {
  int_marker->header.frame_id = frame_id;
  int_marker->pose = pose;
  int_marker->name = CloudPoser::int_marker_name();
  int_marker->scale = scale;

  InteractiveMarkerControl move_x_control;
  MakeMoveAxisControl("x", &move_x_control);
  InteractiveMarkerControl move_y_control;
  MakeMoveAxisControl("y", &move_y_control);
  InteractiveMarkerControl move_z_control;
  MakeMoveAxisControl("z", &move_z_control);
  InteractiveMarkerControl rotate_x_control;
  MakeRotateAxisControl("x", &rotate_x_control);
  InteractiveMarkerControl rotate_y_control;
  MakeRotateAxisControl("y", &rotate_y_control);
  InteractiveMarkerControl rotate_z_control;
  MakeRotateAxisControl("z", &rotate_z_control);

  int_marker->controls.push_back(move_x_control);
  int_marker->controls.push_back(move_y_control);
  int_marker->controls.push_back(move_z_control);
  int_marker->controls.push_back(rotate_x_control);
  int_marker->controls.push_back(rotate_y_control);
  int_marker->controls.push_back(rotate_z_control);
}
}  // namespace

CloudPoser::CloudPoser(const sensor_msgs::PointCloud2& cloud,
                       const ros::Publisher& cloud_pub,
                       const std::string& im_topic)
    : cloud_(),
      cloud_pub_(cloud_pub),
      im_server_(im_topic, "", true),
      int_marker_(),
      pose_(),
      max_dim_(0),
      graph_() {
  pose_.orientation.w = 1;

  // Compute center and size.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(cloud, *pcl_cloud);
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(*pcl_cloud, min_pt, max_pt);
  geometry_msgs::Pose center;
  center.orientation.w = 1;
  center.position.x = (max_pt.x() + min_pt.x()) / 2;
  center.position.y = (max_pt.y() + min_pt.y()) / 2;
  center.position.z = (max_pt.z() + min_pt.z()) / 2;
  double x_length = (max_pt.x() - min_pt.x());
  double y_length = (max_pt.y() - min_pt.y());
  double z_length = (max_pt.z() - min_pt.z());
  max_dim_ = std::max(x_length, std::max(y_length, z_length));
  graph_.Add("original", transform_graph::RefFrame(cloud.header.frame_id),
             center);
  pose_ = center;

  // Shift the cloud so that the center of its bounding box is at the origin.
  Eigen::Vector4f centroid = (max_pt + min_pt) / 2;
  pcl::demeanPointCloud(*pcl_cloud, centroid, *pcl_cloud);
  pcl::toROSMsg(*pcl_cloud, cloud_);
}

std::string CloudPoser::int_marker_name() { return "cloud_poser_6dof"; }

void CloudPoser::Start() {
  Make6DofMarker(cloud_.header.frame_id, max_dim_, pose_, &int_marker_);

  im_server_.clear();
  im_server_.insert(int_marker_);
  im_server_.setCallback(int_marker_.name,
                         boost::bind(&CloudPoser::ProcessFeedback, this, _1));
  im_server_.applyChanges();
  Update();
}

void CloudPoser::Stop() {
  im_server_.clear();
  im_server_.applyChanges();
  viz::PublishBlankCloud(cloud_pub_, cloud_.header.frame_id);
}

geometry_msgs::Pose CloudPoser::pose() const { return pose_; }

void CloudPoser::set_pose(const geometry_msgs::Pose& pose) {
  pose_ = pose;
  im_server_.setPose(int_marker_name(), pose_);
  im_server_.applyChanges();
  Update();
}

void CloudPoser::ProcessFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  if (feedback->event_type ==
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
    pose_ = feedback->pose;
    Update();
  }
  im_server_.applyChanges();
}

// Update when the pose changes.
void CloudPoser::Update() {
  im_server_.setPose(int_marker_name(), pose_);
  im_server_.applyChanges();

  graph_.Add("current", transform_graph::RefFrame(cloud_.header.frame_id),
             pose_);
  transform_graph::Transform transform;
  graph_.ComputeMapping(transform_graph::From(cloud_.header.frame_id),
                        transform_graph::To("current"), &transform);

  sensor_msgs::PointCloud2 cloud_out;
  pcl_ros::transformPointCloud(transform.matrix().cast<float>(), cloud_,
                               cloud_out);
  viz::PublishCloud(cloud_pub_, cloud_out);
}
}  // namespace viz
}  // namespace rapid
