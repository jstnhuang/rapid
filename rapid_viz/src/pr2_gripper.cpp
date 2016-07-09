#include "rapid_viz/pr2_gripper.h"

#include <math.h>
#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/ColorRGBA.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/Marker.h"

#include "rapid_viz/markers.h"

namespace rapid {
namespace viz {
visualization_msgs::InteractiveMarker Pr2GripperMarker(
    const std::string& name, const geometry_msgs::PoseStamped& ps,
    double openness) {
  std_msgs::ColorRGBA blank;
  return Pr2GripperMarker(name, ps, openness, blank);
}

visualization_msgs::InteractiveMarker Pr2GripperMarker(
    const std::string& name, const geometry_msgs::PoseStamped& ps,
    double openness, std_msgs::ColorRGBA& color) {
  double angle = openness * PR2_GRIPPER_MAX_RADIANS;
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header = ps.header;
  int_marker.name = name;
  int_marker.scale = 1.0;
  int_marker.pose = ps.pose;

  visualization_msgs::InteractiveMarkerControl control;

  visualization_msgs::Marker mesh;
  mesh.mesh_use_embedded_materials = true;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.scale.x = 1;
  mesh.scale.y = 1;
  mesh.scale.z = 1;
  mesh.color = color;

  tf::Transform T1, T2;
  tf::Transform T_proximal, T_distal;

  T1.setOrigin(tf::Vector3(0.07691, 0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = PR2_GRIPPER_PALM_URI;
  mesh.pose.orientation.w = 1;
  control.markers.push_back(mesh);

  mesh.mesh_resource = PR2_GRIPPER_FINGER_URI;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(T_proximal, pose);
  mesh.pose = pose;
  control.markers.push_back(mesh);

  mesh.mesh_resource = PR2_GRIPPER_FINGERTIP_URI;
  tf::poseTFToMsg(T_distal, pose);
  mesh.pose = pose;
  control.markers.push_back(mesh);

  T1.setOrigin(tf::Vector3(0.07691, -0.01, 0));
  T1.setRotation(tf::Quaternion(tf::Vector3(1, 0, 0), M_PI) *
                 tf::Quaternion(tf::Vector3(0, 0, 1), angle));
  T2.setOrigin(tf::Vector3(0.09137, 0.00495, 0));
  T2.setRotation(tf::Quaternion(tf::Vector3(0, 0, 1), -angle));
  T_proximal = T1;
  T_distal = T1 * T2;

  mesh.mesh_resource = PR2_GRIPPER_FINGER_URI;
  tf::poseTFToMsg(T_proximal, pose);
  mesh.pose = pose;
  control.markers.push_back(mesh);
  mesh.mesh_resource = PR2_GRIPPER_FINGERTIP_URI;
  tf::poseTFToMsg(T_distal, pose);
  mesh.pose = pose;
  control.markers.push_back(mesh);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  int_marker.controls.push_back(control);

  return int_marker;
}
}  // namespace viz
}  // namespace rapid
