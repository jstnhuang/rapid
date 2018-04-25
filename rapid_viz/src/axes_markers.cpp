#include "rapid_viz/axes_markers.h"

#include "Eigen/Dense"

namespace rapid {
visualization_msgs::MarkerArray AxesMarkerArray(const std::string& ns,
                                                const std::string& frame_id,
                                                const geometry_msgs::Pose& pose,
                                                double scale) {
  const double kDiameter = std::max(scale * 0.1, 0.001);

  visualization_msgs::Marker x_axis;
  x_axis.ns = ns;
  x_axis.header.frame_id = frame_id;
  x_axis.id = 0;
  x_axis.type = visualization_msgs::Marker::CYLINDER;
  x_axis.scale.x = kDiameter;
  x_axis.scale.y = kDiameter;
  x_axis.scale.z = scale;
  x_axis.color.r = 1.0;
  x_axis.color.a = 1.0;

  Eigen::Matrix3f rotation_matrix(
      Eigen::Quaternionf(pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z));
  // X axis orientation
  Eigen::Matrix3f x_matrix;
  x_matrix.col(0) = rotation_matrix.col(2) * -1.0;
  x_matrix.col(1) = rotation_matrix.col(1);
  x_matrix.col(2) = rotation_matrix.col(0);
  Eigen::Quaternionf x_quaternion(x_matrix);
  x_axis.pose.orientation.x = x_quaternion.x();
  x_axis.pose.orientation.y = x_quaternion.y();
  x_axis.pose.orientation.z = x_quaternion.z();
  x_axis.pose.orientation.w = x_quaternion.w();

  // Shift x axis position a bit to relocate the cylinder
  Eigen::Vector3f x_position =
      Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z) +
      x_matrix.col(2) * scale / 2.0;
  x_axis.pose.position.x = x_position(0);
  x_axis.pose.position.y = x_position(1);
  x_axis.pose.position.z = x_position(2);

  visualization_msgs::Marker y_axis;
  y_axis.ns = ns;
  y_axis.header.frame_id = frame_id;
  y_axis.id = 1;
  y_axis.type = visualization_msgs::Marker::CYLINDER;
  y_axis.scale.x = kDiameter;
  y_axis.scale.y = kDiameter;
  y_axis.scale.z = scale;
  y_axis.color.g = 1.0;
  y_axis.color.a = 1.0;

  // Y axis orientation
  Eigen::Matrix3f y_matrix;
  y_matrix.col(0) = rotation_matrix.col(0);
  y_matrix.col(1) = rotation_matrix.col(2) * -1.0;
  y_matrix.col(2) = rotation_matrix.col(1);
  Eigen::Quaternionf y_quaternion(y_matrix);
  y_axis.pose.orientation.x = y_quaternion.x();
  y_axis.pose.orientation.y = y_quaternion.y();
  y_axis.pose.orientation.z = y_quaternion.z();
  y_axis.pose.orientation.w = y_quaternion.w();

  Eigen::Vector3f y_position =
      Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z) +
      y_matrix.col(2) * scale / 2.0;
  y_axis.pose.position.x = y_position(0);
  y_axis.pose.position.y = y_position(1);
  y_axis.pose.position.z = y_position(2);

  visualization_msgs::Marker z_axis;
  z_axis.ns = ns;
  z_axis.header.frame_id = frame_id;
  z_axis.id = 2;
  z_axis.type = visualization_msgs::Marker::CYLINDER;
  z_axis.scale.x = kDiameter;
  z_axis.scale.y = kDiameter;
  z_axis.scale.z = scale;
  z_axis.color.b = 1.0;
  z_axis.color.a = 1.0;

  z_axis.pose = pose;
  Eigen::Vector3f z_position =
      Eigen::Vector3f(pose.position.x, pose.position.y, pose.position.z) +
      rotation_matrix.col(2) * scale / 2.0;
  z_axis.pose.position.x = z_position(0);
  z_axis.pose.position.y = z_position(1);
  z_axis.pose.position.z = z_position(2);

  visualization_msgs::MarkerArray result;
  result.markers.push_back(x_axis);
  result.markers.push_back(y_axis);
  result.markers.push_back(z_axis);
  return result;
}
}  // namespace rapid
