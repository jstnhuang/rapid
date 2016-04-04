#include "rapid_viz/markers.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"

#include "ros/ros.h"

using std::string;

namespace rapid {
namespace viz {
Marker Marker::Box(const ros::Publisher& pub,
                   const geometry_msgs::PoseStamped& pose,
                   const geometry_msgs::Vector3& scale) {
  Marker m(pub);
  m.set_type(visualization_msgs::Marker::CUBE);
  m.set_pose(pose);
  m.set_scale(scale);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

Marker Marker::Text(const ros::Publisher& pub,
                    const geometry_msgs::PoseStamped& pose,
                    const std::string& text, double size) {
  Marker m(pub);
  m.set_type(visualization_msgs::Marker::TEXT_VIEW_FACING);
  m.set_pose(pose);
  geometry_msgs::Vector3 scale;
  scale.z = size;
  m.set_scale(scale);
  m.set_text(text);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

void Marker::SetNamespace(const std::string& ns) { marker_.ns = ns; }

void Marker::SetColor(double r, double g, double b, double a) {
  marker_.color.r = r;
  marker_.color.g = g;
  marker_.color.b = b;
  marker_.color.a = a;
}

void Marker::Publish() {
  marker_.action = visualization_msgs::Marker::ADD;
  pub_.publish(marker_);
}

visualization_msgs::Marker Marker::marker() const { return marker_; }

Marker::Marker(const ros::Publisher& pub) : pub_(pub), marker_() {}

Marker::Marker(const Marker& rhs) : pub_(rhs.pub_), marker_(rhs.marker_) {
  marker_.id = rand();
}

Marker& Marker::operator=(const Marker& rhs) {
  pub_ = rhs.pub_;
  marker_ = rhs.marker_;
  marker_.id = rand();
  return *this;
}

Marker::~Marker() {
  marker_.action = visualization_msgs::Marker::DELETE;
  pub_.publish(marker_);
}
}  // namespace viz
}  // namespace rapid
