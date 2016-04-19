#include "rapid_viz/markers.h"

#include <string>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_utils/math.h"

using std::string;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;

namespace rapid {
namespace viz {
// Default constructor.
// Do not use unless you are making a copy of the marker.
Marker::Marker() : pub_(), marker_() {}

Marker Marker::Box(const ros::Publisher& pub,
                   const geometry_msgs::PoseStamped& pose,
                   const geometry_msgs::Vector3& scale) {
  Marker m(pub);
  m.SetType(visualization_msgs::Marker::CUBE);
  m.SetPose(pose);
  m.SetScale(scale);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

Marker Marker::Text(const ros::Publisher& pub,
                    const geometry_msgs::PoseStamped& pose,
                    const std::string& text, double size) {
  Marker m(pub);
  m.SetType(visualization_msgs::Marker::TEXT_VIEW_FACING);
  m.SetPose(pose);
  geometry_msgs::Vector3 scale;
  scale.z = size;
  m.SetScale(scale);
  m.SetText(text);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

Marker Marker::Vector(const ros::Publisher& pub, const string& frame_id,
                      const Point& origin, const Vector3& vector) {
  Marker m(pub);
  m.SetType(visualization_msgs::Marker::ARROW);
  m.SetFrame(frame_id);

  std::vector<Point> points;
  points.push_back(origin);

  double norm = rapid::utils::Norm(vector);
  Point end = origin;
  end.x += vector.x * 0.03 / norm;
  end.y += vector.y * 0.03 / norm;
  end.z += vector.z * 0.03 / norm;
  points.push_back(end);
  m.SetPoints(points);

  Vector3 scale;
  scale.x = 0.005;
  scale.y = 0.01;
  m.SetScale(scale);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

void Marker::Publish() {
  marker_.action = visualization_msgs::Marker::ADD;
  pub_.publish(marker_);
}

void Marker::SetColor(double r, double g, double b, double a) {
  marker_.color.r = r;
  marker_.color.g = g;
  marker_.color.b = b;
  marker_.color.a = a;
}

void Marker::SetFrame(const std::string& frame_id) {
  marker_.header.frame_id = frame_id;
}

void Marker::SetNamespace(const std::string& ns) { marker_.ns = ns; }

void Marker::SetPoints(const std::vector<geometry_msgs::Point>& points) {
  marker_.points = points;
}

void Marker::SetPose(const geometry_msgs::PoseStamped& ps) {
  marker_.header = ps.header;
  marker_.pose = ps.pose;
}

void Marker::SetScale(const geometry_msgs::Vector3& scale) {
  marker_.scale = scale;
}

void Marker::SetText(const std::string& text) { marker_.text = text; }

void Marker::SetType(uint8_t type) { marker_.type = type; }

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
