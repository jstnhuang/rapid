#include "rapid_viz/markers.h"

#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_utils/math.h"

using std::string;
using std::vector;
using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;

namespace rapid {
namespace viz {
Marker Marker::Box(const MarkerPub* pub, const geometry_msgs::PoseStamped& pose,
                   const geometry_msgs::Vector3& scale) {
  Marker m(pub);
  m.SetType(visualization_msgs::Marker::CUBE);
  m.SetPose(pose);
  m.SetScale(scale);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);
  return m;
}

Marker Marker::OutlineBox(const MarkerPub* pub,
                          const geometry_msgs::PoseStamped& pose,
                          const geometry_msgs::Vector3& scale) {
  Marker m(pub);
  m.SetType(visualization_msgs::Marker::LINE_LIST);
  geometry_msgs::Vector3 line_scale;
  line_scale.x = 0.01;
  m.SetFrame(pose.header.frame_id);
  m.SetScale(line_scale);
  m.SetNamespace("marker");
  m.SetColor(0, 1, 0);

  double max_x = pose.pose.position.x + scale.x / 2;
  double max_y = pose.pose.position.y + scale.y / 2;
  double max_z = pose.pose.position.z + scale.z / 2;
  double min_x = pose.pose.position.x - scale.x / 2;
  double min_y = pose.pose.position.y - scale.y / 2;
  double min_z = pose.pose.position.z - scale.z / 2;

  vector<Point> points;
  // Top layer
  Point t1;
  t1.x = min_x;
  t1.y = min_y;
  t1.z = max_z;
  Point t2 = t1;
  t2.x = max_x;
  Point t3 = t2;
  t3.y = max_y;
  Point t4 = t3;
  t4.x = min_x;
  points.push_back(t1);
  points.push_back(t2);
  points.push_back(t2);
  points.push_back(t3);
  points.push_back(t3);
  points.push_back(t4);
  points.push_back(t4);
  points.push_back(t1);

  // Bottom layer
  Point b1;
  b1.x = min_x;
  b1.y = min_y;
  b1.z = min_z;
  Point b2 = b1;
  b2.x = max_x;
  Point b3 = b2;
  b3.y = max_y;
  Point b4 = b3;
  b4.x = min_x;
  points.push_back(b1);
  points.push_back(b2);
  points.push_back(b2);
  points.push_back(b3);
  points.push_back(b3);
  points.push_back(b4);
  points.push_back(b4);
  points.push_back(b1);

  // Middle layer
  points.push_back(t1);
  points.push_back(b1);
  points.push_back(t2);
  points.push_back(b2);
  points.push_back(t3);
  points.push_back(b3);
  points.push_back(t4);
  points.push_back(b4);

  m.SetPoints(points);

  return m;
}

Marker Marker::Text(const MarkerPub* pub,
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

Marker Marker::Vector(const MarkerPub* pub, const string& frame_id,
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

Marker Marker::Null() {
  MarkerPub* null_pub = NULL;
  Marker m(null_pub);
  return m;
}

void Marker::Publish() {
  if (pub_ && pub_->IsValid()) {
    marker_.action = visualization_msgs::Marker::ADD;
    pub_->publish(marker_);
  }
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

Marker::Marker(const MarkerPub* pub) : pub_(pub), marker_() {}

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
  if (pub_ && pub_->IsValid()) {
    marker_.action = visualization_msgs::Marker::DELETE;
    pub_->publish(marker_);
  }
}
}  // namespace viz
}  // namespace rapid
