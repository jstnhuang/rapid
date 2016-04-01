#include "rapid_viz/markers.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"

#include "ros/ros.h"

using std::string;
using visualization_msgs::Marker;

namespace rapid {
namespace viz {
void PublishMarker(const visualization_msgs::Marker& marker) {
  ros::NodeHandle nh;
  ros::Publisher vis_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  vis_pub.publish(marker);
}

void BoundingBoxMarker(const geometry_msgs::PoseStamped& pose,
                       const geometry_msgs::Vector3& scale, Marker* marker) {
  marker->header = pose.header;
  marker->type = Marker::CUBE;
  marker->action = Marker::ADD;
  marker->pose = pose.pose;
  marker->scale = scale;
}

void TextMarker(const geometry_msgs::PoseStamped& pose, const string& text,
                double size, Marker* marker) {
  marker->header = pose.header;
  marker->type = Marker::TEXT_VIEW_FACING;
  marker->action = Marker::ADD;
  marker->pose = pose.pose;
  marker->text = text;
  marker->scale.z = size;
}

void SetMarkerId(const string& ns, int id, Marker* marker) {
  marker->ns = ns;
  marker->id = id;
}

void SetMarkerColor(double r, double g, double b, double a, Marker* marker) {
  marker->color.r = r;
  marker->color.g = g;
  marker->color.b = b;
  marker->color.a = a;
}
}  // namespace viz
}  // namespace rapid
