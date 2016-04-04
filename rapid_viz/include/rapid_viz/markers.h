#ifndef _RAPID_VIZ_MARKERS_H_
#define _RAPID_VIZ_MARKERS_H_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros.h"

namespace rapid {
namespace viz {
// A resource-managing marker that deletes itself from rviz when the object goes
// out of scope or is destroyed.
//
// Usage:
//  Marker m = Marker::Box(marker_pub, pose, scale);
//  m.SetId("object", 0);
//  m.SetColor(1, 0, 0);
//  m.Publish(); // Publishes the marker
//  // When m goes out of scope, the rviz marker is deleted.
class Marker {
 public:
  ~Marker();

  // Custom copy constructor, generates a new random ID.
  Marker(const Marker& rhs);
  Marker& operator=(const Marker& rhs);

  // Create a marker of a specific shape.
  static Marker Box(const ros::Publisher& pub,
                    const geometry_msgs::PoseStamped& pose,
                    const geometry_msgs::Vector3& scale);
  static Marker Text(const ros::Publisher& pub,
                     const geometry_msgs::PoseStamped& pose,
                     const std::string& text, double size);

  // Set general properties of the marker.
  void SetNamespace(const std::string& ns);
  void SetColor(double r, double g, double b, double a = 0.9);

  void Publish();
  visualization_msgs::Marker marker() const;  // Returns the marker.

  // Methods to set fields of the marker directly.
  void set_type(uint8_t type) { marker_.type = type; }
  void set_namespace(const std::string& ns) { marker_.ns = ns; }
  void set_pose(const geometry_msgs::PoseStamped& ps) {
    marker_.header = ps.header;
    marker_.pose = ps.pose;
  }
  void set_scale(const geometry_msgs::Vector3& scale) { marker_.scale = scale; }
  void set_text(const std::string& text) { marker_.text = text; }

 private:
  explicit Marker(const ros::Publisher& pub);
  ros::Publisher pub_;
  visualization_msgs::Marker marker_;
};
}  // namespace viz
}  // namespace rapid
#endif  // _RAPID_VIZ_MARKERS_H_
