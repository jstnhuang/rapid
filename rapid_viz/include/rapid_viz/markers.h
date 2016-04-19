#ifndef _RAPID_VIZ_MARKERS_H_
#define _RAPID_VIZ_MARKERS_H_

#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
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
  // Default constructor.
  // Do not use unless you are making a copy of the marker.
  Marker();
  ~Marker();

  // Custom copy constructor, generates a new random ID.
  Marker(const Marker& rhs);
  Marker& operator=(const Marker& rhs);

  // Create a box marker with the given pose and scale.
  static Marker Box(const ros::Publisher& pub,
                    const geometry_msgs::PoseStamped& pose,
                    const geometry_msgs::Vector3& scale);

  // Create a text marker with the given pose. The size is the size of a capital
  // 'A,' in  meters.
  static Marker Text(const ros::Publisher& pub,
                     const geometry_msgs::PoseStamped& pose,
                     const std::string& text, double size);

  // Create an arrow marker with origin at ps and direction vec. Both are
  // expected to be in the frame of frame_id.
  static Marker Vector(const ros::Publisher& pub, const std::string& frame_id,
                       const geometry_msgs::Point& origin,
                       const geometry_msgs::Vector3& vector);

  // Publish the marker to the publisher that was passed in.
  void Publish();

  // Set properties of the marker.
  void SetColor(double r, double g, double b, double a = 0.9);
  void SetFrame(const std::string& frame_id);
  void SetNamespace(const std::string& ns);
  void SetPoints(const std::vector<geometry_msgs::Point>& points);
  void SetPose(const geometry_msgs::PoseStamped& ps);
  void SetScale(const geometry_msgs::Vector3& scale);
  void SetText(const std::string& text);
  void SetType(uint8_t type);

  visualization_msgs::Marker marker() const;  // Returns the marker.

 private:
  explicit Marker(const ros::Publisher& pub);
  ros::Publisher pub_;
  visualization_msgs::Marker marker_;
};
}  // namespace viz
}  // namespace rapid
#endif  // _RAPID_VIZ_MARKERS_H_