#ifndef _RAPID_VIZ_MARKERS_H_
#define _RAPID_VIZ_MARKERS_H_

#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "ros/ros.h"

#include "rapid_ros/publisher.h"

namespace rapid {
namespace viz {
visualization_msgs::Marker OutlineBox(const geometry_msgs::PoseStamped& pose,
                                      const geometry_msgs::Vector3& scale);

typedef rapid_ros::PublisherInterface<visualization_msgs::Marker> MarkerPub;

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

  // Create a box marker with the given pose and scale.
  static Marker Box(const MarkerPub* pub,
                    const geometry_msgs::PoseStamped& pose,
                    const geometry_msgs::Vector3& scale);

  // Like a box marker, but only draws the edges of the box.
  static Marker OutlineBox(const MarkerPub* pub,
                           const geometry_msgs::PoseStamped& pose,
                           const geometry_msgs::Vector3& scale);

  // Creates a mesh marker.
  // uri is the URI-form used by the resource_retriever ROS package.
  // E.g., package://pr2_description/meshes/base_v0/base.dae
  // color is a color to use, if desired. Leave it as all zeros to use the
  // mesh's color, or specify a non-zero value to override the mesh's color.
  static Marker Mesh(const MarkerPub* pub, const geometry_msgs::PoseStamped& ps,
                     const std::string& uri);
  static Marker Mesh(const MarkerPub* pub, const geometry_msgs::PoseStamped& ps,
                     const std::string& uri, const std_msgs::ColorRGBA& color);

  // Create a text marker with the given pose. The size is the size of a capital
  // 'A,' in  meters.
  static Marker Text(const MarkerPub* pub,
                     const geometry_msgs::PoseStamped& pose,
                     const std::string& text, double size);

  // Create an arrow marker with origin at ps and direction vec. Both are
  // expected to be in the frame of frame_id.
  static Marker Vector(const MarkerPub* pub, const std::string& frame_id,
                       const geometry_msgs::Point& origin,
                       const geometry_msgs::Vector3& vector);

  // Null marker, for initialization purposes.
  static Marker Null();

  // Publish the marker to the publisher that was passed in.
  void Publish();
  void Delete();

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
  explicit Marker(const MarkerPub* pub);
  const MarkerPub* pub_;
  visualization_msgs::Marker marker_;
};
}  // namespace viz
}  // namespace rapid
#endif  // _RAPID_VIZ_MARKERS_H_
