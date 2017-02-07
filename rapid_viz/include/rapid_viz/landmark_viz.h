#ifndef _RAPID_VIZ_LANDMARK_VIZ_H_
#define _RAPID_VIZ_LANDMARK_VIZ_H_

#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace viz {
class LandmarkViz {
 public:
  LandmarkViz(const ros::Publisher& cloud_pub,
              const ros::Publisher& marker_pub);
  LandmarkViz(const ros::Publisher& cloud_pub, const ros::Publisher& marker_pub,
              const std::string& base_frame);
  void set_landmark(const sensor_msgs::PointCloud2& cloud,
                    const rapid_msgs::Roi3D& roi);
  // Update the pose, which refers to the center of the landmark box.
  // Give the pose in the base frame.
  void UpdatePose(const geometry_msgs::Pose& pose);
  void Publish();
  void Clear();

 private:
  ros::Publisher cloud_pub_;
  std::string base_frame_;

  sensor_msgs::PointCloud2 cloud_;
  rapid_msgs::Roi3D roi_;
  geometry_msgs::Pose pose_;  // Refers to center of landmark box.
  rapid_ros::Publisher<visualization_msgs::Marker> marker_pub_;
  Marker box_marker_;
};
}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_LANDMARK_VIZ_H_
