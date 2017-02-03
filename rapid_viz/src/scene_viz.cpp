#include "rapid_viz/scene_viz.h"

#include "rapid_viz/publish.h"
#include "ros/publisher.h"

namespace rapid {
namespace viz {
SceneViz::SceneViz(const ros::Publisher& scene_pub) : scene_pub_(scene_pub) {}

void SceneViz::set_scene(const sensor_msgs::PointCloud2& cloud) {
  PublishCloud(scene_pub_, cloud);
}

void SceneViz::Clear() { PublishBlankCloud(scene_pub_); }
}  // namespace viz
}  // namespace rapid
