#ifndef _RAPID_VIZ_SCENE_VIZ_H_
#define _RAPID_VIZ_SCENE_VIZ_H_

#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"

namespace rapid {
namespace viz {
// Visualizes a scene, which is just a PointCloud2.
class SceneViz {
 public:
  SceneViz(const ros::Publisher& scene_pub);
  void set_scene(const sensor_msgs::PointCloud2& cloud);
  void Clear();

 private:
  ros::Publisher scene_pub_;
};
}  // namespace viz
}  // namespace rapid

#endif  // _RAPID_VIZ_SCENE_VIZ_H_
