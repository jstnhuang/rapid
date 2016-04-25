#ifndef _RAPID_PERCEPTION_SCENE_VIZ_H_
#define _RAPID_PERCEPTION_SCENE_VIZ_H_

#include <vector>

#include "ros/ros.h"

#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_perception/scene.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace perception {
class ObjectViz {
 public:
  explicit ObjectViz(const ros::Publisher& pub);
  void set_object(const Object& object);
  void Visualize();

 private:
  Object object_;
  ros::Publisher pub_;
  rapid::viz::Marker marker_;
  rapid::viz::Marker text_marker_;
};

class HSurfaceViz {
 public:
  explicit HSurfaceViz(const ros::Publisher& pub);
  void set_hsurface(const HSurface& hsurface);
  void Visualize();

 private:
  HSurface surface_;
  ros::Publisher pub_;
  rapid::viz::Marker marker_;
  std::vector<ObjectViz> objects_;
};

class SceneViz {
 public:
  explicit SceneViz(const ros::Publisher& pub);
  void set_scene(const Scene& scene);
  void Visualize();

 private:
  Scene scene_;
  ros::Publisher pub_;
  rapid::viz::Marker marker_;
  HSurfaceViz hsurface_viz_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_SCENE_VIZ_H_
