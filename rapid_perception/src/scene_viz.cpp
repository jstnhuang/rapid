#include "rapid_perception/scene_viz.h"

#include "visualization_msgs/Marker.h"
#include "ros/ros.h"

#include "rapid_perception/hsurface.h"
#include "rapid_perception/object.h"
#include "rapid_perception/scene.h"
#include "rapid_viz/markers.h"

namespace rapid {
namespace perception {
ObjectViz::ObjectViz(const ros::Publisher& pub)
    : object_(), pub_(pub), marker_(), text_marker_() {}

void ObjectViz::set_object(const Object& object) { object_ = object; }

void ObjectViz::Visualize() {
  marker_ = rapid::viz::Marker::Box(pub_, object_.pose(), object_.scale());
  marker_.SetNamespace(object_.name());
  marker_.SetColor(0, 0, 1, 0.9);
  marker_.Publish();

  text_marker_ =
      rapid::viz::Marker::Text(pub_, object_.pose(), object_.name(), 0.03);
  text_marker_.SetNamespace(object_.name());
  text_marker_.SetColor(1, 1, 1, 1);
  text_marker_.Publish();
}

HSurfaceViz::HSurfaceViz(const ros::Publisher& pub)
    : surface_(), pub_(pub), marker_(), objects_() {}

void HSurfaceViz::set_hsurface(const HSurface& hsurface) {
  surface_ = hsurface;
}

void HSurfaceViz::Visualize() {
  marker_ = rapid::viz::Marker::Box(pub_, surface_.pose(), surface_.scale());
  marker_.SetNamespace(surface_.name());
  marker_.SetColor(0, 1, 0, 0.5);
  marker_.Publish();

  objects_.clear();
  for (size_t i = 0; i < surface_.objects().size(); ++i) {
    const Object& obj = surface_.objects()[i];
    ObjectViz oviz(pub_);
    oviz.set_object(obj);
    objects_.push_back(oviz);
  }
  for (size_t i = 0; i < objects_.size(); ++i) {
    ObjectViz& viz = objects_[i];
    viz.Visualize();
  }
}

SceneViz::SceneViz(const ros::Publisher& pub)
    : scene_(), pub_(pub), marker_(), hsurface_viz_(pub) {}

void SceneViz::set_scene(const Scene& scene) { scene_ = scene; }

void SceneViz::Visualize() {
  marker_ = rapid::viz::Marker::OutlineBox(pub_, scene_.pose(), scene_.scale());
  marker_.Publish();

  const HSurface& surface = scene_.primary_surface();
  hsurface_viz_.set_hsurface(surface);
  hsurface_viz_.Visualize();
}

}  // namespace perception
}  // namespace rapid
