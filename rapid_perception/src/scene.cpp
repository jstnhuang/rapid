#include "rapid_perception/scene.h"

#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/hsurface.h"

using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;

namespace rapid {
namespace perception {
Scene::Scene() : cloud_(), primary_surface_() {}

PointCloud<PointXYZRGB>::ConstPtr Scene::cloud() const { return cloud_; }
HSurface Scene::primary_surface() const { return primary_surface_; }
void Scene::set_cloud(const PointCloud<PointXYZRGB>::ConstPtr& cloud) {
  cloud_ = cloud;
}
void Scene::set_primary_surface(const HSurface& surface) {
  primary_surface_ = surface;
}

bool Scene::GetObject(const std::string& name, Object* object) {
  return primary_surface_.GetObject(name, object);
}
}  // namespace rapid
}  // namespace perception
