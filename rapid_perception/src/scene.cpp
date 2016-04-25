#include "rapid_perception/scene.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
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
geometry_msgs::PoseStamped Scene::pose() { return pose_; }
HSurface Scene::primary_surface() const { return primary_surface_; }
geometry_msgs::Vector3 Scene::scale() { return scale_; }
void Scene::set_cloud(const PointCloud<PointXYZRGB>::ConstPtr& cloud) {
  cloud_ = cloud;
}
void Scene::set_pose(const geometry_msgs::PoseStamped& ps) { pose_ = ps; }
void Scene::set_primary_surface(const HSurface& surface) {
  primary_surface_ = surface;
}
void Scene::set_scale(const geometry_msgs::Vector3& scale) { scale_ = scale; }

bool Scene::GetObject(const std::string& name, Object* object) {
  return primary_surface_.GetObject(name, object);
}
}  // namespace rapid
}  // namespace perception
