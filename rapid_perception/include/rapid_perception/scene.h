#ifndef _RAPID_PERCEPTION_SCENE_H_
#define _RAPID_PERCEPTION_SCENE_H_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/hsurface.h"

namespace rapid {
namespace perception {
// A Scene is a semantic representation of a point cloud.
//
// Currently, it models a single horizontal surface (HSurface) with objects on
// top of it.
class Scene {
 public:
  Scene();

  // Getters/setters.
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud() const;
  geometry_msgs::PoseStamped pose();
  HSurface primary_surface() const;
  geometry_msgs::Vector3 scale();
  void set_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
  void set_pose(const geometry_msgs::PoseStamped& ps);
  void set_primary_surface(const HSurface& surface);
  void set_scale(const geometry_msgs::Vector3& scale);

  // Look up an object by name. If the object exists, returns it.
  //
  // Returns false if the object is not found, true otherwise.
  bool GetObject(const std::string& name, Object* object);

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
  HSurface primary_surface_;

  // Pose and scale represent the bounding box of the cropped scene.
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::Vector3 scale_;
};
}  // namespace rapid
}  // namespace perception
#endif  // _RAPID_PERCEPTION_SCENE_H_
