#include "rapid_perception/object.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/rgbd.hpp"

using geometry_msgs::PoseStamped;
using geometry_msgs::Vector3;
using pcl::PointCloud;
using pcl::PointIndices;
using pcl::PointXYZRGB;
using std::string;

namespace rapid {
namespace perception {
Object::Object() : cloud_(), indices_(), name_(), pose_stamped_(), scale_() {}

PointCloud<PointXYZRGB>::Ptr Object::GetCloud() const {
  return IndicesToCloud<PointXYZRGB>(cloud_, indices_);
}

void Object::SetCloud(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                      const PointIndices::ConstPtr& indices) {
  cloud_ = cloud;
  indices_ = indices;
}

string Object::name() const { return name_; }
PoseStamped Object::pose() const { return pose_stamped_; }
Vector3 Object::scale() const { return scale_; }

void Object::set_name(const string& name) { name_ = name; }
void Object::set_pose(const PoseStamped& ps) { pose_stamped_ = ps; }
void Object::set_scale(const Vector3& scale) { scale_ = scale; }

// Object::Object(Scene* scene, const PointIndices::Ptr& indices)
//    : scene_(scene),
//      indices_(indices),
//      primitive_(),
//      obj_marker_(rapid::viz::Marker::Box(
//          scene->viz_pub(), primitive_.pose_stamped(), primitive_.scale())),
//      text_marker_(rapid::viz::Marker::Text(scene->viz_pub(),
//                                            primitive_.pose_stamped(),
//                                            primitive_.name(), 0.03)) {
//  geometry_msgs::Pose pose;
//  PointCloud<PointXYZRGB>::Ptr cloud = GetCloud();
//  geometry_msgs::Vector3 scale;
//  GetPlanarBoundingBox(*cloud, &pose, &scale);
//  geometry_msgs::PoseStamped ps;
//  ps.header.frame_id = cloud->header.frame_id;
//  ps.pose = pose;
//  primitive_.set_pose_stamped(ps);
//  primitive_.set_scale(scale);
//}

// void Object::Visualize() {
//  obj_marker_ = rapid::viz::Marker::Box(
//      scene_->viz_pub(), primitive_.pose_stamped(), primitive_.scale());
//  obj_marker_.SetNamespace(primitive_.name());
//  obj_marker_.SetColor(0, 0, 1, 0.9);
//  obj_marker_.Publish();
//
//  text_marker_ = rapid::viz::Marker::Text(
//      scene_->viz_pub(), primitive_.pose_stamped(), primitive_.name(), 0.03);
//  text_marker_.SetNamespace(primitive_.name());
//  text_marker_.SetColor(1, 1, 1, 1);
//  text_marker_.Publish();
//}

}  // namespace perception
}  // namespace rapid
