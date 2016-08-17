#include "rapid_perception/object.h"

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/rgbd.h"

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
  return IndicesToCloud(cloud_, indices_);
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
}  // namespace perception
}  // namespace rapid
