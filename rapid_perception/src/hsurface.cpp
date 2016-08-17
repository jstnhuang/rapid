#include "rapid_perception/hsurface.h"

#include <string>
#include <vector>

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
using std::vector;

namespace rapid {
namespace perception {
HSurface::HSurface()
    : cloud_(), indices_(), name_(), pose_stamped_(), scale_(), objects_() {}

void HSurface::AddObject(const Object& obj) { objects_.push_back(obj); }

PointCloud<PointXYZRGB>::Ptr HSurface::GetCloud() const {
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  return IndicesToCloud(cloud_, indices_);
}

void HSurface::SetCloud(const PointCloud<PointXYZRGB>::ConstPtr& cloud,
                        const PointIndices::ConstPtr& indices) {
  cloud_ = cloud;
  indices_ = indices;
}

bool HSurface::GetObject(const std::string& name, Object* object) {
  // TODO(jstn): Could search with an index.
  for (size_t i = 0; i < objects_.size(); ++i) {
    const Object& obj = objects_[i];
    if (obj.name() == name) {
      *object = obj;
      return true;
    }
  }
  return false;
}

string HSurface::name() const { return name_; }
PoseStamped HSurface::pose() const { return pose_stamped_; }
Vector3 HSurface::scale() const { return scale_; }
std::vector<Object> HSurface::objects() const { return objects_; }

void HSurface::set_name(const string& name) { name_ = name; }
void HSurface::set_pose(const PoseStamped& ps) { pose_stamped_ = ps; }
void HSurface::set_scale(const Vector3& scale) { scale_ = scale; }
}  // namespace perception
}  // namespace rapid
