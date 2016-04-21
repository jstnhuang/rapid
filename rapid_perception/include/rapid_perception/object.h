#ifndef _RAPID_PERCEPTION_OBJECT_H_
#define _RAPID_PERCEPTION_OBJECT_H_

#include <string>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace rapid {
namespace perception {
// Represents an object that can be picked/placed.
class Object {
 public:
  // Default constructor.
  Object();

  // Get a PointCloud representation of this Object.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() const;

  // Set the point cloud containing this Object, and the indices of that point
  // cloud representing this Object.
  void SetCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                const pcl::PointIndices::ConstPtr& indices);

  // Getters / setters.
  std::string name() const;
  geometry_msgs::PoseStamped pose() const;
  geometry_msgs::Vector3 scale() const;

  void set_name(const std::string& name);
  void set_pose(const geometry_msgs::PoseStamped& ps);
  void set_scale(const geometry_msgs::Vector3& scale);

  // void Visualize();

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
  pcl::PointIndices::ConstPtr indices_;

  std::string name_;  // Name of this HSurface, e.g., "table"
  geometry_msgs::PoseStamped pose_stamped_;  // Pose of table's center.
  geometry_msgs::Vector3 scale_;  // Dimensions along x, y, and z, in meters.
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_OBJECT_H_
