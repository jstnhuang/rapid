#ifndef _RAPID_PERCEPTION_HSURFACE_H_
#define _RAPID_PERCEPTION_HSURFACE_H_

#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rapid_perception/object.h"

namespace rapid {
namespace perception {
// An HSurface is a horizontal surface that objects can rest on.
// Like an rviz marker, the position of an HSurface refers to its center, and
// its scale gives the dimensions in the x, y, and z directions.
class HSurface {
 public:
  HSurface();

  // Add an object to this HSurface.
  void AddObject(const Object& object);

  // Get a PointCloud representation of this HSurface.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() const;

  // Set the point cloud containing this HSurface, and the indices of that point
  // cloud representing this HSurface.
  void SetCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                const pcl::PointIndices::ConstPtr& indices);

  // Finds an object by name. Returns false if the object does not exist.
  bool GetObject(const std::string& name, Object* object);

  // Getters / setters.
  std::string name() const;
  geometry_msgs::PoseStamped pose() const;
  geometry_msgs::Vector3 scale() const;

  void set_name(const std::string& name);
  void set_pose(const geometry_msgs::PoseStamped& ps);
  void set_scale(const geometry_msgs::Vector3& scale);
  std::vector<Object> objects() const;

  // void Visualize();

 private:
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_;
  pcl::PointIndices::ConstPtr indices_;

  std::string name_;  // Name of this HSurface, e.g., "table"
  geometry_msgs::PoseStamped pose_stamped_;  // Pose of table's center.
  geometry_msgs::Vector3 scale_;  // Dimensions along x, y, and z, in meters.

  std::vector<Object> objects_;

  // Visualization
  // ros::Publisher viz_pub_;
  // rapid::viz::Marker marker_;
};

}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_HSURFACE_H_
