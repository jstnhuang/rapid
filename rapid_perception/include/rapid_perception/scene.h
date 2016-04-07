#ifndef _RAPID_PERCEPTION_SCENE_H_
#define _RAPID_PERCEPTION_SCENE_H_

#include <string>
#include <vector>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"

#include "rapid_viz/markers.h"

namespace rapid {
namespace perception {
class Scene;

class ScenePrimitive {
 public:
  ScenePrimitive();
  ScenePrimitive(const geometry_msgs::PoseStamped& pose_stamped,
                 const geometry_msgs::Vector3& scale, const std::string& name);
  geometry_msgs::PoseStamped pose_stamped() const { return pose_stamped_; }
  geometry_msgs::Vector3 scale() const { return scale_; }
  std::string name() const { return name_; }
  void set_pose_stamped(const geometry_msgs::PoseStamped& pose_stamped) {
    pose_stamped_ = pose_stamped;
  }
  void set_scale(const geometry_msgs::Vector3& scale) { scale_ = scale; }
  void set_name(const std::string& name) { name_ = name; }
  bool IsNull() { return name_ == ""; }

 private:
  geometry_msgs::PoseStamped pose_stamped_;
  geometry_msgs::Vector3 scale_;
  std::string name_;
};

class Object {
 public:
  Object(Scene* scene, const pcl::PointIndices::Ptr& indices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() const;
  geometry_msgs::PoseStamped pose() const { return primitive_.pose_stamped(); };
  geometry_msgs::Vector3 scale() const { return primitive_.scale(); };
  std::string name() const {
    return primitive_.name();
    ;
  };
  void set_name(const std::string& name) { primitive_.set_name(name); };
  void Visualize();

 private:
  Scene* scene_;
  pcl::PointIndices::Ptr indices_;
  ScenePrimitive primitive_;

  // Visualization
  ros::Publisher viz_pub_;
  rapid::viz::Marker obj_marker_;
  rapid::viz::Marker text_marker_;
};

void SegmentObjects(Scene* scene, pcl::PointIndices::Ptr indices,
                    double distance_threshold, std::vector<Object>* objects);

// A horizontal plane may cut across the entire scene, so we cluster the plane
// and call the largest cluster the tabletop.
bool FindTabletop(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                  double distance_threshold, pcl::PointIndices::Ptr inliers);

// A tabletop is any horizontal surface that objects can rest on. It is not
// necessarily part of a table.
// Like an rviz marker, the position of a tabletop refers to its center, and its
// scale gives the dimensions in the x, y, and z directions.
class Tabletop {
 public:
  Tabletop(Scene* scene, const pcl::PointIndices::Ptr& indices);
  void AddObject(const Object& object);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() const;
  std::string name() const { return primitive_.name(); };
  geometry_msgs::PoseStamped pose() const { return primitive_.pose_stamped(); };
  geometry_msgs::Vector3 scale() const { return primitive_.scale(); };
  const std::vector<Object>* objects() const { return &objects_; };
  void Visualize();

 private:
  Scene* scene_;
  pcl::PointIndices::Ptr indices_;
  ScenePrimitive primitive_;
  std::vector<Object> objects_;

  // Visualization
  ros::Publisher viz_pub_;
  rapid::viz::Marker marker_;
};

// A Scene is a semantic representation of a point cloud. Given a point cloud,
// it will extract surfaces and objects.
// This class works best when the given point cloud is cropped to a small area
// of interest, such as the robot's current workspace.
class Scene {
 public:
  Scene();
  void set_cloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
  void Parse();
  boost::shared_ptr<Tabletop> GetPrimarySurface() const;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud() const;
  ros::Publisher viz_pub() const { return viz_pub_; }
  void Visualize();

 private:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  boost::shared_ptr<Tabletop> primary_surface_;
  ros::NodeHandle nh_;
  ros::Publisher viz_pub_;
};
}  // namespace rapid
}  // namespace perception
#endif  // _RAPID_PERCEPTION_SCENE_H_
