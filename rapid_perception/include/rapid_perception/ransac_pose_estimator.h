#ifndef _RAPID_RANSAC_POSE_ESTIMATOR_H_
#define _RAPID_RANSAC_POSE_ESTIMATOR_H_

#include "boost/shared_ptr.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/recognition/ransac_based/obj_rec_ransac.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "rapid_msgs/Roi3D.h"
#include "rapid_perception/pose_estimation_interface.h"
#include "rapid_perception/pose_estimation_match.h"
#include "rapid_ros/publisher.h"

namespace rapid {
namespace perception {
class RansacPoseEstimator : public PoseEstimationInterface {
 public:
  RansacPoseEstimator();
  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& object);
  void set_roi(const rapid_msgs::Roi3D& roi);
  void Find(std::vector<PoseEstimationMatch>* matches);

  void set_scene_publisher(const ros::Publisher& pub);
  void set_object_publisher(const ros::Publisher& pub);
  void set_output_publisher(const ros::Publisher& pub);
  void set_pose_publisher(const ros::Publisher& pub);
  void set_marker_publisher(const ros::Publisher& pub);

  void set_pair_width(double val);
  void set_voxel_size(double val);
  void set_scene_normal_radius(double val);
  void set_object_normal_radius(double val);
  void set_threshold(double val);
  double pair_width() const;
  double voxel_size() const;

  static const double kDefaultPairWidth;
  static const double kDefaultVoxelSize;
  static const double kDefaultSceneNormalRadius;
  static const double kDefaultObjectNormalRadius;
  static const double kDefaultThreshold;
  static const char kObjectName[];

 private:
  double pair_width_;
  double voxel_size_;
  double scene_normal_radius_;
  double object_normal_radius_;
  double threshold_;

  boost::shared_ptr<pcl::recognition::ObjRecRANSAC> recognizer_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_;
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_object_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_object_in_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_scene_in_;
  rapid_msgs::Roi3D object_roi_;
  ros::Publisher scene_pub_;
  ros::Publisher object_pub_;
  ros::Publisher output_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher marker_pub_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_RANSAC_POSE_ESTIMATOR_H_
