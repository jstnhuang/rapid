#ifndef _RAPID_PERCEPTION_POSE_ESTIMATION_H_
#define _RAPID_PERCEPTION_POSE_ESTIMATION_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/features/fpfh_omp.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"
#include "Eigen/Core"

namespace rapid {
namespace perception {
class PoseEstimator {
 public:
  PoseEstimator();
  // Setters for source and target
  void set_scene(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr object);
  void set_scene_features(
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr scene_features);
  void set_object_features(
      pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr object_features);

  // Setters for parameters
  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_max_sample_radius(double val);
  void set_max_neighbors(int val);
  void set_feature_threshold(double val);
  void set_num_candidates(int val);

  void set_debug(bool val);

  // Setters for visualization publishers
  void set_heatmap_publisher(const ros::Publisher& pub);
  void set_initial_publisher(const ros::Publisher& pub);
  void set_candidates_publisher(const ros::Publisher& pub);
  void set_best_publisher(const ros::Publisher& pub);

  // Methods for doing the alignment
  bool Find();
  bool Find2();

 private:
  void PublishCloud(const ros::Publisher& pub,
                    pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);
  std::string FeatureString(const pcl::FPFHSignature33& feature);

  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_;
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr scene_tree_;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_;
  double object_radius_est_;       // Object radius estimate.
  Eigen::Vector3d object_center_;  // Approx center point of object.

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_;

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features_;
  pcl::search::KdTree<pcl::FPFHSignature33>::Ptr object_features_tree_;

  // Parameters
  // The fraction of points in the scene to randomly sample when searching for
  // pose.
  double sample_ratio_;
  // The maximum number of samples to search, regardless of the sample ratio.
  int max_samples_;
  // The maximum radius around each sample to look for neighbors. The system
  // generally looks in a radius roughly the size of the object.
  double max_sample_radius_;
  // The maximum number of neighbors, regardless of the sample radius.
  int max_neighbors_;
  // The threshold in feature distance, below which a feature is considered to
  // be a "close match." For FPFH features, feature distances tend to range from
  // 500 - 8000 (just a rough estimate), and a possibly good threshold is 1500.
  double feature_threshold_;
  // Number of candidate samples to consider for ICP.
  int num_candidates_;

  bool debug_;

  // Visualization publishers
  ros::Publisher heatmap_pub_;
  ros::Publisher initial_pub_;
  ros::Publisher candidates_pub_;
  ros::Publisher best_pub_;
};
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_POSE_ESTIMATION_H_
