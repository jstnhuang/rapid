#ifndef _RAPID_PERCEPTION_FPFH_HEAT_MAPPER_H_
#define _RAPID_PERCEPTION_FPFH_HEAT_MAPPER_H_

#include "Eigen/Core"
#include "pcl/features/fpfh_omp.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "ros/ros.h"

#include "rapid_perception/pose_estimation_heat_mapper.h"

namespace rapid {
namespace perception {
// Heat mapper that uses FPFH features on the point cloud.
class FpfhHeatMapper : public PoseEstimationHeatMapper {
 public:
  FpfhHeatMapper();

  void Compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr heatmap,
               Eigen::VectorXd* importances);

  void set_scene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene);
  void set_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object);

  void set_sample_ratio(double val);
  void set_max_samples(int val);
  void set_max_sample_radius(double val);
  void set_max_neighbors(int val);
  void set_feature_threshold(double val);

 private:
  void ComputeNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  void ComputeFeatures(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr in,
                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr out);
  // Source and target data structures
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_scene_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene_;
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr scene_tree_;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object_;
  double object_radius_est_;  // Object radius estimate.
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features_;
  pcl::search::KdTree<pcl::FPFHSignature33>::Ptr object_features_tree_;

  // Parameters
  double normal_radius_;   // Radius to use when computing normals
  double feature_radius_;  // Radius to use when computing features
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
};

// Convert a feature to a string for debugging purposes.
std::string FpfhFeatureString(const pcl::FPFHSignature33& feature);
}  // namespace perception
}  // namespace rapid

#endif  // _RAPID_PERCEPTION_FPFH_HEAT_MAPPER_H_
